#include <Arduino.h>
#include <driver/i2s.h>
#include <Preferences.h>

// =========================================================================
// SOUND FILES IN YOUR SKETCH FOLDER:
// =========================================================================
#include "start_sound.h"  // startSamples[], startSampleCount, startSampleRate
#include "idle_sound.h"   // samples[], sampleCount, sampleRate
#include "rev_sound.h"    // revSamples[], revSampleCount, revSampleRate
#include "knock_sound.h"  // knockSamples[], knockSampleCount, knockSampleRate

// Set to FALSE once you drop horn.h and tick_sound.h into your sketch folder!
#define USE_CUSTOM_HORN_AND_TICK_HEADERS true

#if USE_CUSTOM_HORN_AND_TICK_HEADERS
  #include "horn.h"       // hornSamples[], hornSampleCount, hornSampleRate
  #include "tick_sound.h" // tickSamples[], tickSampleCount, tickSampleRate
#endif

// ================= ESC MODE CONFIGURATION =================
enum EscMode { MODE_FBR = 0, MODE_FB = 1, MODE_FR = 2 };
#define CURRENT_ESC_MODE     MODE_FBR  // Match your Hobbywing 1060 jumper!
#define INVERT_THROTTLE_DIR  false     // Set true if Forward/Reverse are swapped on CH2

// ================= SOUND & TAMIYA AUTO-IDLE CONFIGURATION =================
#define DEFAULT_VOLUME         65      // Startup volume (saved in ESP32 flash once adjusted)
#define MIN_VOLUME             20      // Minimum volume level
#define MAX_VOLUME             170     // Maximum volume level
#define VOLUME_STEP            15      // Volume step per Short Tap of CH5 while CH4 is Left/Right
#define ENABLE_REVERSE_BEEP    true    // Reverse sound restored!
#define AUTO_SHUTOFF_IDLE_MS   60000   // Tamiya Method: Turn off engine after 1 minute (60000ms) idle in Neutral

// ================= BRAKE TAIL-LIGHT DIMMING (PURE DIGITALWRITE) =================
// Higher number = dimmer tail light when Headlight is ON (4 = 25%, 5 = 20%, 6 = 16%)
#define TAIL_DIM_DIVIDER       5

// ================= PIN DEFINITIONS =================
// 1. Audio I2S (PCM5102A)
#define I2S_WS_PIN       25
#define I2S_DIN_PIN      22
#define I2S_BCK_PIN      26

// 2. HotRC CT6A Receiver Inputs (3.3V safe or via voltage divider)
#define RC_THROTTLE_PIN  34   // CH2: Throttle / Brake / Reverse
#define RC_CH3_PIN       4    // CH3: 1 Click = Headlights, 2 Clicks = Hazards, 3 Clicks = Engine/Rev Sound Mute
#define RC_CH4_PIN       35   // CH4: 3-Way Switch (Left Indicator / Off / Right Indicator) + Volume Mode
#define RC_CH5_PIN       32   // CH5: Short Tap = Fog Light (or Vol+/Vol- when CH4 is Right/Left), Hold = Horn
#define RC_CH6_PIN       33   // CH6: 1 Click = Light Bar, 2 Clicks = Reading Light

// 3. Vibration Motor Settings
#define VIB_MOTOR_PIN      27
#define INVERT_VIB_SIGNAL  false  // false = Active-HIGH (3.3V = ON, 0V = OFF)
#define VIB_IDLE_BLOCKS    8      // Low RPM at idle (8 out of 30 blocks = ~26% power)
#define VIB_CRANK_BLOCKS   12     // Starter crank rumble (12 out of 30 blocks = 40% power)
#define VIB_CRUISE_BLOCKS  12     // Normal driving (12 out of 30 blocks = 40% power)
#define VIB_CLIMB_BLOCKS   26     // Steep climb / heavy throttle boost (26/30 = ~87% power)

// 4. Lighting Outputs -> Connect to BC547 / 2N2222 Base Resistors (Active HIGH)
#define PIN_HEADLIGHTS   13   // Headlights + Speedometer + Footwell + Number Plate + Rear Door
#define PIN_BRAKE_LIGHT  14   // 2x 1206 Red Brake Light (Pure digitalWrite: OFF / Dim Tail / 100% Brake)
#define PIN_REV_LIGHT    12   // 2x 1206 Warm White Reverse Light
#define PIN_FOG_LIGHT    15   // 2x 1206 Warm White Fog Light
#define PIN_IND_LEFT     16   // 2x 1206 Yellow Left Indicator
#define PIN_IND_RIGHT    17   // 2x 1206 Yellow Right Indicator
#define PIN_LIGHT_BAR    18   // Light Bar
#define PIN_READING_LGT  19   // 1x 2835 Top Reading Light
#define PIN_DOOR_LIGHTS  2    // Synced strictly with Headlight switch

// ================= AUDIO & THROTTLE TUNING =================
#define I2S_SAMPLE_RATE  22050
#define MAX_THROTTLE_US  230.0f  
#define DEADZONE_US      30   // Clean neutral detection

// ================= ENGINE & DRIVE STATE VARIABLES =================
Preferences prefs;

enum EngineState {
  ENGINE_OFF,
  ENGINE_CRANKING,
  ENGINE_RUNNING,
  ENGINE_SHUTTING_DOWN
};
EngineState currentState = ENGINE_OFF;

enum DriveState {
  DRV_NEUTRAL,
  DRV_FORWARD,
  DRV_BRAKING,         // 1st pull back: Brake lights 100% ON, Engine drops to idle
  DRV_BRAKE_RELEASED,  // Returned to neutral after 1st brake pull (or after reversing): Armed for Reverse!
  DRV_REVERSING        // 2nd, 3rd, 4th... endless pulls back: Reverse lights & Reverse sound ON!
};
DriveState driveState = DRV_NEUTRAL;

uint32_t neutralEnterTime         = 0;
uint32_t lastThrottleActivityTime = 0;
bool     idleAutoShutoff          = true;   // Tamiya Style: Silent at power-on until throttle is pulled
bool     manualSoundMute          = false;  // Toggled by 3 clicks on CH3 (mutes ONLY Engine + Reverse sound)
int      masterVolume             = DEFAULT_VOLUME;

// Neutral Calibration Tracking
bool     neutralCalibrated        = false;
uint8_t  neutralSettleCount       = 0;
uint32_t lastNeutralSampleTime    = 0;

// Volatile ISR variables for 5 RC Channels (with 2-frame glitch rejection)
volatile uint32_t ch2Start = 0, ch3Start = 0, ch4Start = 0, ch5Start = 0, ch6Start = 0;
volatile uint32_t ch2LastRise = 0;
volatile uint16_t rcPulseWidth = 1500, prevCh2Pulse = 1500;
volatile uint16_t ch3Pulse = 1000, prevCh3Pulse = 1000;
volatile uint16_t ch4Pulse = 1500;
volatile uint16_t ch5Pulse = 1000, prevCh5Pulse = 1000;
volatile uint16_t ch6Pulse = 1000, prevCh6Pulse = 1000;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t lastAuxPulseTime = 0;
volatile uint16_t validPulseCount = 0;
volatile bool     ch3SignalSeen = false;

int   neutralPulse    = 1500;
float currentThrottle = 0.0f;
float targetThrottle  = 0.0f;

// Steep Climb / Heavy Load Tracking
uint32_t heavyLoadStartTime = 0;
bool     isClimbingLoad     = false;

// Audio sample playback indices
float    startIdx     = 0.0f;
uint32_t shutdownIdx  = 0;
float    idleIdx      = 0.0f;
float    revIdx       = 0.0f;
float    knockIdx     = 0.0f;
float    hornIdx      = 0.0f;
float    tickIdx      = 0.0f;
bool     playingTick  = false;

int      popTimer        = 0;
uint8_t  vibBlockCounter = 0;
uint8_t  vibSubTick      = 0;
uint8_t  tailDimCounter  = 0;
uint32_t revBeepPhase    = 0;

// ================= LIGHTING & BUTTON STATES =================
bool headlightOn    = false;
bool fogLightOn     = false;
bool lightBarOn     = false;
bool readingLightOn = false;
bool brakeLightOn   = false;
bool revLightOn     = false;
bool doorLightsOn   = false;
bool hornActive     = false;

enum IndState { IND_OFF, IND_LEFT, IND_RIGHT, IND_HAZARD };
IndState indState      = IND_OFF;
bool     hazardLatched = false;
bool     indBlinkPhase = false;
uint32_t lastBlinkTime = 0;

// ================= VIBRATION MOTOR HELPER =================
inline void setVibMotor(bool turnOn) {
  bool pinHigh = INVERT_VIB_SIGNAL ? !turnOn : turnOn;
  digitalWrite(VIB_MOTOR_PIN, pinHigh ? HIGH : LOW);
}

inline void updateVibPWM(uint8_t activeBlocks) {
  vibSubTick = (vibSubTick + 1) & 1;
  if (vibSubTick == 0) {
    vibBlockCounter = (vibBlockCounter + 1) % 30;
  }
  setVibMotor(vibBlockCounter < activeBlocks);
}

// ================= INTERRUPT SERVICE ROUTINES =================
void IRAM_ATTR rcThrottleISR() {
  uint32_t now = micros();
  if (digitalRead(RC_THROTTLE_PIN) == HIGH) {
    uint32_t period = now - ch2LastRise;
    ch2LastRise = now;
    if (period >= 12000 && period <= 28000) {
      ch2Start = now;
    } else {
      ch2Start = 0;
      if (validPulseCount > 0) validPulseCount--;
    }
  } else if (ch2Start != 0) {
    uint32_t diff = now - ch2Start;
    ch2Start = 0;
    if (diff >= 900 && diff <= 2100) {
      int delta = abs((int)diff - (int)prevCh2Pulse);
      prevCh2Pulse = (uint16_t)diff;
      if (delta <= 50) {
        rcPulseWidth = (uint16_t)diff;
        lastPulseTime = millis();
        if (validPulseCount < 100) validPulseCount++;
      }
    } else {
      if (validPulseCount > 0) validPulseCount--;
    }
  }
}

void IRAM_ATTR rcCh3ISR() {
  uint32_t now = micros();
  if (digitalRead(RC_CH3_PIN) == HIGH) {
    ch3Start = now;
  } else if (ch3Start != 0) {
    uint32_t diff = now - ch3Start;
    ch3Start = 0;
    if (diff >= 900 && diff <= 2100) {
      int delta = abs((int)diff - (int)prevCh3Pulse);
      prevCh3Pulse = (uint16_t)diff;
      if (delta <= 80) {
        ch3Pulse = (uint16_t)diff;
        ch3SignalSeen = true;
        lastAuxPulseTime = millis();
      }
    }
  }
}

void IRAM_ATTR rcCh4ISR() {
  uint32_t now = micros();
  if (digitalRead(RC_CH4_PIN) == HIGH) {
    ch4Start = now;
  } else if (ch4Start != 0) {
    uint32_t diff = now - ch4Start;
    ch4Start = 0;
    if (diff >= 900 && diff <= 2100) {
      ch4Pulse = (uint16_t)diff;
    }
  }
}

void IRAM_ATTR rcCh5ISR() {
  uint32_t now = micros();
  if (digitalRead(RC_CH5_PIN) == HIGH) {
    ch5Start = now;
  } else if (ch5Start != 0) {
    uint32_t diff = now - ch5Start;
    ch5Start = 0;
    if (diff >= 900 && diff <= 2100) {
      int delta = abs((int)diff - (int)prevCh5Pulse);
      prevCh5Pulse = (uint16_t)diff;
      if (delta <= 80) {
        ch5Pulse = (uint16_t)diff;
        lastAuxPulseTime = millis();
      }
    }
  }
}

void IRAM_ATTR rcCh6ISR() {
  uint32_t now = micros();
  if (digitalRead(RC_CH6_PIN) == HIGH) {
    ch6Start = now;
  } else if (ch6Start != 0) {
    uint32_t diff = now - ch6Start;
    ch6Start = 0;
    if (diff >= 900 && diff <= 2100) {
      int delta = abs((int)diff - (int)prevCh6Pulse);
      prevCh6Pulse = (uint16_t)diff;
      if (delta <= 80) {
        ch6Pulse = (uint16_t)diff;
        lastAuxPulseTime = millis();
      }
    }
  }
}

// ================= I2S SETUP & AUDIO HELPER =================
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

inline float readArrayInterp(const signed char* arr, uint32_t count, float idx) {
  uint32_t i0 = (uint32_t)idx % count;
  uint32_t i1 = (i0 + 1) % count;
  float frac = idx - (uint32_t)idx;
  return (arr[i0] * (1.0f - frac)) + (arr[i1] * frac);
}

// ================= HOBBYWING 1060 F/B/R & ENDLESS REVERSE LOGIC =================
void updateDriveAndThrottle(uint32_t now) {
  int rawDiff = (int)rcPulseWidth - neutralPulse;
  if (INVERT_THROTTLE_DIR) rawDiff = -rawDiff;

  bool isForward = (rawDiff > DEADZONE_US);
  bool isReverse = (rawDiff < -DEADZONE_US);
  bool isNeutral = (!isForward && !isReverse);

  if (!isNeutral) {
    lastThrottleActivityTime = now;
  }

  if (CURRENT_ESC_MODE == MODE_FBR) {
    switch (driveState) {
      case DRV_NEUTRAL:
        if (isForward)      driveState = DRV_FORWARD;
        else if (isReverse) driveState = DRV_BRAKING; // 1st reverse pull -> Brake Light!
        break;

      case DRV_FORWARD:
        if (isReverse) {
          driveState = DRV_BRAKING;                   // 1st reverse pull after Forward -> Brake Light!
        } else if (isNeutral) {
          if (now - neutralEnterTime > 1200) driveState = DRV_NEUTRAL;
        } else {
          neutralEnterTime = now;
        }
        break;

      case DRV_BRAKING:
        if (isForward) {
          driveState = DRV_FORWARD;                   // Forward trigger resets cycle
        } else if (isNeutral) {
          driveState = DRV_BRAKE_RELEASED;            // Released 1st pull -> Armed for endless Reverse!
        }
        break;

      case DRV_BRAKE_RELEASED:
        // Stays armed for Reverse indefinitely until Forward trigger is pulled!
        if (isReverse) {
          driveState = DRV_REVERSING;                 // 2nd, 3rd, 4th... pull -> Reverse Light & Sound!
        } else if (isForward) {
          driveState = DRV_FORWARD;                   // Only Forward trigger resets back to Braking mode!
        }
        break;

      case DRV_REVERSING:
        if (isForward) {
          driveState = DRV_FORWARD;                   // Forward trigger resets back to Forward
        } else if (isNeutral) {
          driveState = DRV_BRAKE_RELEASED;            // Return to DRV_BRAKE_RELEASED so 3rd+ pull is STILL Reverse!
        }
        break;
    }
  } 
  else if (CURRENT_ESC_MODE == MODE_FB) {
    if (isForward)      driveState = DRV_FORWARD;
    else if (isReverse) driveState = DRV_BRAKING;
    else                driveState = DRV_NEUTRAL;
  } 
  else {
    if (isForward)      driveState = DRV_FORWARD;
    else if (isReverse) driveState = DRV_REVERSING;
    else                driveState = DRV_NEUTRAL;
  }

  int absDelta = abs(rawDiff);
  float normInput = constrain((float)(absDelta - DEADZONE_US) / MAX_THROTTLE_US, 0.0f, 1.0f);

  if (driveState == DRV_FORWARD && isForward) {
    brakeLightOn   = false;
    revLightOn     = false;
    targetThrottle = normInput;
  } 
  else if (driveState == DRV_BRAKING) {
    brakeLightOn   = true;
    revLightOn     = false;
    targetThrottle = 0.0f;
  } 
  else if (driveState == DRV_REVERSING && isReverse) {
    brakeLightOn   = false;
    revLightOn     = true;
    targetThrottle = normInput * 0.55f;
  } 
  else {
    brakeLightOn   = (CURRENT_ESC_MODE == MODE_FR);
    revLightOn     = false;
    targetThrottle = 0.0f;
  }

  // --- STEEP CLIMB / HEAVY THROTTLE DETECTION ---
  if (driveState == DRV_FORWARD && targetThrottle > 0.65f) {
    if (heavyLoadStartTime == 0) {
      heavyLoadStartTime = now;
    } else if (now - heavyLoadStartTime > 400) {
      isClimbingLoad = true;
    }
  } else {
    heavyLoadStartTime = 0;
    isClimbingLoad = false;
  }
}

// ================= AUX CHANNELS, VOLUME CONTROL & BLINKER LOGIC =================
void updateAuxChannelsAndLights(uint32_t now, bool signalActive) {
  if (signalActive) {
    // --- 1. CH4: 3-Way Switch (Left Indicator / Off / Right Indicator) ---
    // Read CH4 first so CH5 knows whether to toggle Fog Lights (CH4 Center) or adjust Volume (CH4 Left/Right)
    static IndState rawCh4Prev = IND_OFF;
    static IndState debouncedCh4Ind = IND_OFF;
    static uint32_t ch4StableStart = 0;

    IndState rawCh4 = IND_OFF;
    if (ch4Pulse < 1220)      rawCh4 = IND_LEFT;
    else if (ch4Pulse > 1780) rawCh4 = IND_RIGHT;

    if (rawCh4 != rawCh4Prev) {
      rawCh4Prev = rawCh4;
      ch4StableStart = now;
    } else if ((now - ch4StableStart >= 100) && (rawCh4 != debouncedCh4Ind)) {
      debouncedCh4Ind = rawCh4;
      if (debouncedCh4Ind != IND_OFF) {
        indState = debouncedCh4Ind;
        indBlinkPhase = true;
        lastBlinkTime = now;
        playingTick = true; // Indicator tick ALWAYS works (even when engine is off/muted)
        tickIdx = 0.0f;
      } else {
        indState = hazardLatched ? IND_HAZARD : IND_OFF;
      }
    }

    // --- 2. CH5: Short Press = Fog Light (or Volume Up/Down if CH4 is Right/Left) | Hold = Horn ---
    static bool ch5Initialized   = false;
    static bool ch5Prev          = false;
    static uint32_t ch5ChangeTime = 0;
    static bool ch5HornTriggered = false;

    bool ch5Active = (ch5Pulse > 1650);
    if (!ch5Initialized) {
      ch5Prev = ch5Active;
      ch5Initialized = true;
    } else if (ch5Active != ch5Prev) {
      if (ch5Active) {
        ch5ChangeTime = now;
        ch5HornTriggered = false;
      } else {
        // Button released before Horn threshold (Short Tap)
        if (!ch5HornTriggered && (now - ch5ChangeTime > 40)) {
          if (debouncedCh4Ind == IND_RIGHT) {
            // CH4 is in RIGHT position -> Short Tap CH5 = VOLUME UP!
            masterVolume = constrain(masterVolume + VOLUME_STEP, MIN_VOLUME, MAX_VOLUME);
            prefs.putInt("vol", masterVolume);
            playingTick = true;
            tickIdx = 0.0f;
            Serial.printf("Volume UP (Saved) -> %d\n", masterVolume);
          } 
          else if (debouncedCh4Ind == IND_LEFT) {
            // CH4 is in LEFT position -> Short Tap CH5 = VOLUME DOWN!
            masterVolume = constrain(masterVolume - VOLUME_STEP, MIN_VOLUME, MAX_VOLUME);
            prefs.putInt("vol", masterVolume);
            playingTick = true;
            tickIdx = 0.0f;
            Serial.printf("Volume DOWN (Saved) -> %d\n", masterVolume);
          } 
          else {
            // CH4 is in CENTER (OFF) position -> Short Tap CH5 = Toggle Fog Light!
            fogLightOn = !fogLightOn;
          }
        }
        hornActive = false;
      }
      ch5Prev = ch5Active;
    }

    // Hold CH5 >= 350ms -> Blow Horn (Always works even when engine sound is off/muted!)
    if (ch5Initialized && ch5Active && (now - ch5ChangeTime >= 350)) {
      ch5HornTriggered = true;
      hornActive = true;
    }

    // --- 3. CH3: 1 Click = Headlights | 2 Clicks = Hazards | 3 Clicks = Engine & Reverse Sound Mute ---
    static bool ch3Initialized = false;
    static bool ch3Prev = false;
    static uint8_t ch3Clicks = 0;
    static uint32_t ch3LastEdgeTime = 0;

    if (ch3SignalSeen) {
      bool ch3Active = (ch3Pulse > 1650);
      if (!ch3Initialized) {
        ch3Prev = ch3Active;
        ch3Initialized = true;
      } else if (ch3Active != ch3Prev) {
        if (now - ch3LastEdgeTime > 60) {
          ch3Clicks++;
          ch3LastEdgeTime = now;
        }
        ch3Prev = ch3Active;
      }

      if (ch3Clicks > 0 && (now - ch3LastEdgeTime > 650)) {
        if (ch3Clicks == 1) {
          headlightOn = !headlightOn;
        } else if (ch3Clicks == 2) {
          hazardLatched = !hazardLatched;
          indState = hazardLatched ? IND_HAZARD : debouncedCh4Ind;
          indBlinkPhase = true;
          lastBlinkTime = now;
          if (indState != IND_OFF) {
            playingTick = true; // Hazard tick ALWAYS works!
            tickIdx = 0.0f;
          }
        } else if (ch3Clicks >= 3) {
          manualSoundMute = !manualSoundMute;
          if (manualSoundMute) {
            Serial.println("Engine & Reverse Sound Muted (Horn & Indicators remain active)");
          } else {
            idleAutoShutoff = true; // Wait in Tamiya standby until next throttle pull
            Serial.println("Engine Sound Unmuted (Pull throttle to start engine)");
          }
        }
        ch3Clicks = 0;
      }
    }

    // --- 4. CH6: 1 Click = Light Bar | 2 Clicks = Reading Light ---
    static bool ch6Initialized = false;
    static bool ch6Prev = false;
    static uint8_t ch6Clicks = 0;
    static uint32_t ch6LastEdgeTime = 0;

    bool ch6Active = (ch6Pulse > 1650);
    if (!ch6Initialized) {
      ch6Prev = ch6Active;
      ch6Initialized = true;
    } else if (ch6Active != ch6Prev) {
      if (now - ch6LastEdgeTime > 60) {
        ch6Clicks++;
        ch6LastEdgeTime = now;
      }
      ch6Prev = ch6Active;
    }
    if (ch6Clicks > 0 && (now - ch6LastEdgeTime > 450)) {
      if (ch6Clicks == 1) {
        lightBarOn = !lightBarOn;
      } else if (ch6Clicks >= 2) {
        readingLightOn = !readingLightOn;
      }
      ch6Clicks = 0;
    }
  } else {
    hornActive = false;
  }

  // --- Indicator Blinker Timer (380ms ON / 380ms OFF) & Tick Sound Trigger ---
  // Works just like a real vehicle even when Engine & Reverse sounds are OFF!
  if (indState != IND_OFF) {
    if (now - lastBlinkTime >= 380) {
      lastBlinkTime = now;
      indBlinkPhase = !indBlinkPhase;
      playingTick = true;
      tickIdx = 0.0f;
    }
  } else {
    indBlinkPhase = false;
    playingTick = false;
  }

  // --- Headlight Switch controls ONLY Headlights, Speedo, Footwell, Plate, Rear Door, and Dim Brake ---
  doorLightsOn = headlightOn;

  // --- Brake Light: PURE digitalWrite (Zero analogWrite timer lockups!) ---
  tailDimCounter = (tailDimCounter + 1) % TAIL_DIM_DIVIDER;
  bool brakePinOut = false;
  if (brakeLightOn) {
    brakePinOut = true;
  } else if (headlightOn) {
    brakePinOut = (tailDimCounter == 0);
  } else {
    brakePinOut = false;
  }

  // --- Write States to Transistor Pins ---
  digitalWrite(PIN_HEADLIGHTS,  headlightOn ? HIGH : LOW);
  digitalWrite(PIN_DOOR_LIGHTS, doorLightsOn ? HIGH : LOW);
  digitalWrite(PIN_BRAKE_LIGHT, brakePinOut ? HIGH : LOW);
  digitalWrite(PIN_REV_LIGHT,   revLightOn ? HIGH : LOW);
  digitalWrite(PIN_FOG_LIGHT,   fogLightOn ? HIGH : LOW);
  digitalWrite(PIN_LIGHT_BAR,   lightBarOn ? HIGH : LOW);
  digitalWrite(PIN_READING_LGT, readingLightOn ? HIGH : LOW);

  bool leftOut  = (indState == IND_LEFT || indState == IND_HAZARD) && indBlinkPhase;
  bool rightOut = (indState == IND_RIGHT || indState == IND_HAZARD) && indBlinkPhase;
  digitalWrite(PIN_IND_LEFT,  leftOut ? HIGH : LOW);
  digitalWrite(PIN_IND_RIGHT, rightOut ? HIGH : LOW);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(VIB_MOTOR_PIN, OUTPUT);
  setVibMotor(false);

  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_CH3_PIN,      INPUT_PULLDOWN);
  pinMode(RC_CH4_PIN,      INPUT);
  pinMode(RC_CH5_PIN,      INPUT_PULLDOWN);
  pinMode(RC_CH6_PIN,      INPUT_PULLDOWN);

  const uint8_t lightPins[] = {
    PIN_HEADLIGHTS, PIN_BRAKE_LIGHT, PIN_REV_LIGHT, PIN_FOG_LIGHT,
    PIN_IND_LEFT, PIN_IND_RIGHT, PIN_LIGHT_BAR, PIN_READING_LGT, PIN_DOOR_LIGHTS
  };
  for (uint8_t p : lightPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // Load saved volume from ESP32 NVS Flash
  prefs.begin("rc_sound", false);
  masterVolume = constrain(prefs.getInt("vol", DEFAULT_VOLUME), MIN_VOLUME, MAX_VOLUME);
  Serial.printf("Loaded Saved Volume: %d\n", masterVolume);

  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), rcThrottleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3_PIN),      rcCh3ISR,      CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_PIN),      rcCh4ISR,      CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH5_PIN),      rcCh5ISR,      CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH6_PIN),      rcCh6ISR,      CHANGE);

  setupI2S();
  Serial.println("Waiting for HotRC CT6A & Hobbywing 1060 (Tamiya Silent Standby Mode)...");
}

// ================= MAIN LOOP =================
void loop() {
  uint32_t now = millis();

  bool ch2Locked = (lastPulseTime > 0) && (now - lastPulseTime < 250) && (validPulseCount >= 30);
  bool auxLocked = (lastAuxPulseTime > 0) && (now - lastAuxPulseTime < 350);
  bool signalActive = (now > 1000) && ch2Locked && auxLocked;

  if (now - lastPulseTime >= 250) {
    validPulseCount = 0;
  }

  // Silently calibrate neutral when TX & ESC link up, keeping engine OFF until first throttle pull
  if (!signalActive) {
    neutralCalibrated = false;
    neutralSettleCount = 0;
    idleAutoShutoff = true;
    brakeLightOn = false;
    revLightOn   = false;
  } else {
    if (!neutralCalibrated) {
      if (rcPulseWidth >= 1350 && rcPulseWidth <= 1650 && (now - lastNeutralSampleTime >= 20)) {
        lastNeutralSampleTime = now;
        neutralSettleCount++;
        if (neutralSettleCount >= 20) {
          neutralPulse = rcPulseWidth;
          neutralCalibrated = true;
          idleAutoShutoff = true; // Stay silent until first throttle pull
          Serial.printf("Link Ready! Neutral = %d us. Pull throttle to start engine.\n", neutralPulse);
        }
      }
    } else {
      updateDriveAndThrottle(now);
    }
  }

  updateAuxChannelsAndLights(now, signalActive);

  bool throttleMoved = neutralCalibrated && (abs((int)rcPulseWidth - neutralPulse) > DEADZONE_US);

  // Tamiya Wake-Up: Start cranking only when throttle is pulled!
  if (throttleMoved) {
    lastThrottleActivityTime = now;
    if (idleAutoShutoff) {
      idleAutoShutoff = false;
    }
  }

  // 1-Minute Idle Auto-Shutoff check
  if (currentState == ENGINE_RUNNING && AUTO_SHUTOFF_IDLE_MS > 0) {
    if (!throttleMoved && (int32_t)(now - lastThrottleActivityTime) > (int32_t)AUTO_SHUTOFF_IDLE_MS) {
      Serial.println("1 minute idle reached -> Shutting down engine.");
      idleAutoShutoff = true;
    }
  }

  bool shouldEngineRun = signalActive && neutralCalibrated && !manualSoundMute && !idleAutoShutoff;

  // ================= STATE TRANSITIONS =================
  if (currentState == ENGINE_OFF && shouldEngineRun) {
    Serial.println("Throttle pulled -> Cranking Engine!");
    startIdx = 0.0f;
    currentState = ENGINE_CRANKING;
  } 
  else if (currentState == ENGINE_CRANKING && !shouldEngineRun) {
    currentState = ENGINE_OFF;
    setVibMotor(false);
  } 
  else if (currentState == ENGINE_RUNNING && !shouldEngineRun) {
    if (!signalActive || manualSoundMute) {
      currentState = ENGINE_OFF;
      currentThrottle = 0.0f;
      isClimbingLoad = false;
      setVibMotor(false);
    } else {
      shutdownIdx = 0;
      currentState = ENGINE_SHUTTING_DOWN;
    }
  } 
  else if (currentState == ENGINE_SHUTTING_DOWN) {
    if (!signalActive || manualSoundMute) {
      currentState = ENGINE_OFF;
      setVibMotor(false);
    } else if (shouldEngineRun) {
      currentState = ENGINE_RUNNING;
    }
  }

  // ================= NON-BLOCKING 32-FRAME I2S AUDIO & VIBRATION ENGINE =================
  int16_t buffer[64];
  float startStep = (float)startSampleRate / (float)I2S_SAMPLE_RATE;
  float baseRatio = (float)sampleRate / (float)I2S_SAMPLE_RATE;

  #if USE_CUSTOM_HORN_AND_TICK_HEADERS
    float hornStep = (float)hornSampleRate / (float)I2S_SAMPLE_RATE;
    float tickStep = (float)tickSampleRate / (float)I2S_SAMPLE_RATE;
  #endif

  if (currentState == ENGINE_CRANKING) {
    updateVibPWM(VIB_CRANK_BLOCKS);

    for (int i = 0; i < 32; i++) {
      int32_t mixed = 0;
      if ((uint32_t)startIdx < startSampleCount) {
        mixed = (int32_t)startSamples[(uint32_t)startIdx] * masterVolume;
        startIdx += startStep;
      } else {
        currentThrottle = 0.25f;
        lastThrottleActivityTime = now;
        currentState = ENGINE_RUNNING;
      }

      // Horn and Indicator ticks always active
      if (hornActive) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          hornIdx += hornStep;
          if (hornIdx >= hornSampleCount) hornIdx -= hornSampleCount;
          mixed += (int32_t)(readArrayInterp(hornSamples, hornSampleCount, hornIdx) * masterVolume * 1.25f);
        #endif
      } else {
        hornIdx = 0.0f;
      }

      if (playingTick) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          if ((uint32_t)tickIdx < tickSampleCount) {
            mixed += (int32_t)(tickSamples[(uint32_t)tickIdx] * masterVolume * 1.1f);
            tickIdx += tickStep;
          } else {
            playingTick = false;
          }
        #endif
      }

      mixed = constrain(mixed, -32768, 32767);
      buffer[i * 2]     = (int16_t)mixed;
      buffer[i * 2 + 1] = (int16_t)mixed;
    }
  } 
  else if (currentState == ENGINE_SHUTTING_DOWN) {
    updateVibPWM(VIB_IDLE_BLOCKS);

    for (int i = 0; i < 32; i++) {
      int32_t mixed = 0;
      if (shutdownIdx < 7000) {
        float fade = 1.0f - ((float)shutdownIdx / 7000.0f);
        idleIdx += (0.55f * fade * baseRatio);
        if (idleIdx >= sampleCount) idleIdx -= sampleCount;
        mixed = (int32_t)(readArrayInterp(samples, sampleCount, idleIdx) * fade * masterVolume);
        shutdownIdx++;
      } else {
        currentThrottle = 0.0f;
        isClimbingLoad = false;
        currentState = ENGINE_OFF;
        setVibMotor(false);
      }

      if (hornActive) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          hornIdx += hornStep;
          if (hornIdx >= hornSampleCount) hornIdx -= hornSampleCount;
          mixed += (int32_t)(readArrayInterp(hornSamples, hornSampleCount, hornIdx) * masterVolume * 1.25f);
        #endif
      } else {
        hornIdx = 0.0f;
      }

      if (playingTick) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          if ((uint32_t)tickIdx < tickSampleCount) {
            mixed += (int32_t)(tickSamples[(uint32_t)tickIdx] * masterVolume * 1.1f);
            tickIdx += tickStep;
          } else {
            playingTick = false;
          }
        #endif
      }

      mixed = constrain(mixed, -32768, 32767);
      buffer[i * 2]     = (int16_t)mixed;
      buffer[i * 2 + 1] = (int16_t)mixed;
    }
  } 
  else if (currentState == ENGINE_RUNNING) {
    if (currentThrottle - targetThrottle > 0.30f && popTimer == 0) {
      popTimer = 2800;
    }

    if (targetThrottle > currentThrottle) {
      currentThrottle += (targetThrottle - currentThrottle) * 0.030f;
    } else {
      currentThrottle += (targetThrottle - currentThrottle) * 0.015f;
      if (currentThrottle < 0.01f) currentThrottle = 0.0f;
    }

    uint8_t activeVibBlocks = isClimbingLoad
      ? VIB_CLIMB_BLOCKS
      : (VIB_IDLE_BLOCKS + (uint8_t)(currentThrottle * (VIB_CRUISE_BLOCKS - VIB_IDLE_BLOCKS)));
    updateVibPWM(activeVibBlocks);

    float pitch      = 1.0f + (currentThrottle * 1.15f);
    float revMix     = constrain((currentThrottle - 0.15f) / 0.55f, 0.0f, 1.0f);
    float idleMix    = 1.0f - (revMix * 0.75f);
    float knockBoost = isClimbingLoad ? 1.35f : 1.0f;
    float knockMix   = constrain((targetThrottle - 0.10f) * 1.2f * knockBoost, 0.0f, 1.0f);

    float idleStep  = pitch * baseRatio;
    float revStep   = pitch * ((float)revSampleRate / (float)I2S_SAMPLE_RATE);
    float knockStep = pitch * ((float)knockSampleRate / (float)I2S_SAMPLE_RATE);

    bool revBeepToneOn = ENABLE_REVERSE_BEEP && (driveState == DRV_REVERSING) && ((now % 800) < 400);

    for (int i = 0; i < 32; i++) {
      idleIdx  += idleStep;  if (idleIdx >= sampleCount)       idleIdx  -= sampleCount;
      revIdx   += revStep;   if (revIdx >= revSampleCount)     revIdx   -= revSampleCount;
      knockIdx += knockStep; if (knockIdx >= knockSampleCount) knockIdx -= knockSampleCount;

      float sIdle  = readArrayInterp(samples, sampleCount, idleIdx) * idleMix;
      float sRev   = readArrayInterp(revSamples, revSampleCount, revIdx) * revMix;
      float sKnock = readArrayInterp(knockSamples, knockSampleCount, knockIdx) * knockMix;

      int32_t mixed = (int32_t)((sIdle + sRev + sKnock) * masterVolume);

      if (popTimer > 0) {
        popTimer--;
        if (popTimer % 550 < 90) {
          mixed += (int32_t)(readArrayInterp(knockSamples, knockSampleCount, knockIdx) * masterVolume * 1.4f);
        }
      }

      // --- MIX 1: HORN SOUND (Always active) ---
      if (hornActive) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          hornIdx += hornStep;
          if (hornIdx >= hornSampleCount) hornIdx -= hornSampleCount;
          mixed += (int32_t)(readArrayInterp(hornSamples, hornSampleCount, hornIdx) * masterVolume * 1.25f);
        #endif
      } else {
        hornIdx = 0.0f;
      }

      // --- MIX 2: INDICATOR RELAY TICK (Always active) ---
      if (playingTick) {
        #if USE_CUSTOM_HORN_AND_TICK_HEADERS
          if ((uint32_t)tickIdx < tickSampleCount) {
            mixed += (int32_t)(tickSamples[(uint32_t)tickIdx] * masterVolume * 1.1f);
            tickIdx += tickStep;
          } else {
            playingTick = false;
          }
        #endif
      }

      // --- MIX 3: REVERSE GEAR BEEP (Scaled with masterVolume) ---
      if (revBeepToneOn) {
        revBeepPhase++;
        int16_t beepAmp = (int16_t)(15 * masterVolume);
        int16_t beepSample = ((revBeepPhase % 21) < 10) ? beepAmp : -beepAmp;
        mixed += beepSample;
      }

      mixed = constrain(mixed, -32768, 32767);
      buffer[i * 2]     = (int16_t)mixed;
      buffer[i * 2 + 1] = (int16_t)mixed;
    }
  } 
  else {
    // ENGINE_OFF (Idle sleep or 3-Click Muted):
    // Engine & Reverse sound are OFF, but Horn and Indicator/Hazard Ticks ALWAYS work like a real vehicle!
    setVibMotor(false);

    for (int i = 0; i < 32; i++) {
      int32_t mixed = 0;

      if (signalActive) {
        if (hornActive) {
          #if USE_CUSTOM_HORN_AND_TICK_HEADERS
            hornIdx += hornStep;
            if (hornIdx >= hornSampleCount) hornIdx -= hornSampleCount;
            mixed += (int32_t)(readArrayInterp(hornSamples, hornSampleCount, hornIdx) * masterVolume * 1.25f);
          #endif
        } else {
          hornIdx = 0.0f;
        }

        if (playingTick) {
          #if USE_CUSTOM_HORN_AND_TICK_HEADERS
            if ((uint32_t)tickIdx < tickSampleCount) {
              mixed += (int32_t)(tickSamples[(uint32_t)tickIdx] * masterVolume * 1.1f);
              tickIdx += tickStep;
            } else {
              playingTick = false;
            }
          #endif
        }
      } else {
        hornIdx = 0.0f;
        playingTick = false;
      }

      mixed = constrain(mixed, -32768, 32767);
      buffer[i * 2]     = (int16_t)mixed;
      buffer[i * 2 + 1] = (int16_t)mixed;
    }
  }

  size_t bytesWritten;
  i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
}
