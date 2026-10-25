#include <Arduino.h>
#include <driver/i2s.h>

// =========================================================================
// 4 SOUND FILES IN YOUR SKETCH FOLDER:
// =========================================================================
#include "start_sound.h"  // startSamples[], startSampleCount, startSampleRate
#include "idle_sound.h"   // samples[], sampleCount, sampleRate
#include "rev_sound.h"    // revSamples[], revSampleCount, revSampleRate
#include "knock_sound.h"  // knockSamples[], knockSampleCount, knockSampleRate

// ================= PIN DEFINITIONS =================
#define I2S_WS_PIN       25   // Connect to WS on Black PCM5100A Module
#define I2S_DIN_PIN      22   // Connect to DIN on Black PCM5100A Module
#define I2S_BCK_PIN      26   // Connect to BCK on Black PCM5100A Module
#define RC_THROTTLE_PIN  34   // Connect to HotRC CH2 Signal
#define VIB_MOTOR_PIN    27   // Connect to L293D Pin 2

#define INVERT_VIB_SIGNAL false

// ================= VIBRATION SPEED TUNING =================
// Out of every 30 audio blocks (~87ms total cycle):
// 4 = ~13% power (Extremely slow rotation, keeps MOSFET cool & saves battery)
// If the motor stalls completely, bump to 5 or 6. If still too fast, lower to 3.
#define RUNNING_VIB_PULSE 4   
#define VIB_CYCLE_TOTAL   30  

#define I2S_SAMPLE_RATE  22050
#define MASTER_VOLUME    170  // Overall loudness (150 to 220)

// ================= THROTTLE TUNING =================
#define MAX_THROTTLE_US  230.0f  
#define DEADZONE_US      25      // Ignores tiny neutral jitter

// ================= ENGINE STATE VARIABLES =================
enum EngineState { ENGINE_OFF, ENGINE_CRANKING, ENGINE_RUNNING };
EngineState currentState = ENGINE_OFF;

volatile uint32_t pwmStart = 0;
volatile uint16_t rcPulseWidth = 1500;
volatile uint32_t lastPulseTime = 0;
volatile uint16_t validPulseCount = 0;

int   neutralPulse = 1500;    // Auto-calibrated when ESC turns on!
float currentThrottle = 0.0f; // 0.0 = Idle, 1.0 = Full Throttle
float targetThrottle  = 0.0f;

float idleIdx  = 0.0f;
float revIdx   = 0.0f;
float knockIdx = 0.0f;
int   popTimer = 0;
uint8_t vibBlockCounter = 0;

inline void setVibMotor(bool turnOn) {
  if (INVERT_VIB_SIGNAL) {
    digitalWrite(VIB_MOTOR_PIN, turnOn ? LOW : HIGH);
  } else {
    digitalWrite(VIB_MOTOR_PIN, turnOn ? HIGH : LOW);
  }
}

void IRAM_ATTR rcThrottleISR() {
  uint32_t now = micros();
  if (digitalRead(RC_THROTTLE_PIN) == HIGH) {
    pwmStart = now;
  } else {
    uint32_t diff = now - pwmStart;
    if (diff >= 850 && diff <= 2150) {
      rcPulseWidth = (uint16_t)diff;
      lastPulseTime = millis();
      if (validPulseCount < 100) validPulseCount++;
    }
  }
}

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 128,
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

void setup() {
  Serial.begin(115200);
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  setVibMotor(false);

  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), rcThrottleISR, CHANGE);
  setupI2S();
  Serial.println("Waiting for Hobbywing 1060 & HotRC CT6A...");
}

void loop() {
  uint32_t now = millis();

  bool signalActive = (now - lastPulseTime < 350) && (validPulseCount >= 15);
  if (now - lastPulseTime >= 350) {
    validPulseCount = 0;
  }

  // 1. Hobbywing 1060 Turned ON -> 100% HIGH VIBRATION DURING CRANKING!
  if (currentState == ENGINE_OFF && signalActive) {
    neutralPulse = rcPulseWidth;
    if (neutralPulse < 1300 || neutralPulse > 1700) neutralPulse = 1500;

    Serial.printf("ESC ON! Auto-Calibrated Neutral = %d us. Cranking...\n", neutralPulse);
    currentState = ENGINE_CRANKING;
    setVibMotor(true); // 100% FULL POWER VIBRATION ON STARTUP CRANK

    float startIdx = 0.0f;
    float startStep = (float)startSampleRate / (float)I2S_SAMPLE_RATE;

    while ((uint32_t)startIdx < startSampleCount) {
      int16_t s = (int16_t)startSamples[(uint32_t)startIdx] * MASTER_VOLUME;
      int16_t stereoFrame[2] = { s, s };
      size_t bytesWritten;
      i2s_write(I2S_NUM_0, stereoFrame, sizeof(stereoFrame), &bytesWritten, portMAX_DELAY);
      startIdx += startStep;
    }

    setVibMotor(false);
    currentThrottle = 0.35f;
    currentState = ENGINE_RUNNING;
  }

  // 2. ESC Turned OFF -> 100% HIGH VIBRATION DURING SHUTOFF!
  else if (currentState == ENGINE_RUNNING && !signalActive) {
    Serial.println("Signal lost / ESC OFF -> Shutting down engine.");
    setVibMotor(true); // 100% FULL POWER VIBRATION ON SHUTDOWN
    float baseRatio = (float)sampleRate / (float)I2S_SAMPLE_RATE;

    for (int i = 0; i < 7000; i++) {
      float fade = 1.0f - ((float)i / 7000.0f);
      idleIdx += (0.55f * fade * baseRatio);
      int16_t s = (int16_t)(readArrayInterp(samples, sampleCount, idleIdx) * fade * MASTER_VOLUME);
      int16_t stereoFrame[2] = { s, s };
      size_t bytesWritten;
      i2s_write(I2S_NUM_0, stereoFrame, sizeof(stereoFrame), &bytesWritten, portMAX_DELAY);
    }
    setVibMotor(false); // Turn OFF completely once engine stops
    currentThrottle = 0.0f;
    currentState = ENGINE_OFF;
  }

  // 3. Engine Running -> EXTREMELY SLOW Vibration in Idle & Driving
  if (currentState == ENGINE_RUNNING) {
    int delta = abs((int)rcPulseWidth - neutralPulse);
    if (delta < DEADZONE_US) {
      targetThrottle = 0.0f;
    } else {
      targetThrottle = (float)(delta - DEADZONE_US) / MAX_THROTTLE_US;
      if (targetThrottle > 1.0f) targetThrottle = 1.0f;
    }

    if (currentThrottle - targetThrottle > 0.30f && popTimer == 0) {
      popTimer = 2800;
    }

    if (targetThrottle > currentThrottle) {
      currentThrottle += (targetThrottle - currentThrottle) * 0.045f;
    } else {
      currentThrottle += (targetThrottle - currentThrottle) * 0.025f;
      if (currentThrottle < 0.01f) currentThrottle = 0.0f;
    }

    // --- EXTREMELY SLOW VIBRATION FOR IDLE & DRIVING ---
    // Only turns ON for 4 blocks (~11.6ms) and rests OFF for 26 blocks (~75.4ms).
    // Keeps MOSFET cool, saves battery, and keeps rotation super slow.
    vibBlockCounter = (vibBlockCounter + 1) % VIB_CYCLE_TOTAL;
    setVibMotor(vibBlockCounter < RUNNING_VIB_PULSE);

    float pitch = 1.0f + (currentThrottle * 1.15f);

    float revMix   = constrain((currentThrottle - 0.15f) / 0.55f, 0.0f, 1.0f);
    float idleMix  = 1.0f - (revMix * 0.75f);
    float knockMix = constrain((targetThrottle - 0.10f) * 1.2f, 0.0f, 0.9f);

    float idleStep  = pitch * ((float)sampleRate / (float)I2S_SAMPLE_RATE);
    float revStep   = pitch * ((float)revSampleRate / (float)I2S_SAMPLE_RATE);
    float knockStep = pitch * ((float)knockSampleRate / (float)I2S_SAMPLE_RATE);

    int16_t buffer[128];
    for (int i = 0; i < 64; i++) {
      idleIdx  += idleStep;  if (idleIdx >= sampleCount)       idleIdx  -= sampleCount;
      revIdx   += revStep;   if (revIdx >= revSampleCount)     revIdx   -= revSampleCount;
      knockIdx += knockStep; if (knockIdx >= knockSampleCount) knockIdx -= knockSampleCount;

      float sIdle  = readArrayInterp(samples, sampleCount, idleIdx) * idleMix;
      float sRev   = readArrayInterp(revSamples, revSampleCount, revIdx) * revMix;
      float sKnock = readArrayInterp(knockSamples, knockSampleCount, knockIdx) * knockMix;

      int32_t mixed = (int32_t)((sIdle + sRev + sKnock) * MASTER_VOLUME);

      if (popTimer > 0) {
        popTimer--;
        if (popTimer % 550 < 90) {
          mixed += (int32_t)(readArrayInterp(knockSamples, knockSampleCount, knockIdx) * MASTER_VOLUME * 1.4f);
        }
      }

      if (mixed > 32767)  mixed = 32767;
      if (mixed < -32768) mixed = -32768;

      buffer[i * 2]     = (int16_t)mixed;
      buffer[i * 2 + 1] = (int16_t)mixed;
    }

    size_t bytesWritten;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
  } 
  else {
    setVibMotor(false);
    int16_t silence[64] = {0};
    size_t bytesWritten;
    i2s_write(I2S_NUM_0, silence, sizeof(silence), &bytesWritten, portMAX_DELAY);
  }
}