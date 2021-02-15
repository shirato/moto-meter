#include <Arduino.h>
#include <Preferences.h>

#include "UserSettings.h"

/* Uncomment below line to change to scale factor adjusting mode. Note that blynk widgets should be changed too. */
// #define ADJUST_MODE

/* Comment this out to disable prints and save space */
/* Insert before blynk libraries */
#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

// Virtual pin setting
#define VP_GAUGE_SPEED V0
#define VP_GAUGE_REV V1
#define VP_VALUE_RATIO V2
#define VP_VALUE_POSITION V3

#define NUM_SHIFTPOSITION 5

#ifdef ADJUST_MODE
#define VP_DISP_SPEED V0
#define VP_DISP_REV V1
#define VP_DISP_SPEED_FREQ V2
#define VP_DISP_REV_FREQ V3
#define VP_STEP_SPEED V4
#define VP_STEP_REV V5
#define VP_BUTTON_SAVE V6
#endif

typedef struct
{
  float frequency;
  unsigned long prevTime;
  int scaleFactor;
} pulse;

Preferences preferences;
BlynkTimer timer;

char auth[] = AUTH_TOKEN;

// set speed-to-rev ratio for each shift position (speed / rev * 1000)
const float SPRatio[NUM_SHIFTPOSITION] = {5.010, 7.607, 9.958, 12.324, 14.189};

// the number of the signal input pin
const int inputPin_speed = 16;
const int inputPin_tacho = 17;
const int inputPin_neutral = 32;
const int outputPin_ind = 33;

// We make these values volatile, as they are used in interrupt context
volatile pulse pulse_speed = {0, 0, 1};
volatile pulse pulse_tacho = {0, 0, 1};
volatile int isNeutral = false;

void IRAM_ATTR calcFrequency_speed()
{
  unsigned long currentTime = micros();
  if ((currentTime - pulse_speed.prevTime) > 500) // if wave length is less than 500 us, regarded as noise.
  {
    pulse_speed.frequency = 1000000.0 / (currentTime - pulse_speed.prevTime);
    pulse_speed.prevTime = currentTime;
  }
}

void IRAM_ATTR calcFrequency_tacho()
{
  unsigned long currentTime = micros();
  if ((currentTime - pulse_tacho.prevTime) > 500) // if wave length is less than 500 us, regarded as noise.
  {
    pulse_tacho.frequency = 1000000.0 / (currentTime - pulse_tacho.prevTime);
    pulse_tacho.prevTime = currentTime;
  }
}

void IRAM_ATTR checkNeutral()
{
  isNeutral = digitalRead(inputPin_neutral);
}

void sendValue()
{
  float speed = pulse_speed.frequency * pulse_speed.scaleFactor; // speed [m/h]
  float rev = pulse_tacho.frequency * pulse_tacho.scaleFactor;   // rev [rpm]
  float ratio = 0;

  if ((int)rev)
  {
    ratio = speed / rev;
  }

  Blynk.virtualWrite(VP_GAUGE_SPEED, speed / 1000);
  Blynk.virtualWrite(VP_GAUGE_REV, rev / 1000);
  Blynk.virtualWrite(VP_VALUE_RATIO, ratio);

  if (isNeutral)
  {
    Blynk.setProperty(VP_VALUE_POSITION, "color", "#23C48E");
    Blynk.virtualWrite(VP_VALUE_POSITION, "N");
  }
  else
  {
    Blynk.setProperty(VP_VALUE_POSITION, "color", "#000000");
    boolean send = false;
    for (int i = 0; i < NUM_SHIFTPOSITION - 1; i++)
    {
      if (ratio < (SPRatio[i] + SPRatio[i + 1]) / 2)
      {
        Blynk.virtualWrite(VP_VALUE_POSITION, i + 1);
        send = true;
        break;
      }
    }
    if (!send)
    {
      Blynk.virtualWrite(VP_VALUE_POSITION, NUM_SHIFTPOSITION);
    }
  }

#ifdef ADJUST_MODE
  Blynk.virtualWrite(VP_DISP_SPEED, pulse_speed.frequency * pulse_speed.scaleFactor / 1000);
  Blynk.virtualWrite(VP_DISP_REV, (int)(pulse_tacho.frequency * pulse_tacho.scaleFactor));
  Blynk.virtualWrite(VP_DISP_SPEED_FREQ, pulse_speed.frequency);
  Blynk.virtualWrite(VP_DISP_REV_FREQ, pulse_tacho.frequency);
#endif
}

BLYNK_CONNECTED()
{
#ifdef ADJUST_MODE
  Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(pulse_speed.scaleFactor) + " / " + String(pulse_tacho.scaleFactor));
#endif
}

#ifdef ADJUST_MODE
BLYNK_WRITE(VP_STEP_SPEED)
{
  if (pulse_speed.scaleFactor + param.asInt() > 0)
  {
    pulse_speed.scaleFactor += param.asInt();
    Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(pulse_speed.scaleFactor) + " / " + String(pulse_tacho.scaleFactor));
  }
}

BLYNK_WRITE(VP_STEP_REV)
{
  if (pulse_tacho.scaleFactor + param.asInt() > 0)
  {
    pulse_tacho.scaleFactor += param.asInt();
    Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(pulse_speed.scaleFactor) + " / " + String(pulse_tacho.scaleFactor));
  }
}

BLYNK_WRITE(VP_BUTTON_SAVE)
{
  if (param.asInt() == 1)
  {
    preferences.begin("moto-meter", false);
    preferences.putUInt("speedFreqRatio", pulse_speed.scaleFactor);
    preferences.putUInt("revFreqRatio", pulse_tacho.scaleFactor);
    preferences.end();
    Serial.println("preferences saved.");
  }
}
#endif

void setup()
{
  // Debug console
  Serial.begin(9600);
  Serial.println("Waiting for connections...");

  Blynk.setDeviceName("Blynk_ESP32");
  Blynk.begin(auth);

  preferences.begin("moto-meter", true);
  pulse_speed.scaleFactor = preferences.getUInt("speedFreqRatio", 135); // default scale factor of speed : 135 [mph/Hz]
  pulse_tacho.scaleFactor = preferences.getUInt("revFreqRatio", 30);    // default scale factor of rev : 30 [rpm/Hz]
  preferences.end();

  // Make input pin HIGH by default
  pinMode(inputPin_speed, INPUT_PULLUP);
  pinMode(inputPin_tacho, INPUT_PULLUP);
  pinMode(inputPin_neutral, INPUT_PULLUP);

  // Attach INT to our handler
  attachInterrupt(digitalPinToInterrupt(inputPin_speed), calcFrequency_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(inputPin_tacho), calcFrequency_tacho, RISING);
  attachInterrupt(digitalPinToInterrupt(inputPin_neutral), checkNeutral, CHANGE);

  timer.setInterval(100L, sendValue);

  BLYNK_LOG("speedFreqRatio=%d\trevFreqRatio=%d\n", pulse_speed.scaleFactor, pulse_tacho.scaleFactor);
}

void loop()
{
  Blynk.run();
  timer.run();
}
