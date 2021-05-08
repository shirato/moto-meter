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
#ifdef ADJUST_MODE
#define VP_DISP_SPEED V0
#define VP_DISP_REV V1
#define VP_DISP_SPEED_FREQ V2
#define VP_DISP_REV_FREQ V3
#define VP_STEP_SPEED V4
#define VP_STEP_REV V5
#define VP_BUTTON_SAVE V6
#else
#define VP_GAUGE_SPEED V0
#define VP_GAUGE_REV V1
#define VP_VALUE_RATIO V2
#define VP_VALUE_GEAR V3
#endif

#define GEAR_NUM 5

typedef struct
{
  float frequency;
  unsigned long prevTime;
  int scaleFactor;
  int durationThreshold;
} Pulse;

Preferences preferences;
BlynkTimer timer;

char auth[] = AUTH_TOKEN;

// set speed-to-rev ratio for each shift position (speed / rev * 1000)
const float defaultSpeedToRevRatioArray[GEAR_NUM] = {5.010, 7.607, 9.958, 12.324, 14.189};
const int defaultSpeedFreqRatio = 135;
const int defaultRevFreqRatio = 30;
const int maxSpeed = 170;
const int maxRev = 13500;

// the number of the signal input pin
const int speedPin = 16;
const int tachoPin = 17;
const int neutralPin = 32;
const int indicatorPin = 33;

// We make these values volatile, as they are used in interrupt context
volatile Pulse speedPulse = {0, 0, 1, 500};
volatile Pulse tachoPulse = {0, 0, 1, 500};
volatile int isNeutral = false;
volatile boolean indicatorState = false;

void IRAM_ATTR calcSpeedFrequency()
{
  if (digitalRead(speedPin))
  {
    unsigned long currentTime = micros();
    if ((currentTime - speedPulse.prevTime) > speedPulse.durationThreshold)
    {
      speedPulse.frequency = 1000000.0 / (currentTime - speedPulse.prevTime);
      speedPulse.prevTime = currentTime;
    }
  }
}

void IRAM_ATTR calcTachoFrequency()
{
  if (digitalRead(tachoPin))
  {
    unsigned long currentTime = micros();
    if ((currentTime - tachoPulse.prevTime) > tachoPulse.durationThreshold)
    {
      tachoPulse.frequency = 1000000.0 / (currentTime - tachoPulse.prevTime);
      tachoPulse.prevTime = currentTime;
    }
  }
}

void IRAM_ATTR checkNeutralState()
{
  isNeutral = !digitalRead(neutralPin);
}

void sendValue()
{
#ifdef ADJUST_MODE
  Blynk.virtualWrite(VP_DISP_SPEED, speedPulse.frequency * speedPulse.scaleFactor / 1000);
  Blynk.virtualWrite(VP_DISP_REV, (int)(tachoPulse.frequency * tachoPulse.scaleFactor));
  Blynk.virtualWrite(VP_DISP_SPEED_FREQ, speedPulse.frequency);
  Blynk.virtualWrite(VP_DISP_REV_FREQ, tachoPulse.frequency);
#else
  float speed = speedPulse.frequency * speedPulse.scaleFactor; // speed [m/h]
  float rev = tachoPulse.frequency * tachoPulse.scaleFactor;   // rev [rpm]
  float speedToRevRatio = 0;

  if ((int)rev)
  {
    speedToRevRatio = speed / rev;
  }

  Blynk.virtualWrite(VP_GAUGE_SPEED, speed / 1000);
  Blynk.virtualWrite(VP_GAUGE_REV, rev / 1000);
  Blynk.virtualWrite(VP_VALUE_RATIO, speedToRevRatio);

  if (isNeutral)
  {
    Blynk.setProperty(VP_VALUE_GEAR, "color", BLYNK_GREEN);
    Blynk.virtualWrite(VP_VALUE_GEAR, "N");
    // if (indicatorState)
    // {
      digitalWrite(indicatorPin, LOW);
    //   indicatorState = !indicatorState;
    // }
  }
  else
  {
    boolean topGear = true;
    for (int i = 0; i < GEAR_NUM - 1; i++)
    {
      if (speedToRevRatio < (defaultSpeedToRevRatioArray[i] + defaultSpeedToRevRatioArray[i + 1]) / 2)
      {
        Blynk.setProperty(VP_VALUE_GEAR, "color", "#888888");
        Blynk.virtualWrite(VP_VALUE_GEAR, i + 1);
        topGear = false;
        // if (indicatorState)
        // {
          digitalWrite(indicatorPin, LOW);
        //   indicatorState = !indicatorState;
        // }
        break;
      }
    }
    if (topGear)
    {
      Blynk.setProperty(VP_VALUE_GEAR, "color", BLYNK_YELLOW);
      Blynk.virtualWrite(VP_VALUE_GEAR, GEAR_NUM);
      // if (!indicatorState)
      // {
        digitalWrite(indicatorPin, HIGH);
      //   indicatorState = !indicatorState;
      // }
    }
  }
#endif
}

BLYNK_CONNECTED()
{
#ifdef ADJUST_MODE
  Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(speedPulse.scaleFactor) + " / " + String(tachoPulse.scaleFactor));
#else
  Blynk.virtualWrite(VP_GAUGE_SPEED, 0);
  // Blynk.virtualWrite(VP_GAUGE_SPEED, speedPulse.scaleFactor);
  Blynk.virtualWrite(VP_GAUGE_REV, 0);
  // Blynk.virtualWrite(VP_GAUGE_REV, tachoPulse.scaleFactor);
  Blynk.virtualWrite(VP_VALUE_RATIO, "Connected");
  Blynk.setProperty(VP_VALUE_GEAR, "color", BLYNK_GREEN);
  Blynk.virtualWrite(VP_VALUE_GEAR, "N");
  // Blynk.setProperty(VP_VALUE_GEAR, "color", BLYNK_YELLOW);
  // Blynk.virtualWrite(VP_VALUE_GEAR, GEAR_NUM);

  delay(3000);
#endif
}

#ifdef ADJUST_MODE
BLYNK_WRITE(VP_STEP_SPEED)
{
  if (speedPulse.scaleFactor + param.asInt() > 0)
  {
    speedPulse.scaleFactor += param.asInt();
    Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(speedPulse.scaleFactor) + " / " + String(tachoPulse.scaleFactor));
  }
}

BLYNK_WRITE(VP_STEP_REV)
{
  if (tachoPulse.scaleFactor + param.asInt() > 0)
  {
    tachoPulse.scaleFactor += param.asInt();
    Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(speedPulse.scaleFactor) + " / " + String(tachoPulse.scaleFactor));
  }
}

BLYNK_WRITE(VP_BUTTON_SAVE)
{
  if (param.asInt() == 1)
  {
    preferences.begin("moto-meter", false);
    preferences.putUInt("speedFreqRatio", speedPulse.scaleFactor);
    preferences.putUInt("revFreqRatio", tachoPulse.scaleFactor);
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
  speedPulse.scaleFactor = preferences.getUInt("speedFreqRatio", defaultSpeedFreqRatio);
  tachoPulse.scaleFactor = preferences.getUInt("revFreqRatio", defaultRevFreqRatio);
  preferences.end();

  speedPulse.durationThreshold = (speedPulse.scaleFactor * 10 / maxSpeed) * 100;
  tachoPulse.durationThreshold = (tachoPulse.scaleFactor * 10000 / maxRev) * 100;

  // Make input pin HIGH by default
  pinMode(speedPin, INPUT_PULLUP);
  pinMode(tachoPin, INPUT_PULLUP);
  pinMode(neutralPin, INPUT_PULLUP);

  pinMode(indicatorPin, OUTPUT);

  // Attach INT to our handler
  attachInterrupt(digitalPinToInterrupt(speedPin), calcSpeedFrequency, RISING);
  attachInterrupt(digitalPinToInterrupt(tachoPin), calcTachoFrequency, RISING);
  attachInterrupt(digitalPinToInterrupt(neutralPin), checkNeutralState, CHANGE);

  timer.setInterval(100L, sendValue);

  BLYNK_LOG("speedFreqRatio=%d\trevFreqRatio=%d\n", speedPulse.scaleFactor, tachoPulse.scaleFactor);
}

void loop()
{
  Blynk.run();
  timer.run();
}
