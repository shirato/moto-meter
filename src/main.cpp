#include <Arduino.h>
#include "UserSettings.h"

#include <Preferences.h>

/* Uncomment below line to change to scale factor adjusting mode. Note that blynk widgets should be changed too. */
#define ADJUST_MODE

/* Comment this out to disable prints and save space */
/* Insert before blynk libraries */
#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>


#ifdef ADJUST_MODE
#define VP_DISP_SPEED V0
#define VP_DISP_REV V1
#define VP_DISP_SPEED_FREQ V2
#define VP_DISP_REV_FREQ V3
#define VP_STEP_SPEED V4
#define VP_STEP_REV V5
#define VP_BUTTON_SAVE V6
#endif

char auth[] = AUTH_TOKEN;

typedef struct
{
  float frequency;
  unsigned long prevTime;
  int scaleFactor;
} pulse;

Preferences preferences;

BlynkTimer timer;

// the number of the signal input pin
const int inputPin_speed = 16;
const int inputPin_tacho = 17;

// We make these values volatile, as they are used in interrupt context
volatile pulse pulse_speed = {0, 0, 1};
volatile pulse pulse_tacho = {0, 0, 1};

// Most boards won't send data to WiFi out of interrupt handler.
// We just store the value and process it in the main loop.
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

void sendValue()
{
  // Serial.printf("speed = %f [Hz]\t tacho = %f [Hz]\n", pulse_speed.frequency, pulse_tacho.frequency);

  // Blynk.virtualWrite(VP_DISP_SPEED, (int)(pulse_speed.frequency * pulse_speed.scaleFactor / 1000));
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
#endif

#ifdef ADJUST_MODE
BLYNK_WRITE(VP_STEP_REV)
{
  if (pulse_tacho.scaleFactor + param.asInt() > 0)
  {
    pulse_tacho.scaleFactor += param.asInt();
    Blynk.setProperty(VP_BUTTON_SAVE, "offLabel", String(pulse_speed.scaleFactor) + " / " + String(pulse_tacho.scaleFactor));
  }
}
#endif

#ifdef ADJUST_MODE
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

  // Make input pin HIGH by default
  pinMode(inputPin_speed, INPUT_PULLUP);
  pinMode(inputPin_tacho, INPUT_PULLUP);
  // Attach INT to our handler
  attachInterrupt(digitalPinToInterrupt(inputPin_speed), calcFrequency_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(inputPin_tacho), calcFrequency_tacho, RISING);

  timer.setInterval(100L, sendValue);

  preferences.begin("moto-meter", true);
  pulse_speed.scaleFactor = preferences.getUInt("speedFreqRatio", 135); // default scale factor of speed : 135 [mph/Hz]
  pulse_tacho.scaleFactor = preferences.getUInt("revFreqRatio", 30);    // default scale factor of rev : 30 [rpm/Hz]
  preferences.end();
}

void loop()
{
  Blynk.run();
  timer.run();
}
