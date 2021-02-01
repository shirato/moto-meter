#include <Arduino.h>
#include "UserSettings.h"

#include <Preferences.h>

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT

char auth[] = AUTH_TOKEN;

typedef struct
{
  float frequency;
  unsigned long prevTime;
} pulse;

Preferences preferences;

// WidgetTerminal terminal(V1);
BlynkTimer timer;

// the number of the signal input pin
const int inputPin_speed = 16;
const int inputPin_tacho = 17;

// We make these values volatile, as they are used in interrupt context
volatile pulse pulse_speed = {0, 0};
volatile pulse pulse_tacho = {0, 0};

int speedFreqRatio; // [m/h/Hz]
int revFreqRatio;   // [rpm/Hz]

// Most boards won't send data to WiFi out of interrupt handler.
// We just store the value and process it in the main loop.
void IRAM_ATTR calcFrequency_speed()
{
  unsigned long currentTime = micros();
  pulse_speed.frequency = 1000000.0 / (currentTime - pulse_speed.prevTime);
  pulse_speed.prevTime = currentTime;
}

void IRAM_ATTR calcFrequency_tacho()
{
  unsigned long currentTime = micros();
  pulse_tacho.frequency = 1000000.0 / (currentTime - pulse_tacho.prevTime);
  pulse_tacho.prevTime = currentTime;
}

void sendValue()
{
  // Serial.printf("speed = %f [Hz]\t tacho = %f [Hz]\n", pulse_speed.frequency, pulse_tacho.frequency);
  Blynk.virtualWrite(V2, pulse_speed.frequency);
  Blynk.virtualWrite(V3, pulse_tacho.frequency);
  // terminal.printf("speed = %f [Hz]\t tacho = %f [Hz]\n", pulse_speed.frequency, pulse_tacho.frequency);
}

BLYNK_WRITE(V4)
{
  if (speedFreqRatio + param.asInt() > 0)
  {
    speedFreqRatio += param.asInt();
    Serial.printf("speedFreqRatio = %d [m/h/Hz]\n", speedFreqRatio);
  }
}

BLYNK_WRITE(V5)
{
  if ((revFreqRatio + param.asInt()) > 0)
  {
    revFreqRatio += param.asInt();
    Serial.printf("revFreqRatio = %d [rpm/Hz]\n", revFreqRatio);
  }
}

BLYNK_WRITE(V6)
{
  if(param.asInt() == 1)
  {
    preferences.begin("moto-meter", false);
    preferences.putUInt("speedFreqRatio", speedFreqRatio);
    preferences.putUInt("revFreqRatio", revFreqRatio);
    preferences.end();
    Serial.println("preferences saved.");
  }
}

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
  speedFreqRatio = preferences.getUInt("speedFreqRatio", 1);
  revFreqRatio = preferences.getUInt("revFreqRatio", 1);
  preferences.end();

  Serial.printf("speedFreqRatio = %d [m/h/Hz]\n", speedFreqRatio);
  Serial.printf("revFreqRatio = %d [rpm/Hz]\n", revFreqRatio);
}

void loop()
{
  Blynk.run();
  timer.run();
}
