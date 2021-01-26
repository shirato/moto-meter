#include <Arduino.h>
#include "UserSettings.h"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

char auth[] = AUTH_TOKEN;

BlynkTimer timer;

const int inputPin = 16; // the number of the signal input pin

// We make these values volatile, as they are used in interrupt context
volatile float frequency = 0;
volatile unsigned long prevTime = 0;

// Most boards won't send data to WiFi out of interrupt handler.
// We just store the value and process it in the main loop.
void IRAM_ATTR calcDuration()
{
  unsigned long currentTime = micros();
  frequency = 1000000.0 /(currentTime - prevTime);
  prevTime = currentTime;
}

void sendValue()
{
  Serial.printf("%f [Hz]\n", frequency);
  Blynk.virtualWrite(V2, frequency);
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  Serial.println("Waiting for connections...");

  Blynk.setDeviceName("Blynk_ESP32");

  Blynk.begin(auth);

  // Make pin inputPin HIGH by default
  pinMode(inputPin, INPUT_PULLUP);
  // Attach INT to our handler
  attachInterrupt(digitalPinToInterrupt(inputPin), calcDuration, RISING);

  timer.setInterval(100L, sendValue);
}

void loop()
{
  Blynk.run();
  timer.run();
}
