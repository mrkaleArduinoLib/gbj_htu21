/*
  NAME:
  List of all possible resolutions of the sensor HTU21D(F) with gbjHTU21 library.

  DESCRIPTION:
  The sketch alters resolution of HTU21D(F) sensor and display it in bits and
  respective measurement units.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - The sketch sets resolution by library constants for demonstration of their
    usage.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#define SKETCH "GBJ_HTU21_RESOLUTION_CONST 1.0.0"

#include "gbj_htu21.h"

gbj_htu21 Sensor = gbj_htu21();
// gbj_htu21 Sensor = gbj_htu21(gbj_htu21::CLOCK_100KHZ, true, D2, D1);
// gbj_htu21 Sensor = gbj_htu21(gbj_htu21::CLOCK_400KHZ);

byte resolutionList[4] = {gbj_htu21::RESOLUTION_T14_RH12, \
                          gbj_htu21::RESOLUTION_T13_RH10, \
                          gbj_htu21::RESOLUTION_T12_RH8, \
                          gbj_htu21::RESOLUTION_T11_RH11};


void errorHandler(String location)
{
  if (Sensor.isSuccess()) return;
  Serial.print(location);
  Serial.print(" - Error: ");
  Serial.print(Sensor.getLastResult());
  Serial.print(" - ");
  switch (Sensor.getLastResult())
  {
    // General
    case gbj_htu21::ERROR_ADDRESS:
      Serial.println("ERROR_ADDRESS");
      break;

    case gbj_htu21::ERROR_PINS:
      Serial.println("ERROR_PINS");
      break;

    // Arduino, Esspressif specific
#if defined(__AVR__) || defined(ESP8266) || defined(ESP32)
    case gbj_htu21::ERROR_BUFFER:
      Serial.println("ERROR_BUFFER");
      break;

    case gbj_htu21::ERROR_NACK_DATA:
      Serial.println("ERROR_PINS");
      break;

    case gbj_htu21::ERROR_NACK_OTHER:
      Serial.println("ERROR_NACK_OTHER");
      break;

    // Particle specific
#elif defined(PARTICLE)
    case gbj_htu21::ERROR_BUSY:
      Serial.println("ERROR_BUSY");
      break;

    case gbj_htu21::ERROR_END:
      Serial.println("ERROR_END");
      break;

    case gbj_htu21::ERROR_TRANSFER:
      Serial.println("ERROR_TRANSFER");
      break;

    case gbj_htu21::ERROR_TIMEOUT:
      Serial.println("ERROR_TIMEOUT");
      break;
#endif

    case gbj_htu21::ERROR_SERIAL_A:
      Serial.println("ERROR_SERIAL_A");
      break;

    case gbj_htu21::ERROR_SERIAL_B:
      Serial.println("ERROR_SERIAL_B");
      break;

    case gbj_htu21::ERROR_REG_RHT_READ:
      Serial.println("ERROR_REG_RHT_READ");
      break;

    case gbj_htu21::ERROR_MEASURE_RHUM:
      Serial.println("ERROR_MEASURE_RHUM");
      break;

    case gbj_htu21::ERROR_MEASURE_TEMP:
      Serial.println("ERROR_MEASURE_TEMP");
      break;

    default:
      Serial.println("Uknown error");
      break;
  }
}


void setup()
{
  Serial.begin(9600);
  Serial.println(SKETCH);
  Serial.println("Libraries:");
  Serial.println(gbj_twowire::VERSION);
  Serial.println(gbj_htu21::VERSION);
  Serial.println("---");

  // Initialize Sensor
  if (Sensor.begin()) // Use default holdMasterMode
  {
    errorHandler("Begin");
    return;
  }
  // List possible resolution
  Serial.println("Temperature and Humidity resolution in bits:");
  for (byte i = 0; i < (sizeof(resolutionList) / sizeof(resolutionList[0])); i++)
  {
    Sensor.setResolution(resolutionList[i]);
    if (Sensor.isSuccess())
    {
      Serial.print("T: ");
      Serial.print(Sensor.getResolutionTemp());
      Serial.print(" - RH: ");
      Serial.println(Sensor.getResolutionRhum());
    }
    errorHandler("Resolution");
  }
}


void loop() {}
