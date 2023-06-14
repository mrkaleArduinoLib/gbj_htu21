/*
  NAME:
  Using the gbjHTU21 library for identifying the sensor HTU21D(F).

  DESCRIPTION:
  The sketch displays all identification and status data stored in the sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
  for used platform accordingly.

  LICENSE:
  This program is free software; you can redistribute it and/or modify it under
  the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_htu21.h"

gbj_htu21 sensor = gbj_htu21();
// gbj_htu21 sensor = gbj_htu21(sensor.CLOCK_100KHZ, D2, D1);
// gbj_htu21 sensor = gbj_htu21(sensor.CLOCK_400KHZ);

void errorHandler(String location)
{
  Serial.println(sensor.getLastErrorTxt(location));
  Serial.println("---");
  return;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("---");

  // Initialize sensor - default holdMasterMode
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    return;
  }
  // Address
  Serial.print("Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  // Serial number
  char text[30];
  snprintf(text,
           30,
           "%04lx-%08lx-%04lx",
           static_cast<long>(sensor.getSNA()),
           static_cast<long>(sensor.getSNB()),
           static_cast<long>(sensor.getSNC()));
  Serial.print("Serial Number (SNA-SNB-SNC): ");
  Serial.println(text);
  // Vdd status
  Serial.print("Vdd Status: ");
  Serial.println((sensor.getVddStatus() ? "OK" : "LOW"));
  // Heater status
  Serial.print("Heater: ");
  Serial.println((sensor.getHeaterEnabled() ? "Enabled" : "Disabled"));
  // Temperature resolution
  Serial.print("Temperature Resolution: ");
  Serial.print(sensor.getResolutionTemp());
  Serial.println(" bits");
  // Humidity resolution
  Serial.print("Humidity Resolution: ");
  Serial.print(sensor.getResolutionRhum());
  Serial.println(" bits");
  Serial.println("---");
}

void loop() {}
