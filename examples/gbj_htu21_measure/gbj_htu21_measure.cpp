/*
  NAME:
  Basic measurement with gbjHTU21 library.

  DESCRIPTION:
  The sketch measures humidity and temperature with HTU21D(F) sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
  for used platform accordingly.

  LICENSE:
  This program is free software; you can redistribute it and/or modify it under
  the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_htu21.h"

// Time in miliseconds between measurements
const unsigned int PERIOD_MEASURE = 3000;

gbj_htu21 sensor = gbj_htu21();
// gbj_htu21 sensor = gbj_htu21(sensor.CLOCK_100KHZ, D2, D1);
// gbj_htu21 sensor = gbj_htu21(sensor.CLOCK_400KHZ);

float tempValue, rhumValue;

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
  if (sensor.isError(sensor.setResolutionTemp12()))
  {
    errorHandler("Resolution");
    return;
  }
  Serial.println("Humidity (%) / Temperature (°C)");
}

void loop()
{
  // Humidity
  rhumValue = sensor.measureHumidity();
  if (sensor.isError())
  {
    errorHandler("Humidity");
  }
  // Temperature
  tempValue = sensor.measureTemperature();
  if (sensor.isError())
  {
    errorHandler("Temperature");
  }
  Serial.print(rhumValue);
  Serial.print(" / ");
  Serial.print(tempValue);
  Serial.println(" :: separated");
  // Temperature and humidity compensated
  rhumValue = sensor.measureHumidity(tempValue);
  if (sensor.isError())
  {
    errorHandler("Measurement");
  }
  Serial.print(rhumValue);
  Serial.print(" / ");
  Serial.print(tempValue);
  Serial.println(" :: compensated");
  Serial.println();
  delay(PERIOD_MEASURE);
}
