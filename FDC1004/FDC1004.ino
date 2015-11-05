/**************************************************************
 * Example Program for FDC1004 Library
 * This program will repeatedly read the capacitance on CAP1 of the FDC1004
 * and print the value to serial.
 * Capdac will automatically increment or decrement if the capacitance value goes out of range
 * Note that the output is a signed integer, so values greater than 7FFF FFFF are actually negative!
 **************************************************************
 * Written by Benjamin Shaya for Rest Devices
 **************************************************************/
 
#include <Wire.h>
#include <FDC1004.h>

int capdac = 0;

FDC1004 fdc;

void setup() {
  //Wire.begin();
  Serial.begin(115200);
}

void loop() {
  uint8_t measurement = 0;
  uint8_t channel = 0;
  char result[100];


  fdc.configureMeasurementSingle(measurement, channel, capdac);
  fdc.triggerSingleMeasurement(measurement, FDC1004_100HZ);
  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! fdc.readMeasurement(measurement, value)) {
    int16_t msb = (int16_t) value[0];
    sprintf(result, "%04X %04X @ %02X\n", msb, value[1], capdac);
    Serial.print(result);
    //adjust capdac
    if (msb > 0x7000) {
      if (capdac < FDC1004_CAPDAC_MAX) capdac++;
    } else if (msb < 0x8FF) {
      if (capdac > 0) capdac--;
    }
  }
  delay(20);
}
