/**************************************************************
 * Example Program for FDC1004 Library
 * This example will make a single reading on channels 0 and 1 and auto-set CAPDAC for each channel
 * You can uncomment the relevant lines in setup() and loop() to also read channels 2 and 3
 **************************************************************
 * Setup
 * Connect 3.3V and Ground to the FDC1004 (don't use 5V, you'll fry your chip!)
 * Connect SDA and SCL to your Arduino
 * Power on and run this code
 * Open a Serial monitor at 115200 baud
 **************************************************************
 * Read/write convenience functions written by Benjamin Shaya for Rest Devices
 * bshaya@alum.mit.edu
 * https://github.com/beshaya/FDC1004
 **************************************************************
 * Functions in this .ino file written by Kris Dorsey, Northeastern University
 * k.dorsey@northeastern.edu
 * https://github.com/kdorseyNU/FDC1004
 **************************************************************/
 
#include <Wire.h>
#include <FDC1004.h>

FDC1004 fdc;

////////////Fixed measurement parameters, do not change/////////////////////
const float capMax = 14.9;
const float capdacConversion = 3.125;

const uint8_t measSingl = 0; // 0 is single read, 1 is continuous read
const uint8_t measCont = 1; // 0 is single read, 1 is continuous read

const uint8_t measA = 0; 
const uint8_t measB = 1; 
const uint8_t measC = 2; 
const uint8_t measD = 3; 

const uint8_t pin1 = (uint8_t)FDC1004_Chan0;
const uint8_t pin2 = (uint8_t)FDC1004_Chan1; 
const uint8_t pin3 = (uint8_t)FDC1004_Chan2; 
const uint8_t pin4 = (uint8_t)FDC1004_Chan3; 

uint8_t capdacA; 
uint8_t capdacB; 
uint8_t capdacC; 
uint8_t capdacD; 

unsigned long measTime;

///////////Parameters you can change//////////////////////////////////////////
int readRate = 10; //in Hz

//////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(500);
  Wire.begin();
  delay(500);

  //Make sure device is present on I2C bus
  Wire.beginTransmission(fdc._addr);
  int error = Wire.endTransmission();
  int I2C_timeout = 10; 
  while (I2C_timeout > 0 && error > 0){
    Serial.print("I2C device NOT found. ");
    Serial.println("Waiting for 5 seconds before attempting setup...");
    delay(5000);
    I2C_timeout -= 1; 
    error = Wire.endTransmission();
  }
  if (error == 0){
    Serial.println("FDC1004 device found");
  }
  else{
    Serial.print("I2C device NOT found  and timeout reached. Data is invalid.");
  }

  fdc.resetDevice();
  delay(500);
  
  setCAPDAC(measA, pin1, pin1, measSingl, &capdacA);
  setCAPDAC(measB, pin2, pin2, measSingl, &capdacB);
  setCAPDAC(measC, pin3, pin3, measSingl, &capdacC); 
  setCAPDAC(measD, pin4, pin4, measSingl, &capdacD);
}



void loop() {  
  Serial.println("Starting power on check.");

  float capA = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  if (capA > 0){
    Serial.println("Channel A OK");
    delay(1000);
  }

  float capB = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
    if (capB > 0){
    Serial.println("Channel B OK");
    delay(1000);
  }

  float capC = absoluteCapacitance(measC, pin3, pin3, measSingl, &capdacC); 
    if (capB > 0){
    Serial.println("Channel C OK");
    delay(1000);
  }

  float capD = absoluteCapacitance(measD, pin4, pin4, measSingl, &capdacD); 
    if (capD > 0){
    Serial.println("Channel D OK");
    delay(1000);
  }

  if (analogRead(A0) > 0){
    Serial.print("Temperature sensor reading: ");
    float temp = (analogRead(A0) / 4095.0) * (1100.0/19.5); //Sensitivity of 19.5 mV/C and max voltage of 1.1 V
    Serial.println(temp);
  }
  else{
    Serial.println("Temperature sensor not OK");
  }
  delay(3600000);
}

//////////////////////////////////////////////////////////////////////////////


float absoluteCapacitance(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t * capdacPtr) {
  float capacitanceRelative;
  float capacitanceAbsolute = -1;
  if (measType == 0){
    capacitanceRelative =  configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
    }
  else{
    //do not need to re-trigger for continuous read
    capacitanceRelative = relativeCapacitance(measSlot);
    }
  measTime = millis();
  if ((capacitanceRelative <= capMax) && (capacitanceRelative >= -capMax)){
    capacitanceAbsolute = capacitanceRelative + (3.125 * (float)*capdacPtr); //converts capdac to pF
  }
  else{
    Serial.println("Capacitance out of bounds, re-running setCAPDAC");
    setCAPDAC(measSlot, chanA, chanB, measType, capdacPtr);
    }
  return capacitanceAbsolute;
}

float relativeCapacitance(uint8_t measSlot) {  
  delay(1000/readRate);
  uint16_t value[2];
  float capacitanceRelative = capMax + 1;
  if (!fdc.readMeasurement(measSlot, value)) {
    // The absolute capacitance is a function of the capdac and the read MSB (value[0])
    // The measurement resolution is 0.5 fF and the max range is 30 pF 
    // A 16 bit two's complement binary number ranges from -32768 to 32767 (i.e., resolution of ~0.46 fF with +/- 15 pF range)
    // Therefore, we don't need to calculate the capacitance using the LSB since the most significant bit of LSB is below the measurement resolution
    // We do still need to read LSB since the FDC1004 is expecting it to be read before the next measurement. The convenience functions in the .cpp file take care of this read.
    int16_t capBits = value[0];
    float capRatio = (15.0) / (32767.0); //This converts bits (2^15 - 1) into capacitance (+/- 15 pF range)
    capacitanceRelative = (float)(capBits) * capRatio; // value in picofarads (pF)
  }
  return capacitanceRelative;
}

uint8_t setCAPDAC(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t * capdacPtr){
  *capdacPtr = 0; 
  if (chanA == chanB){
    //only execute if in single-ended mode. CAPDAC is not set in differential mode

    uint8_t capdacTimeout = 0;
    uint8_t capdacTimeoutLimit = 30;

    float capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
    uint8_t capdacFlag = 1;

    while ((capdacTimeout < capdacTimeoutLimit) && (capdacFlag)){
      capdacFlag = adjustCAPDAC(capacitanceRelative, capdacPtr);
      capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr);
      capdacTimeout++;
      delay(5);
    }

    if (capdacFlag){
      //resetting capdac timed out
      *capdacPtr = 0;
    }
  }
  return *capdacPtr;
}

uint8_t adjustCAPDAC(float capacitanceRelative, uint8_t * capdacPtr) {  
  if ((capacitanceRelative < capMax) && (capacitanceRelative > -1*capMax)){
    // if it's in range, adjust capdac so capacitance is as close to 0 as possible
    *capdacPtr += capacitanceRelative/capdacConversion;
    *capdacPtr = max(*capdacPtr, ((uint8_t)0));
    *capdacPtr = min(*capdacPtr, ((uint8_t)FDC1004_CAPDAC_MAX));
    return 0;
  }
  else if (capacitanceRelative <= -1*capMax) {
    *capdacPtr -= 5; //decrease by 15.625 pF, which is just outside the bounds of +/- 15 pF measurement range
    *capdacPtr = max(*capdacPtr, ((uint8_t)0));
    return 1;
  }
  else {
    *capdacPtr += 5; //increase by 15.625 pF, which is just outside the bounds of +/- 15 pF measurement range
    *capdacPtr = min(*capdacPtr, ((uint8_t)FDC1004_CAPDAC_MAX));
    return 1;
  } 
}

float configTrigRead(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t capdac){
  fdc.configureMeasurement(measSlot, chanA, chanB, capdac);
  fdc.triggerMeasurement(measSlot, FDC1004_100HZ, measType);
  return relativeCapacitance(measSlot);
}