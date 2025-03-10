/***********************************************************************
 FDC1004 Library
 This library provides functions for using TI's FDC1004 Capacitance to Digital Sensor
 Written by Benjamin Shaya and Kris Dorsey
************************************************************************/

#include <FDC1004.h>

uint8_t MEAS_CONFIG[4] = {0x08, 0x09, 0x0A, 0x0B};
uint8_t MEAS_MSB[4] = {0x00, 0x02, 0x04, 0x06};
uint8_t MEAS_LSB[4] = {0x01, 0x03, 0x05, 0x07};
uint8_t pins[4] = {(uint8_t)FDC1004_Chan0, (uint8_t)FDC1004_Chan1, (uint8_t)FDC1004_Chan2, (uint8_t)FDC1004_Chan3};
uint8_t meas[4] = {0, 1, 2, 3};

const float capMax = 14.9;
const float capdacConversion = 3.125;

uint8_t freq[3] = {1, 2, 3};

FDC1004::FDC1004(uint16_t rate){
  this->_addr = 0b1010000; 
  this->_rate = rate;
}

void FDC1004::write16(uint8_t reg, uint16_t data) {
  Wire.beginTransmission(_addr);
  Wire.write(reg); //send address
  Wire.write( (uint8_t) (data >> 8));
  Wire.write( (uint8_t) data);
  Wire.endTransmission();
}

uint16_t FDC1004::read16(uint8_t reg) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.endTransmission();
  uint16_t value;
  Wire.requestFrom(_addr, (uint8_t)2);
  if (Wire.available() == 2){
    value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  return value;
  }
  else{
    Serial.print("Wrong number of bytes available: ");
    Serial.println(Wire.available());
  }
  return 0;
}

//configure a measurement (call only when changing the setup of a measurement)
uint8_t FDC1004::configureMeasurement(uint8_t measurement, uint8_t channel, uint8_t diffChannel, uint8_t capdac) {
    //Verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_CHANNEL(channel) || capdac > FDC1004_CAPDAC_MAX) {
        Serial.println("Measurement, channel, or capdac out of bounds");
        return 1;
    }

    //build 16 bit measurement configuration
    uint16_t configuration_data;
    configuration_data = ((uint16_t)channel) << 13; //CHA diff

    if (channel == diffChannel && capdac != 0){
        //set CAPDAC, disable differential excitation on EXB
        configuration_data |=  ((uint16_t)0x04) << 10; //CHB disable and CAPDAC enable
        configuration_data |= ((uint16_t) capdac) << 5; //CAPDAC value
    }
    else if (channel == diffChannel && capdac == 0){
        //Disable CAPDAC and differential excitation on EXB
        configuration_data |=  ((uint16_t)0x05) << 10; //CHB disable and CAPDAC disable
    }
    else {
        //disable CAPDAC, set differential excitation
        configuration_data |=  ((uint16_t)diffChannel) << 10; //CHB enable and CAPDAC disable
    }
    write16(MEAS_CONFIG[measurement], configuration_data);
    return 0;
}

uint8_t FDC1004::triggerMeasurement(uint8_t measurement, uint8_t rate, uint8_t measType) {
  //verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_RATE(rate)) {
       Serial.println("bad trigger request");
       return 1;
    }

    uint16_t trigger_data;
    trigger_data = ((uint16_t)rate) << 10; // sample rate
    trigger_data |= measType << 8; 
    trigger_data |= (1 << (7-measurement)); // 0 > bit 7, 1 > bit 6, etc
    write16(FDC_REGISTER, trigger_data);
    return 0;
}

uint8_t FDC1004::readMeasurement(uint8_t measurement, uint16_t * value) {
    if (!FDC1004_IS_MEAS(measurement)) {
        Serial.println("bad read request");
        return 1;
    }
    
    //check if measurement is complete
    uint16_t fdcRegister = read16(FDC_REGISTER);
    uint8_t delayCount = 0;
    uint8_t maxDelayCount = 10;
    while ((! (fdcRegister & ( 1 << (3-measurement)))) && delayCount < maxDelayCount) {
      //confirm with FDC that measurement is complete
      delayMicroseconds(500); 
      delayCount++;
      fdcRegister = read16(FDC_REGISTER);
    }

    if (delayCount < maxDelayCount){
      //read the value
      uint16_t msb = read16(MEAS_MSB[measurement]);
      uint16_t lsb = read16(MEAS_LSB[measurement]);  
      value[0] = msb;
      value[1] = lsb;
      return 0;
    }
    else{
      //read failed or timed out
      return 1;    
    }  
}

//reset FDC1004
uint16_t FDC1004::resetDevice() {
  uint16_t resetData = 1 << 15; //reset is 0x1000
  uint8_t delayCount = 0;
  uint8_t maxDelayCount = 10;

  write16(FDC_REGISTER, resetData);
  delay(100); 
  uint16_t fdcRegister = read16(FDC_REGISTER);
  return fdcRegister; //will return zero if properly reset
}

void FDC1004::I2CSetup(){
  //Make sure device is present on I2C bus
    Wire.beginTransmission(_addr);
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
      Serial.print("FDC1004 device NOT found on I2C bus and timeout reached. Check board connections.");
    }
}

void FDC1004::absoluteCapacitance(uint8_t currChan, bool measType, uint8_t * capdacPtr, unsigned long * measTime, float * capacitanceAbsolute, uint8_t readRate) { 
  uint8_t measSlot = meas[currChan];
  uint8_t chanA = pins[currChan];
  uint8_t chanB = pins[currChan];
  float capacitanceRelative;
  *capacitanceAbsolute = -1;
  if (!measType){
    capacitanceRelative =  configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr, readRate);
    }
  else{
    //do not need to re-trigger for continuous read
    capacitanceRelative = relativeCapacitance(measSlot);
    }
  *measTime = millis();
  if ((capacitanceRelative <= capMax) && (capacitanceRelative >= -capMax)){
    *capacitanceAbsolute = capacitanceRelative + (3.125 * (float)*capdacPtr); //converts capdac to pF
  }
  else{
    Serial.println("Capacitance out of bounds, re-running setCAPDAC");
    setCAPDAC(currChan, measType, capdacPtr, readRate);
    }
}

float FDC1004::relativeCapacitance(uint8_t measSlot) {  
  uint16_t value[2];
  float capacitanceRelative = capMax + 1;
  if (!FDC1004::readMeasurement(measSlot, value)) {
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

uint8_t FDC1004::setCAPDAC(uint8_t currChan, uint8_t measType, uint8_t * capdacPtr, uint8_t readRate){
  uint8_t measSlot = meas[currChan];
  uint8_t chanA = pins[currChan];
  uint8_t chanB = pins[currChan];
  if (chanA == chanB){
    //only execute if in single-ended mode. CAPDAC is not set in differential mode

    uint8_t capdacTimeout = 0;
    uint8_t capdacTimeoutLimit = 30;

    float capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr, readRate);
    uint8_t capdacFlag = 1;

    while ((capdacTimeout < capdacTimeoutLimit) && (capdacFlag)){
      capdacFlag = adjustCAPDAC(capacitanceRelative, capdacPtr);
      capacitanceRelative = configTrigRead(measSlot, chanA, chanB, measType, *capdacPtr, readRate);
      capdacTimeout++;
      delayMicroseconds(500);
    }

    if (capdacFlag){
      //resetting capdac timed out
      *capdacPtr = 0;
    }
  }
  return *capdacPtr;
}

uint8_t FDC1004::adjustCAPDAC(float capacitanceRelative, uint8_t * capdacPtr) {  
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

float FDC1004::configTrigRead(uint8_t measSlot, uint8_t chanA, uint8_t chanB, uint8_t measType, uint8_t capdac, uint8_t readRate){
  FDC1004::configureMeasurement(measSlot, chanA, chanB, capdac);
  FDC1004::triggerMeasurement(measSlot, freq[readRate], measType);
  return FDC1004::relativeCapacitance(measSlot);
}
