#include <Wire.h>
#include <FDC1004.h>
FDC1004 fdc;

//parameters to set for your measurement
uint8_t numChan = 2; //1 to 4 measurement channels
bool measType = 0; // 0 is single read, 1 is continuous read
uint8_t readRate = 0; // 0 ->100 Hz, 1- > 200 Hz, or 3 -> 400 Hz

uint8_t readDelay[3] = {10, 5, 2}; // number of microseconds to wait per channel
float capAbsolute = -1;
uint8_t capdac[4] = {0, 0, 0, 0};
unsigned long measTime;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(500);
  Wire.begin();
  delay(500);

  fdc.I2CSetup();
  fdc.resetDevice();

  delay(500);

  for (uint8_t currChan = 0; currChan < numChan; currChan++){
    fdc.setCAPDAC(currChan, measType, &(capdac[currChan]), readRate);
  }
}

void loop() {  
  for (uint8_t currChan = 0; currChan < numChan; currChan++){
    while ((millis() - measTime) < readDelay[readRate]*numChan); //delay until it's time for the next reading
    fdc.absoluteCapacitance(currChan, measType, &(capdac[currChan]), &measTime, &capAbsolute, readRate);
    if (capAbsolute > 0){
      Serial.print(currChan);
      Serial.print(",");
      Serial.print(measTime);
      Serial.print(",");
      Serial.println(capAbsolute, 4);
    }
  }
}


