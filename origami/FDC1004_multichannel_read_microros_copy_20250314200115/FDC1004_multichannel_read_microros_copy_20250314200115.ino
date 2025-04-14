#include <Wire.h>
#include <FDC1004.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

rcl_publisher_t string_publisher;
std_msgs__msg__String msgStringA;
std_msgs__msg__String msgStringB;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


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
int readRate = 50; //does not currently work above 100 Hz
//this is 10 Hz: 10 reads per second

//////////////////////////////////////////////////////////////////////////////

unsigned long lastCheckTime = 0;  // Time of last check
int I2C_timeout = 10;  // Number of retries
//////////////////////////////////////////////////////////////////////////////


void setup() {
  set_microros_transports(); //Initialize micro-ros transport
  
  allocator = rcl_get_default_allocator();
  
  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  
  //create node
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  //create publisher 
  rclc_publisher_init_default(
    &string_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "micro_ros_arduino_node_publisher");


  //create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  
  // Initialize Serial and I2C
  Serial.begin(115200);
  Serial.println("Starting...");
  Wire.begin();
  checkI2CDevice();
  fdc.resetDevice();
  setCAPDAC(measA, pin1, pin1, measSingl, &capdacA);
  setCAPDAC(measB, pin2, pin2, measSingl, &capdacB);
  
}



void loop() {  
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));  // Ensure Micro-ROS tasks are executed
 
 // Update the measTime with the current time in milliseconds
  measTime = millis();

  // Read capacitance value
  float capA = absoluteCapacitance(measA, pin1, pin1, measSingl, &capdacA);
  float capB = absoluteCapacitance(measB, pin2, pin2, measSingl, &capdacB);
  
  // Convert values to string
  char strBufferA[128];  // Make sure the buffer is large enough
  snprintf(strBufferA, sizeof(strBufferA), "Cap A, Time: %lu, Capacitance: %.4f", measTime, capA);

  // Assign the string to the message
  msgStringA.data.data = strBufferA;
  msgStringA.data.size = strlen(strBufferA);

 // Convert values to string
  char strBufferB[128];  // Make sure the buffer is large enough
  snprintf(strBufferB, sizeof(strBufferB), "Cap B, Time: %lu, Capacitance: %.4f", measTime, capB);

  // Assign the string to the message
  msgStringB.data.data = strBufferB;
  msgStringB.data.size = strlen(strBufferB);

  // Log message to Serial Monitor
  Serial.print("PublishingA: ");
  Serial.println(strBufferA);
  Serial.print("PublishingB: ");
  Serial.println(strBufferB);

  // Publish the message
  rcl_publish(&string_publisher, &msgStringA, NULL); // Publish capA message
  rcl_publish(&string_publisher, &msgStringB, NULL); // Publish capB message

}

//////////////////////////////////////////////////////////////////////////////


//I2C device check
void checkI2CDevice(){
  Wire.beginTransmission(fdc._addr);
  int error = Wire.endTransmission();

  while (I2C_timeout > 0 && error > 0) {
    if (millis() - lastCheckTime > 5000) {  // Wait for 5 seconds before retrying
      Serial.print("I2C device NOT found. ");
      Serial.println("Waiting for 5 seconds before attempting setup...");
      lastCheckTime = millis();  // Reset the timer
      I2C_timeout -= 1;
      error = Wire.endTransmission();  // Retry I2C device check
    }
  }
  
  if (error == 0) {
    Serial.println("FDC1004 device found");
  } 
  else {
    Serial.print("I2C device NOT found and timeout reached. Data is invalid.");
  }
}


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
  //measTime = millis();
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