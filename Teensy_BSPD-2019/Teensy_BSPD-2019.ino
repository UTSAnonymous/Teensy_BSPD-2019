/*
 * UTSME AUTONOMOUS 2019
 * by Hein Wai Leong
 * 
 * CanBus Library Fork from pawelsky / FlexCAN_Library
 * https://github.com/pawelsky/FlexCAN_Library
 * Arduino library for CAN on Teensy 3.1, 3.2, 3.5 and 3.6
 * By Pawelsky
 * 
 */

#include "FlexCAN.h"

#ifndef __MK20DX256__
  #error "Teensy 3.2 is required to run"
#endif

//DEBUG - Only enable 1 at a time
const int debugCANBUS = 0;    //CANBus Receive
const int debugADC = 1;       //ADC Signals
const int debugADCPlot = 0  ;       //ADC Signals
const int debugFailure = 0;   //Failure CANBus Send
const int debugBSP = 0;      //Brake Current Plausibility
const int debugMC = 0;        //debug MC outputs

//TORQUE LIMITER
volatile int torqueMultiplier = 100; //Limit the max torque value to MC, range 0 -> 1

//TRACTION CONTROL
const int allowTraction = 0;

//REGEN
const int allowRegen = 1;
const int regenValue = 2500; //16384 for maximum regen

//SENSOR VALUES
const int sensorMCLimit = 16384;//16384 for 50%, 32766 for 100%
//Raw read values
int32_t sensorThrottle1Value = 0;
int32_t sensorThrottle2Value = 0;
int32_t sensorBrakeValue = 0;
int32_t sensorCurrentValue = 0;
//Mapped values
int32_t sensorThrottle1Mapped = 0;
int32_t sensorThrottle2Mapped = 0;
int32_t sensorBrakeMapped = 0;
//Raw Signal Ranges
const int32_t throttle1Min = 2650;
const int32_t throttle1Max = 6700;
const int32_t throttle2Min = 1500;
const int32_t throttle2Max = 5900;
const int32_t brakeMin = 1200;
const int32_t brakeMax = 6000;
//Throttle Difference
float throttleDiffP = 0;

//Dash Values
int dashSpeed = 0;
int dashBrakePressure = 0;
float dashTractionMultiplier = 0;
bool dashButtonToggle = false;
int dashBMSCellTemp = 0;

//For remote control
const int mcMaxAutonomousLimit = 1000; //8192 = 16384/2
boolean remoteControlEnabled = false;
const int maxControlInput = 255;
int speedVal = 0;
int32_t throttleVal = 0;

//CANBUS Setup
static CAN_message_t txmsg_mc, txmsg_mcthrottle, txmsg_failures, rxmsg;
static uint8_t hex[17] = "0123456789abcdef";
FlexCAN CANbus(1000000);
const int CANID_MC_TX = 0x181;      //receive mc on can id
const int CANID_MC_RX = 0x191;      //Send throttle on can id
const int CANID_FAILURES = 0x250;   //Send failures on can id
const int CANID_ORIONBMS = 0x3B;    //receive OrionBMS Main Channel
const int CANID_ORIONBMS2 = 0x6B2;  //receive OrionBMS secondary channel
const int CANID_PDMRESET = 0x522;   //Receive PDM Reset signal
const int CANID_DASH = 0x193;       //Receive from DASH
const int CANID_REMOTE_CONTROL = 0x144;     //Receive from jetson for remote control


//Motorcontroller Canbus Variable request
volatile int MCTimeoutCounter = 0;
bool MCVariableRequest = true; //While true, try and request variables from MC
//MCRequestArray Matrix {Received?, Reg Id[HEX], REGID_READ **Always 0x3D**, TIMING [ms]}
int MCRequestArray[8][4] = {
  {0, 0x26, 0x3D, 100},   //I_cmd
  {0, 0x5F, 0x3D, 100},   //I_actual (filtered)
  {0, 0xC6, 0x3D, 100},   //I_device
  {0, 0xEB, 0x3D, 100},    //DC-BUS Voltage
  {0, 0xD9, 0x3D, 200},
  {0, 0xA8, 0x3D, 100},   //N_actual (filtered)
  //{0, 0x49, 0x3D, 100},   //T_Motor temp
  //{0, 0x4B, 0x3D, 100},   //T_air MC temp
  //{0, 0x30, 0x3D, 100},   //N_actual Speed actual RPM
  {0, 0x20, 0x3D, 100},   //I_actual
  {0, 0x22, 0x3D, 100}   //I_cmd_ramp (C  urrent command after ramping)
  
};

//PDMReset
int pdmReset = false;

//OrionBMS
float bmsCurrent = 0;

//Failures
const int32_t thresholdHardBraking = 25000; //Value set for hardbraking
bool failureSensor = false; //If a sensor is disconnect or reading a value of less than 10; Used in function sensorFailure()
bool failureThrottleBrakeCheck = false; //If hard braking with throttle; Used in function torqueBrakeCheck()
bool failureThrottleImplausibility = false; //If throttle signals differ by 10%, used in throttleImplausibility()

//Pin List
const int ledPin = 13;
const int SDRelayPin = 18;
const int RelayResetPin = 20;
const int ADCThrottle1Pin = 14;
const int ADCThrottle2Pin = 15;
const int ADCBrakePin = 17;
const int ADCCurrentPin = 16;
const int FaultCurrentBrakePin = 19;

//Timers -- MAX 4 TIMERS
IntervalTimer intervalMCVariableTimeout;      //MC_variable timeout timer
IntervalTimer intervalReadSensor;             //Timer to read sensor values
IntervalTimer intervalSendThrottle;           //Timer to read sensor values
IntervalTimer intervalThrottleBrakeCheck;     //Timer to check throttle Brake plausibility

//Elapsed Timers
elapsedMillis sinceSendFailures;
elapsedMillis sinceThrottleImplausibility;
elapsedMillis sinceSensorFailure;

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.println(F("Teensy 3.2 - BSPD"));
  pinMode(ledPin, OUTPUT);
  pinMode(SDRelayPin, OUTPUT);
  pinMode(RelayResetPin, OUTPUT);
  pinMode(FaultCurrentBrakePin, INPUT);
  

  analogReference(DEFAULT);
  analogReadRes(14);
  analogReadAveraging(16);

  //CANBUS Setup
  //Can Mask
  CAN_filter_t canMask;
  canMask.id = 0xFFFFFF;
  canMask.rtr = 0;
  canMask.ext = 0;

  //Can Filter
  CAN_filter_t canFilter;

  //Begin CANBUS
  CANbus.begin(canMask);

  //SET Filters
  canFilter.id = CANID_MC_TX;     //Motor Controller Canbus
  CANbus.setFilter(canFilter, 0);
  //canFilter.id = CANID_ORIONBMS;  //OrionBMS
  CANbus.setFilter(canFilter, 1);
  CANbus.setFilter(canFilter, 2);
  canFilter.id = CANID_DASH;      //DASH
  CANbus.setFilter(canFilter, 3);
  //canFilter.id = CANID_ORIONBMS2; //OrionBMS2
  CANbus.setFilter(canFilter, 4);
  
  CANbus.setFilter(canFilter, 5);
  canFilter.id = CANID_PDMRESET;   //PDM
  CANbus.setFilter(canFilter, 6);
  canFilter.id = CANID_REMOTE_CONTROL;  //from jetson for remote control
  CANbus.setFilter(canFilter, 7);

  //Set default values for MC Throttle send can message
  txmsg_mcthrottle.id = CANID_MC_RX;
  txmsg_mcthrottle.len = 3;
  txmsg_mcthrottle.buf[0] = 0x90;
  txmsg_mcthrottle.buf[1] = 0;
  txmsg_mcthrottle.buf[2] = 0;

  //Set default values for Sendfailtures can message
  txmsg_failures.id = CANID_FAILURES;
  txmsg_failures.len = 1;
  txmsg_failures.buf[0] = 0;

 //SDRelayTest --ENABLING FOR TESTING
  digitalWrite(SDRelayPin, 1);
  
  //Begin Interrupt Timers -- MAX 4 TIMERS
  intervalMCVariableTimeout.begin(timeoutMC_variable, 1000000);           //Timer timeout for MC_variable_request, in 5 seconds
  intervalReadSensor.begin(readSensor, 40000);                            //Interrupt every 40ms 25hz to read sensors
  intervalSendThrottle.begin(sendThrottle, 40000);                        //Interrupt every 40ms 25hz to send throttle signal
    
  intervalThrottleBrakeCheck.begin(throttleBrakeCheck, 1000000);          //Interrupt every 1000ms, Timer to check throttle Brake plausibility

  //Interrupt Priorities
  intervalSendThrottle.priority(100);                                     //Default priority is 128;
 
}


// ------------------------------------------------------------- 
void loop(void)
{

  //Read Canbus
  if(CANbus.read(rxmsg))
  {
    if(debugCANBUS)
    {
      Serial.print("ID:");
      Serial.print(rxmsg.id, HEX);
      Serial.print(" DLC:");
      Serial.print(rxmsg.len, HEX);
      Serial.print("DATA:");
      Serial.print(rxmsg.buf[0], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[1], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[2], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[3], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[4], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[5], HEX);
      Serial.print(" ");
      Serial.print(rxmsg.buf[6], HEX);
      Serial.print(" ");
      Serial.println(rxmsg.buf[7], HEX);
    }
    
    //Received message from MC and MC_variable request is true
    if((rxmsg.id == CANID_MC_TX) && MCVariableRequest)
    {
      for(int i = 0; i < (int)(sizeof(MCRequestArray)/sizeof(MCRequestArray[0])); i++)
      {
        //Serial.println(rxmsg.buf[1], HEX);
        if(rxmsg.buf[0] == MCRequestArray[i][1])
        {
          MCRequestArray[i][0] = 1;
        }
      }
    }
    //Received message from PDM Reset
    //Byte0         
    //PDM Reset
    else if(rxmsg.id == CANID_PDMRESET)
    {
      if(pdmReset != rxmsg.buf[0])
      {
        noInterrupts();
        pdmReset = rxmsg.buf[0];
        digitalWrite(RelayResetPin, pdmReset);
        interrupts();
      }
    }
    //Received message from DASH
    //Byte0         
    //Button Torque Adjuster
    else if(rxmsg.id == CANID_DASH)
    {
      noInterrupts();
      dashSpeed = word(rxmsg.buf[0], rxmsg.buf[1]);
      dashBrakePressure = word(rxmsg.buf[2], rxmsg.buf[3]);
      dashTractionMultiplier = (float) rxmsg.buf[5]/100;
      dashBMSCellTemp = rxmsg.buf[6];
      
      if((rxmsg.buf[4] == 1)&&!dashButtonToggle)
      {
        torqueMultiplier = torqueMultiplier - 10;
        if(torqueMultiplier < 0)
          torqueMultiplier = 100;

        dashButtonToggle = true;
        
        
      }
      else if((rxmsg.buf[4] == 0)&&dashButtonToggle)
      {
        dashButtonToggle = false;
      }
      interrupts();
    }
    else if(rxmsg.id == CANID_REMOTE_CONTROL)
    {
      // check if remote control is being activated or not
      if( rxmsg.buf[0] == 0 || rxmsg.buf[0] == 1 ) {
        if( rxmsg.buf[0] == 0 ) remoteControlEnabled = false;
        else remoteControlEnabled = true;
      }
      else { remoteControlEnabled = false; }

      speedVal = rxmsg.buf[2];
    }
  }
  


  //Elapsed Timers
  //Check Throttle Implausibility every 1000ms
  if(sinceThrottleImplausibility > 1000)
  {
    throttleImplausibility();
    sinceThrottleImplausibility = 0;
  }
  //Check Sensor Failture every 1000ms
  if(sinceSensorFailure > 1000)
  {
    sensorFailure();
    sinceSensorFailure = 0;
  }
  //Send Failures every 1000ms
  if(sinceSendFailures > 1000)
  {
    sendFailures();
    sinceSendFailures = 0;
  }
  
  
  if(debugADC)
  {
    //Serial.print("CB_FAULT:");
    //Serial.print(digitalRead(FaultCurrentBrakePin));
    Serial.print("Autonomous mode");
    Serial.print(remoteControlEnabled);
    Serial.print("\tT1raw:");
    Serial.print(sensorThrottle1Value);
    Serial.print(" \tT1map:");
    Serial.print(sensorThrottle1Mapped);
    Serial.print("\tT2raw:");
    Serial.print(sensorThrottle2Value);
    Serial.print("\tT2map:");
    Serial.print(sensorThrottle2Mapped);
    Serial.print("\tTRACTION:");
    Serial.print(dashTractionMultiplier);
    //Serial.print("\tThrDiff:");
    //Serial.print(throttleDiffP);
    //Serial.print("\tBraw:");
    //Serial.print(sensorBrakeValue);
    //Serial.print("\tB:");
    //Serial.print(sensorBrakeMapped);
    //Serial.print("\tC:");
    //Serial.print(sensorCurrentValue);
    Serial.print("\tdashSpeed:");
    Serial.println(dashSpeed);
    //Serial.print("\tdashBrake:");
    //Serial.print(dashBrakePressure);
    //Serial.print("\tTorqueMulti:");
    //Serial.print(torqueMultiplier);
    //Serial.print("\tdashTC:");
    //Serial.println(dashTractionMultiplier);
    
  }
  else if(debugADCPlot)
  {
    Serial.print(sensorThrottle1Mapped);
    Serial.print(",");
    Serial.print(32767);
    Serial.print(",");
    Serial.print(0);
    Serial.print(" ");
    
    Serial.print(sensorThrottle2Mapped);
    Serial.print(",");
    Serial.print(32767);
    Serial.print(",");
    Serial.print(0);
    Serial.println("");
  }
  else if(debugFailure)
  {
    Serial.print("F_BSPD:");
    Serial.print(!digitalRead(FaultCurrentBrakePin));
    Serial.print("\tF_ThrottleBrake:");
    Serial.print(failureThrottleBrakeCheck);
    Serial.print("\tF_Sensor:");
    Serial.print(failureSensor);
    Serial.print("\tF_ThrottleImplausibility:");
    Serial.print(failureThrottleImplausibility);
    Serial.print(" \tT1map:");
    Serial.print(sensorThrottle1Mapped);
    Serial.print("\tB:");
    Serial.print(sensorBrakeMapped);
    Serial.println("");
  }
  else if(debugMC)
  {
    for(int i = 0; i < (int)(sizeof(MCRequestArray)/sizeof(MCRequestArray[0])); i++)
    {
      if(MCRequestArray[i][0] == 0)
        Serial.println(MCRequestArray[i][1], HEX);
    }
  }
  
}

void timeoutMC_variable(void)
{
  MCTimeoutCounter++;

  //MC_variable request
  //MCRequestArray
  for(int i = 0; i < (int)(sizeof(MCRequestArray)/sizeof(MCRequestArray[0])); i++)
  {
    if(MCRequestArray[i][0] == 0) //Check if being received yet
    {
      //Serial.println("SEND");
      txmsg_mc.id = CANID_MC_RX;
      txmsg_mc.len = 3;
      txmsg_mc.buf[0] = MCRequestArray[i][2];
      txmsg_mc.buf[1] = MCRequestArray[i][1];
      txmsg_mc.buf[2] = MCRequestArray[i][3];
      CANbus.write(txmsg_mc);
    }
  }
  
  //End Timer after 5 seconds
  if(MCTimeoutCounter >= 5)
  {
    MCVariableRequest = false;
    intervalMCVariableTimeout.end();
    CAN_filter_t canFilter;
    canFilter.id = CANID_PDMRESET; //PDMReset
    CANbus.setFilter(canFilter, 0);
  }
}

void readSensor(void)
{
  noInterrupts();
  sensorThrottle1Value = analogRead(ADCThrottle1Pin); 
  sensorThrottle2Value = analogRead(ADCThrottle2Pin); 
  sensorBrakeValue = analogRead(ADCBrakePin); 
  sensorCurrentValue = analogRead(ADCCurrentPin); 

  //Mapping with Fixed Values
  sensorThrottle1Mapped = constrain(map(sensorThrottle1Value,throttle1Min,throttle1Max,0,sensorMCLimit),0,sensorMCLimit);
  sensorThrottle2Mapped = constrain(map(sensorThrottle2Value,throttle2Min,throttle2Max,0,sensorMCLimit),0,sensorMCLimit);

  if( remoteControlEnabled == false) {
    throttleVal = sensorThrottle1Mapped;
  }
  else if( remoteControlEnabled == true) {

    //map the vehicleSpeed from 0 to maxControlInput to the sensorMCLimit and put a constrain around it
    // to make it stable.
    throttleVal = constrain(map(speedVal, 0, maxControlInput, 0, mcMaxAutonomousLimit), 0, sensorMCLimit);
  }
  
  sensorBrakeMapped = constrain(map(sensorBrakeValue,brakeMin,brakeMax,0,sensorMCLimit),0,sensorMCLimit);
  interrupts();
}

void sendThrottle(void)
{
  noInterrupts(); 
  //Send Throttle to Motor Controller
  int32_t throttleMC = (throttleVal) * ((float) torqueMultiplier) / 200;

  //FOR TRACTION CONTROL
  if(allowTraction && (torqueMultiplier >=60)){
    if((dashTractionMultiplier > 0) && (dashTractionMultiplier < 1))
      throttleMC *= (float) dashTractionMultiplier;
  }
  
  //FOR REGEN
  //If speed is above 5kph
  if(allowRegen && (dashBMSCellTemp < 45) && (torqueMultiplier <= 50)){ //Allow Regen and if cell temp is below 45
    if((dashSpeed > 32800)&&(dashSpeed < 62000)&&(throttleMC == 0)){
      throttleMC = regenValue * -1;
    }
  }

  //Send Throttle to MC
  txmsg_mcthrottle.buf[1] = ~((int) throttleMC % 256);
  txmsg_mcthrottle.buf[2] = ~((int) (throttleMC) / 256);
  if(throttleMC == 0){
    txmsg_mcthrottle.buf[1] = 0;
    txmsg_mcthrottle.buf[2] = 0;
  }

  CANbus.write(txmsg_mcthrottle);
  interrupts();
}

void throttleBrakeCheck()
{
  //cli();
  noInterrupts();
  // Torque encoder is at more than 25% and brake is actuated
  // simultaneously. The motors have to shut down. The motor power shut
  // down has to remain active until the torque encoder signals less than
  // 5% pedal travel, no matter whether the brake pedal is still actuated or
  // not.
  // implausibilityTog
  
  if(failureThrottleBrakeCheck){
    if(sensorThrottle1Mapped < (sensorMCLimit* (((float) torqueMultiplier) / 200) * 0.05)) //5% of 16384
      failureThrottleBrakeCheck = false;

  }else{
    if(sensorThrottle1Mapped > (sensorMCLimit* (((float) torqueMultiplier) / 200) * 0.25)){ //25% of 16384
      if(sensorBrakeMapped > thresholdHardBraking){
        //failureThrottleBrakeCheck = true;
      }
    }
  }
  interrupts();
  //sei();
  
}    

void sensorFailure()
{
  // When an analogue signal is used, e.g. from a 5V sensor, the brake system
  // encoder sensors will be considered to have failed when they achieve an
  // open circuit or short circuit condition which generates a signal outside
  // of the normal operating range, for example <0.5V or >4.5V. The circuitry
  // used to evaluate the sensor wilsl use pull down or pull up resistors to
  // ensure that open circuit signals result in a failure being detected
  noInterrupts();
  failureSensor = false;
  
  if(sensorBrakeValue < 300)
    failureSensor = true;
  
  if(sensorThrottle1Value < 300)
    failureSensor = true;
  
  if(sensorThrottle2Value < 300)
    failureSensor = true;

  interrupts();
}

void throttleImplausibility(){
  // If implausibility occurkhs between the values of two torque encoder
  // sensors the power to the motor has to be immediately shut down
  // completely. It is not necessary to completely deacticvate the Tractive
  // System.
  // 10% Difference
  
  //Get Percentage difference
  noInterrupts();
  throttleDiffP = ((float)sensorThrottle1Mapped + 5000) / ((float)sensorThrottle2Mapped + 5000);//((throttle1Read / throttle2Read) * 100);
  //if((sensorThrottle1Value < 100)||(sensorThrottle2Value < 100))
  //  throttleDiffP = 0;
  
    //Use throttle Difference percentage to open or close 
  if((0.8 < throttleDiffP)&&(throttleDiffP < 1.2)){  
      failureThrottleImplausibility = false;   //Plausible
  }else{
   // failureThrottleImplausibility = true;    //Implausible
  }  
  interrupts();
}

void sendFailures()
{
  //byteFailures = failureThrottleBrakeCheck || (failureSensor << 1) || (failureThrottleImplausibility << 2) || (digitalRead(FaultCurrentBrakePin) << 3);

  //Serial.println("SEND FAILURE");
  
  txmsg_failures.len = 5;
  noInterrupts();
  txmsg_failures.buf[0] = !digitalRead(FaultCurrentBrakePin);
  txmsg_failures.buf[1] = failureSensor;
  txmsg_failures.buf[2] = failureThrottleBrakeCheck;
  txmsg_failures.buf[3] = failureThrottleImplausibility;
  txmsg_failures.buf[4] = torqueMultiplier;
  
    
  CANbus.write(txmsg_failures);
  interrupts();
}
