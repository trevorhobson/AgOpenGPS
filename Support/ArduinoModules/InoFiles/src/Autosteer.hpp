#pragma once

#include <Arduino.h>
  
#include "EtherCard_AOG.h"
#include <IPAddress.h>
//#include <SparkFun_BNO080_Arduino_Library.h>
#include "BNO08x_AOG.h"
  
  // ethernet interface ip address
  static uint8_t myip[] = { 192,168,1,73 };
  // gateway ip address
  static uint8_t gwip[] = { 192,168,1,1 };
  //DNS- you just need one anyway
  static uint8_t myDNS[] = { 8,8,8,8 };
  //mask
  static uint8_t mask[] = { 255,255,255,0 };
  //this is port of this autosteer module
  uint16_t portMy = 8888; 
  
  //sending back to where and which port
  static uint8_t ipDestination[] = {192, 168, 1, 255};
  uint16_t portDestination = 9999; //AOG port that listens
  
  // ethernet mac address - must be unique on your network
  static uint8_t mymac[] = { 0x70,0x69,0x69,0x2D,0x30,0x31 };
  
  uint8_t Ethernet::buffer[200]; // udp send and receive buffer
    
  //loop time variables in microseconds  
  const uint16_t LOOP_TIME = 25;  //40Hz    
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;

  const uint16_t WATCHDOG_THRESHOLD = 100;
  const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
  uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
  
   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;

  //show life in AgIO
  uint8_t helloAgIO[] = {0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };
  uint8_t helloCounter=0;

  //fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
  int16_t AOGSize = sizeof(AOG);

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;

  // BNO08x address variables to check where it is
  const uint8_t bno08xAddresses[] = {0x4A,0x4B};
  const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses)/sizeof(bno08xAddresses[0]);
  uint8_t bno08xAddress;
  BNO080 bno08x;

  float bno08xHeading = 0;
  double bno08xRoll = 0;
  double bno08xPitch = 0;

  int16_t bno08xHeading10x = 0;
  int16_t bno08xRoll10x = 0;
 
  //EEPROM
  int16_t EEread = 0;

  //Relays
  bool isRelayActiveHigh = true;
  uint8_t relay = 0, relayHi = 0, uTurn = 0;
  uint8_t tram = 0;
    
  //Switches
  uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

  //On Off
  uint8_t guidanceStatus = 0;

  //speed sent as *10
  float gpsSpeed = 0;
  
  //steering variables
  float steerAngleActual = 0;
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  int16_t steeringPosition = 0; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  
  //pwm variables
  int16_t pwmDrive = 0, pwmDisplay = 0;
  float pValue = 0;
  float errorAbs = 0;
  float highLowPerDeg = 0; 

  //Steer switch button  ***********************************************************************************************************
  uint8_t currentState = 1, reading, previous = 0;
  uint8_t pulseCount = 0; // Steering Wheel Encoder
  bool encEnable = false; //debounce flag
  uint8_t thisEnc = 0, lastEnc = 0;

   //Variables for settings  
   struct Storage {
      uint8_t Kp = 40;  //proportional gain
      uint8_t lowPWM = 10;  //band of no action
      int16_t wasOffset = 0;
      uint8_t minPWM = 9;
      uint8_t highPWM = 60;//max PWM value
      float steerSensorCounts = 30;        
      float AckermanFix = 1;     //sent as percent
  };  Storage steerSettings;  //11 bytes

   //Variables for settings - 0 is false  
   struct Setup {
      uint8_t InvertWAS = 0;
      uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
      uint8_t MotorDriveDirection = 0;
      uint8_t SingleInputWAS = 1;
      uint8_t CytronDriver = 1;
      uint8_t SteerSwitch = 0;  //1 if switch selected
      uint8_t SteerButton = 0;  //1 if button selected
      uint8_t ShaftEncoder = 0;
      uint8_t PressureSensor = 0;
      uint8_t CurrentSensor = 0;
      uint8_t PulseCountMax = 5; 
      uint8_t IsDanfoss = 0; 
  };  Setup steerConfig;          //9 bytes

//ISR Steering Wheel Encoder
void EncoderFunc();
