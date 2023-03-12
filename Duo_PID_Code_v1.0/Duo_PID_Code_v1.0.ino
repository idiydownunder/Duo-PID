//======================================================================
//  Duo PID Control
//  ---------------
//
//  Sketch for Duo PID Control project.
//
//  Software Version: 1.0 
//  For PCB Revisions 1.0 to 2.1
//
//  Copyright (c) 2023 iDIY Down Under
//  Author: Julian Carmichael (aka Digital Jester)
//
//  This work is licensed under a Creative Commons 
//  Attribution-NonCommercial-ShareAlike 4.0 International License
//  http://creativecommons.org/licenses/by-nc-sa/4.0/
//
//======================================================================

//===================================
// Include Libarys
//===================================
#include <EEPROM.h>
#include <Wire.h>
#include "ssd1306.h"
#include "ssd1306_console.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
//=================================== 
// Define Button Constants
//===================================
#define btnRIGHT  5
#define btnLEFT   4
#define btnUP     3
#define btnDOWN   2
#define btnSELECT 1
#define btnNONE   0
#define BUTTON_DEBOUNCE 500     // button debounce value
//===================================
// Define Pins
//===================================
#define BUTTON_PIN A3       // Button Array
#define HEAT_1_PIN 4        // PID 1 Heating SSR Pin
#define COOL_1_PIN 5        // PID 1 Cooling SSR Pin
#define ONE_WIRE_BUS_PIN 6  // Temperature Sensor Pin
#define COOL_2_PIN 7        // PID 2 Heating SSR Pin
#define HEAT_2_PIN 8        // PID 2 Cooling SSR Pin
//=================================== 
// Define DS18B20 Constants
//===================================
#define TEMPERATURE_PRECISION 12  // Values 9, 10, 11 and 12 bit
#define tempMax 120               // Sensor MAX value = +125
#define tempMin -50               // Sensor MIN value = -55
//=================================== 
// Define Display Constants
//===================================
#define main 0
#define pid1s 1
#define pid1t 2
#define pid2s 3
#define pid2t 4
#define pidg 5
#define pidtn 6
#define set 7
//=================================== 
// Define Default Constants
//===================================
#define defSetpoint 26.0
#define defKp 5.0
#define defKi 2.5
#define defKd 5.0

const unsigned char PROGMEM idiylogo[] = {
0x00, 0x00, 0x00, 0x1C, 0x3E, 0xFE, 0x3E, 0x3E, 0x3E, 0x1C, 0x10, 0xE0, 0x00, 0x02, 0x02, 0x12,
0x0E, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x02, 0xE2, 0x06, 0x06, 0x1E, 0xFC, 0xFC,
0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x06, 0xFE, 0xFE, 0xFE,
0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x02, 0xC2, 0x02, 0x02, 0x02, 0x06, 0x0E, 0x1E, 0x3E, 0xFE, 0xFE,
0xFE, 0xFE, 0xFE, 0xFE, 0xF2, 0xC2, 0x82, 0x00, 0x10, 0x00, 0x82, 0xC2, 0x72, 0x1E, 0x86, 0x86,
0x02, 0x02, 0x00, 0x10, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF,
0x00, 0x00, 0x00, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x18, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xFE, 0x03,
0xF0, 0x1C, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x3F, 0xFF,
0x3F, 0x3F, 0x3F, 0x3F, 0x20, 0xA0, 0x00, 0x20, 0x20, 0x20, 0x38, 0x3F, 0xFF, 0x3F, 0x3F, 0x3F,
0x3F, 0x3F, 0x3F, 0x20, 0xEF, 0x20, 0x30, 0x3C, 0xBF, 0x7F, 0x1F, 0x1F, 0x1F, 0x0F, 0x87, 0x43,
0x01, 0x10, 0x0E, 0x00, 0x20, 0x20, 0x30, 0x3F, 0xBF, 0x7F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x20,
0x6F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x30, 0x3F, 0xBF, 0x3F, 0x3F,
0x3F, 0x3F, 0x3F, 0x3F, 0x20, 0x6F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00,
0x30, 0x78, 0x30, 0x40, 
};

int displayState=main;     // enumerated variable that holds state for display state

unsigned long last_press;     // Hold Last Button Press, Used to Debounce User Input
double temp_buffer = 1.5;     // Holds the buffer value for PID heat/cool swing
double temp_inc = 0.5;        // Holds stepper value for Temp incress/decress
double pidGAT = 10;           // Holds value for Adaptive Tuning GAP
bool pid1HeatOnly = true;     // Used to hold the PID in heat only mode
bool pid1CoolOnly = false;    // Used to hold the PID in cool only mode
bool pid1AddTune=false;       // Used to toggle Adaptive Tuning mode
bool pid1PonE=false;          // PID P calculation method
bool pid1AS=true;             // Used to auto start PID at start up
bool pid2HeatOnly = true;     // Used to hold the PID in heat only mode
bool pid2CoolOnly = false;    // Used to hold the PID in cool only mode
bool pid2AddTune=false;       // Used to toggle Adaptive Tuning mode
bool pid2PonE=false;          // PID P calculation method
bool pid2AS=true;             // Used to auto start PID at start up
bool showTuneNote = true;     // Toggle Tuning Notes on/off
bool showPIDTune=true;        // Toggle PID tuning on/off
bool showValPer=false;        // Display output as value or percent
bool isF = false;             // Used to display Fahrenheit temps
int placeMarker = 1;          // Used to mark setting options
int placeMax = 0;             // Used to define how many options are in menu
int senCount=0;               // Used to hold the number of temp sensors found at start up

//PID Relay variables
int WindowSize = 5000;        // Used to define PID Timing Window
unsigned long windowStartTimeTun1;  // Holds PID current timing
unsigned long windowStartTimeTun2;  // Holds PID current timing

//Define Variables we'll be connecting to for PID control
double pid1_Setpoint, pid1_Input, pid1_Output;
double pid2_Setpoint, pid2_Input, pid2_Output;

//Specify the links and initial tuning parameters
double pid1_Kp=1.2, pid1_Ki=0.2, pid1_Kd=0.5;
double pid2_Kp=1.2, pid2_Ki=0.2, pid2_Kd=0.5;
double pid1_a_Kp=2, pid1_a_Ki=0.5, pid1_a_Kd=2;
double pid2_a_Kp=2, pid2_a_Ki=0.5, pid2_a_Kd=2;

// Setup PID's
PID PID_1(&pid1_Input, &pid1_Output, &pid1_Setpoint, pid1_Kp, pid1_Ki, pid1_Kd, P_ON_M, DIRECT);
PID PID_2(&pid2_Input, &pid2_Output, &pid2_Setpoint, pid2_Kp, pid2_Ki, pid2_Kd, P_ON_M, DIRECT);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS_PIN);

DallasTemperature sensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress PID_1_TEMP, PID_2_TEMP;   // arrays to hold device addresses

// setup OLED display
Ssd1306Console  display;

//------------------------------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------------------------------
void setup() {
  
  ssd1306_128x64_i2c_init();  // start oled display

  sensors.begin();            //start temp sensors

  int y = 0;
  int inc =8;
  int del = 500;
  
  ssd1306_clearScreen();
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  display.setCursor(0,0); 
  display.print(F("Loading Settings..."));
  LOAD_SETTINGS();  // Load Settings
  delay(del);

  y+=inc;
  display.setCursor(0,y);
  display.print(F("Checking Settings..."));
  CHK_SETTINGS(); // Check Run Time Settings
  delay(del);
  ssd1306_clearScreen();
  y=0;
  display.setCursor(0,y);
  display.print(F("Locating devices..."));
  delay(del);
  
  y+=inc;
  display.setCursor(0,y);
  display.print(F("Found "));
  senCount=sensors.getDeviceCount();
  display.print(senCount);
  display.print(F(" devices."));
  delay(del);
  
  y+=inc;
  display.setCursor(0,y);
  if (!sensors.getAddress(PID_1_TEMP, 0)) {display.print(F("FAIL: PID_1_TEMP"));y+=inc;display.setCursor(0,y);delay(1200);}
  if (!sensors.getAddress(PID_2_TEMP, 1)) {display.print(F("FAIL: PID_2_TEMP"));y+=inc;display.setCursor(0,y);delay(1200);}
  delay(del);

  y+=inc;
  display.setCursor(0,y);
  display.print(F("Set device Percision"));
  sensors.setResolution(PID_1_TEMP, TEMPERATURE_PRECISION);
  sensors.setResolution(PID_2_TEMP, TEMPERATURE_PRECISION);
  delay(del);
  
  ssd1306_clearScreen();
  y=0;
  display.setCursor(0,y);
  display.print(F("Set I/O Pin Operation"));
  pinMode(HEAT_1_PIN,OUTPUT);
  pinMode(HEAT_2_PIN,OUTPUT);
  pinMode(COOL_1_PIN,OUTPUT);
  pinMode(COOL_2_PIN,OUTPUT);
  delay(del);

  y+=inc;
  display.setCursor(0,y);
  display.print(F("Set Time Markers."));
  windowStartTimeTun1 = millis();
  windowStartTimeTun2 = millis();
  last_press=millis();
  delay(del);
  
  ssd1306_clearScreen();
  y=0;
  display.setCursor(0,y);
  display.print(F("Set PID Limits."));
  setPIDLimit();
  delay(del);

  y+=inc;
  display.setCursor(0,y);
  display.print(F("Set PID Tuning."));
  setPIDTuning();
  delay(del);

  y+=inc;
  display.setCursor(0,y);
  display.print(F("Run PID Auto Start."));
  //turn the PID's on
  if(pid1AS){if(senCount>=1){PID_1.SetMode(AUTOMATIC);}}
  if(pid2AS){if(senCount>=2){PID_2.SetMode(AUTOMATIC);}}
  delay(del);
  
  ssd1306_clearScreen();
  ssd1306_printFixed(10,  35, "DUO PID", STYLE_BOLD);
  ssd1306_printFixed(19, 45, "CONTROL", STYLE_BOLD);
  ssd1306_printFixed(32, 52, "Version: 1.0", STYLE_ITALIC);
  ssd1306_drawBitmap(43, 32, 85, 26, idiylogo);
  delay(3000);
  
  ssd1306_clearScreen();

  displayState=main;
  
}
//------------------------------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------------------------------
void loop() {

  sensors.requestTemperatures(); // Send the command to get temperatures

  runPID(); // Run the PID's  

  // Handel User Input
  if ((millis()-last_press) > BUTTON_DEBOUNCE){
    button_handler();               
  }

  oledDisplay(); // Update Display
  
}
//------------------------------------------------------------------------------------------------------
// This routine is used to run and control how the PID's behave.
//------------------------------------------------------------------------------------------------------
void runPID(){

  // Check to see there is an input for PID
  if(senCount>=1){
    pid1_Input = sensors.getTempC(PID_1_TEMP);      // Update PID input temperature
    if(pid1_Input==-127.00){PID_1.SetMode(MANUAL);} // Something went wrong turn off PID
  }else{
    pid1_Input = -127.00;
    PID_1.SetMode(MANUAL);  // Something went wrong turn off PID
  }

  if (PID_1.GetMode()==AUTOMATIC){  // See if PID is on or off    
    if (pid1_Input>=pid1_Setpoint+temp_buffer){
      if(!pid1HeatOnly){
        PID_1.SetControllerDirection(REVERSE);
        digitalWrite(HEAT_1_PIN, LOW);
      }else{
        PID_1.SetControllerDirection(DIRECT);
        digitalWrite(COOL_1_PIN, LOW);
      }
    }else if (pid1_Input<=pid1_Setpoint-temp_buffer){
      if(!pid1CoolOnly){
        PID_1.SetControllerDirection(DIRECT);
        digitalWrite(COOL_1_PIN, LOW);
      }else{
        PID_1.SetControllerDirection(REVERSE);
        digitalWrite(HEAT_1_PIN, LOW);
      }
    }

    if(pid1AddTune){ // Is Adapive tuning mode being used
      if(abs(pid1_Setpoint-pid1_Input)<=pidGAT){
        if(pid1PonE){
          PID_1.SetTunings(pid1_Kp, pid1_Ki, pid1_Kd,P_ON_E);
        }else{
          PID_1.SetTunings(pid1_Kp, pid1_Ki, pid1_Kd,P_ON_M);
        }
      }else{
        if(pid1PonE){
          PID_1.SetTunings(pid1_a_Kp, pid1_a_Ki, pid1_a_Kd,P_ON_E);
        }else{
          PID_1.SetTunings(pid1_a_Kp, pid1_a_Ki, pid1_a_Kd,P_ON_M);
        }
      }
    }
  
    PID_1.Compute(); // Run PID calcutaions

    // Track PID on/off timing
    if (millis() - windowStartTimeTun1 > WindowSize){ 
      windowStartTimeTun1 = millis();
    }

    // Update PID outputs
    if (PID_1.GetDirection()==DIRECT){
      if (pid1_Output > millis() - windowStartTimeTun1){
        digitalWrite(HEAT_1_PIN, HIGH);
      } else {
        digitalWrite(HEAT_1_PIN, LOW);
      }
    }
    if (PID_1.GetDirection()==REVERSE){
      if (pid1_Output > millis() - windowStartTimeTun1){
        digitalWrite(COOL_1_PIN, HIGH);
      } else {
        digitalWrite(COOL_1_PIN, LOW);
      }
    }
  }else{
    // Make sure PID outputs are off
    pid1_Output=0;
    digitalWrite(HEAT_1_PIN, LOW);
    digitalWrite(COOL_1_PIN, LOW);
  }

  if(senCount>=2){
    pid2_Input = sensors.getTempC(PID_2_TEMP); // Update PID input temperature
    if(pid2_Input==-127.00){PID_2.SetMode(MANUAL);}
  }else{
    pid2_Input = -127.00;
    PID_2.SetMode(MANUAL);
  }

  if (PID_2.GetMode()==AUTOMATIC){
    if (pid2_Input>=pid2_Setpoint+temp_buffer){
      if(!pid2HeatOnly){
        PID_2.SetControllerDirection(REVERSE);
        digitalWrite(HEAT_2_PIN, LOW);
      }else{
        PID_2.SetControllerDirection(DIRECT);
        digitalWrite(COOL_2_PIN, LOW);
      }
    }else if (pid2_Input<=pid2_Setpoint-temp_buffer){
      if(!pid2CoolOnly){
        PID_2.SetControllerDirection(DIRECT);
        digitalWrite(COOL_2_PIN, LOW);
      }else{
        PID_2.SetControllerDirection(REVERSE);
        digitalWrite(HEAT_2_PIN, LOW);
      }
    }

    if(pid2AddTune){
      if(abs(pid2_Setpoint-pid2_Input)<=pidGAT){
        if(pid2PonE){
          PID_2.SetTunings(pid2_Kp, pid2_Ki, pid2_Kd,P_ON_E);
        }else{
          PID_2.SetTunings(pid2_Kp, pid2_Ki, pid2_Kd,P_ON_M);
        }
      }else{
        if(pid2PonE){
          PID_2.SetTunings(pid2_a_Kp, pid2_a_Ki, pid2_a_Kd,P_ON_E);
        }else{
          PID_2.SetTunings(pid2_a_Kp, pid2_a_Ki, pid2_a_Kd,P_ON_M);
        }
      }
    }
    
    PID_2.Compute();
  
    if (millis() - windowStartTimeTun2 > WindowSize){ 
      windowStartTimeTun2 = millis();
    }

    if (PID_2.GetDirection()==DIRECT){
      if (pid2_Output > millis() - windowStartTimeTun2){
        digitalWrite(HEAT_2_PIN, HIGH);
      } else {
        digitalWrite(HEAT_2_PIN, LOW);
      }
    }
    if (PID_2.GetDirection()==REVERSE){
      if (pid2_Output > millis() - windowStartTimeTun2){
        digitalWrite(COOL_2_PIN, HIGH);
      } else {
        digitalWrite(COOL_2_PIN, LOW);
      }
    }
  }else{
    pid2_Output=0;
    digitalWrite(HEAT_2_PIN, LOW);
    digitalWrite(COOL_2_PIN, LOW);
  }
}
//------------------------------------------------------------------------------------------------------
// This routine is used to set the PID tuning options.
//------------------------------------------------------------------------------------------------------
void setPIDTuning(){
  if(pid1PonE){
    PID_1.SetTunings(pid1_Kp, pid1_Ki, pid1_Kd,P_ON_E);
  }else{
    PID_1.SetTunings(pid1_Kp, pid1_Ki, pid1_Kd,P_ON_M);
  }
  if(pid2PonE){
    PID_2.SetTunings(pid2_Kp, pid2_Ki, pid2_Kd,P_ON_E);
  }else{
    PID_2.SetTunings(pid2_Kp, pid2_Ki, pid2_Kd,P_ON_M);
  }
}
//------------------------------------------------------------------------------------------------------
// This routine is used to set the PID output limits.
//------------------------------------------------------------------------------------------------------
void setPIDLimit(){
  PID_1.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  PID_2.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
}
//------------------------------------------------------------------------------------------------------
// This routine is used to save settings.
//------------------------------------------------------------------------------------------------------
void SAVE_SETTINGS()
{
  EEPROM.put(0,isF);      
  EEPROM.put(1,pid1HeatOnly);
  EEPROM.put(2,pid1CoolOnly);
  EEPROM.put(3,pid1AddTune);
  EEPROM.put(4,pid1PonE);
  EEPROM.put(5,pid1AS);
  EEPROM.put(6,pid2HeatOnly);
  EEPROM.put(7,pid2CoolOnly);
  EEPROM.put(8,pid2AddTune);
  EEPROM.put(9,pid2PonE);
  EEPROM.put(10,pid2AS);
  EEPROM.put(11,showTuneNote);
  EEPROM.put(12,showPIDTune);
  EEPROM.put(13,showValPer);
  EEPROM.put(18,WindowSize);
  
  EEPROM.put(20,pidGAT);
  EEPROM.put(24,temp_buffer);
  EEPROM.put(28,temp_inc);
  EEPROM.put(32,pid1_Setpoint);
  EEPROM.put(36,pid1_Kp);
  EEPROM.put(40,pid1_Ki);
  EEPROM.put(44,pid1_Kd);
  EEPROM.put(48,pid1_a_Kp);
  EEPROM.put(52,pid1_a_Ki);
  EEPROM.put(56,pid1_a_Kd);
  //EEPROM.put(60,);
  EEPROM.put(64,pid2_Setpoint);
  EEPROM.put(68,pid2_Kp);
  EEPROM.put(72,pid2_Ki);
  EEPROM.put(76,pid2_Kd);
  EEPROM.put(80,pid2_a_Kp);
  EEPROM.put(84,pid2_a_Ki);
  EEPROM.put(88,pid2_a_Kd);
  //EEPROM.put(92,);
}
//------------------------------------------------------------------------------------------------------
// This routine is used to load settings.
//------------------------------------------------------------------------------------------------------
void LOAD_SETTINGS()
{
  EEPROM.get(0,isF);      
  EEPROM.get(1,pid1HeatOnly);
  EEPROM.get(2,pid1CoolOnly);
  EEPROM.get(3,pid1AddTune);
  EEPROM.get(4,pid1PonE);
  EEPROM.get(5,pid1AS);
  EEPROM.get(6,pid2HeatOnly);
  EEPROM.get(7,pid2CoolOnly);
  EEPROM.get(8,pid2AddTune);
  EEPROM.get(9,pid2PonE);
  EEPROM.get(10,pid2AS);
  EEPROM.get(11,showTuneNote);
  EEPROM.get(12,showPIDTune);
  EEPROM.get(13,showValPer);
  EEPROM.get(18,WindowSize);
  
  EEPROM.get(20,pidGAT);
  EEPROM.get(24,temp_buffer);
  EEPROM.get(28,temp_inc);
  EEPROM.get(32,pid1_Setpoint);
  EEPROM.get(36,pid1_Kp);
  EEPROM.get(40,pid1_Ki);
  EEPROM.get(44,pid1_Kd);
  EEPROM.get(48,pid1_a_Kp);
  EEPROM.get(52,pid1_a_Ki);
  EEPROM.get(56,pid1_a_Kd);
  //EEPROM.get(60,);
  EEPROM.get(64,pid2_Setpoint);
  EEPROM.get(68,pid2_Kp);
  EEPROM.get(72,pid2_Ki);
  EEPROM.get(76,pid2_Kd);
  EEPROM.get(80,pid2_a_Kp);
  EEPROM.get(84,pid2_a_Ki);
  EEPROM.get(88,pid2_a_Kd);
  //EEPROM.get(92,);
  
}
//------------------------------------------------------------------------------------------------------
// This routine is used to check settings for valid input or a default value is assigned.
//------------------------------------------------------------------------------------------------------
void CHK_SETTINGS(){
  if(isnan(pid1_Setpoint)){pid1_Setpoint=defSetpoint;}
  if(pid1_Setpoint<tempMin){pid1_Setpoint=defSetpoint;}
  if(pid1_Setpoint>tempMax){pid1_Setpoint=defSetpoint;}
  
  if(isnan(pid2_Setpoint)){pid2_Setpoint=defSetpoint;}
  if(pid2_Setpoint<tempMin){pid2_Setpoint=defSetpoint;}
  if(pid2_Setpoint>tempMax){pid2_Setpoint=defSetpoint;}
  
  if(isnan(pidGAT)){pidGAT=10;}
  if(pidGAT<1){pidGAT=1;}
  if(pidGAT>10){pidGAT=10;}
  
  if(isnan(pid1_Kp)){pid1_Kp=defKp;}
  if(pid1_Kp<0){pid1_Kp=defKp;}
  if(isnan(pid1_Ki)){pid1_Ki=defKi;}
  if(pid1_Ki<0){pid1_Ki=defKi;}
  if(isnan(pid1_Kd)){pid1_Kd=defKd;}
  if(pid1_Kd<0){pid1_Kd=defKd;}
  
  if(isnan(pid2_Kp)){pid2_Kp=defKp;}
  if(pid2_Kp<0){pid2_Kp=defKp;}
  if(isnan(pid2_Ki)){pid2_Ki=defKi;}
  if(pid2_Ki<0){pid2_Ki=defKi;}
  if(isnan(pid2_Kd)){pid2_Kd=defKd;}
  if(pid2_Kd<0){pid2_Kd=defKd;}
  
  if(isnan(pid1_a_Kp)){pid1_a_Kp=4.0;}
  if(pid1_a_Kp<0){pid1_a_Kp=4.0;}
  if(isnan(pid1_a_Ki)){pid1_a_Ki=2.0;}
  if(pid1_a_Ki<0){pid1_a_Ki=2.0;}
  if(isnan(pid1_a_Kd)){pid1_a_Kd=5.0;}
  if(pid1_a_Kd<0){pid1_a_Kd=5.0;}
  
  if(isnan(pid2_a_Kp)){pid2_a_Kp=4.0;}
  if(pid2_a_Kp<0){pid2_a_Kp=4.0;}
  if(isnan(pid2_a_Ki)){pid2_a_Ki=2.0;}
  if(pid2_a_Ki<0){pid2_a_Ki=2.0;}
  if(isnan(pid2_a_Kd)){pid2_a_Kd=5.0;}
  if(pid2_a_Kd<0){pid2_a_Kd=5.0;}
  
  if(isnan(WindowSize)){WindowSize=5000;}
  if(WindowSize<1000){WindowSize=1000;}
  if(WindowSize>10000){WindowSize=10000;}
  
  if(isnan(temp_buffer)){temp_buffer=3.0;}
  if(temp_buffer<0){temp_buffer=1.0;}
  if(temp_buffer>10){temp_buffer=10.0;}
  
  if(isnan(temp_inc)){temp_inc=0.5;}
  if(temp_inc<0.01){temp_inc=0.01;}
  if(temp_inc>1){temp_inc=1;}
  
  if(isnan(pid1HeatOnly)){pid1HeatOnly=true;}
  if(pid1HeatOnly>1){pid1HeatOnly=true;}
  if(isnan(pid1CoolOnly)){pid1CoolOnly=false;}
  if(pid1CoolOnly>1){pid1CoolOnly=false;}
  if((pid1HeatOnly)&&(pid1CoolOnly)){pid1CoolOnly=false;}
  
  if(isnan(pid2HeatOnly)){pid2HeatOnly=true;}  
  if(pid2HeatOnly>1){pid2HeatOnly=true;}
  if(isnan(pid2CoolOnly)){pid2CoolOnly=false;}
  if(pid2CoolOnly>1){pid2CoolOnly=false;}
  if((pid2HeatOnly)&&(pid2CoolOnly)){pid2CoolOnly=false;}
  
  if(isnan(pid1AS)){pid1AS=false;}
  if(pid1AS>1){pid1AS=false;}
  
  if(isnan(pid2AS)){pid2AS=false;}
  if(pid2AS>1){pid2AS=false;}
  
  if(isnan(pid1AddTune)){pid1AddTune=false;}
  if(pid1AddTune>1){pid1AddTune=false;}
  
  if(isnan(pid2AddTune)){pid2AddTune=false;}
  if(pid2AddTune>1){pid2AddTune=false;}
  
  if(isnan(isF)){isF=false;}
  if(isF>1){isF=false;}
  
  if(isnan(showTuneNote)){showTuneNote=true;}
  if(showTuneNote>1){showTuneNote=true;}
  if(isnan(showPIDTune)){showPIDTune=true;}
  if(showPIDTune>1){showPIDTune=true;}
  if(isnan(showValPer)){showValPer=false;}
  if(showValPer>1){showValPer=false;}
}
//------------------------------------------------------------------------------------------------------
// This routine is used to display the temp on the oled.
//------------------------------------------------------------------------------------------------------
void displayTemp(double tmp){
  if(tmp==-127.00){
    display.print(F("ERROR"));
  }else{
    if (isF){
      display.print(DallasTemperature::toFahrenheit(tmp));
      display.print(F(" F"));
    }else{
      display.print(tmp);
      display.print(F(" C"));
    }
  }
}
//------------------------------------------------------------------------------------------------------
// This routine is used to control the display output (GUI).
//------------------------------------------------------------------------------------------------------
void oledDisplay(){

  switch(displayState){
          //===================================
          // Monitor Screen
          //===================================
          case main:
            ssd1306_printFixed(37, 0, "MONITOR", STYLE_BOLD);
            display.setCursor(0,16);
            display.print(F("PID1 Set:  "));
            displayTemp(pid1_Setpoint);
            display.setCursor(0,24);
            display.print(F("PID1 Temp: "));
            displayTemp(pid1_Input);
            display.setCursor(0,34);
            display.setCursor(0,44);
            display.print(F("PID2 Set:  "));
            displayTemp(pid2_Setpoint);
            display.setCursor(0,54);
            display.print(F("PID2 Temp: "));
            displayTemp(pid2_Input);
          break;
          //===================================
          // PID 1 Settings
          //===================================
          case pid1s:
            placeMax=4;
            ssd1306_printFixed(24, 0, "PID 1 SETTINGS", STYLE_BOLD);
            display.setCursor(0,18);
            display.print(F("OUT:  "));
            if (PID_1.GetDirection()==DIRECT){display.print(F("HEAT "));}else{display.print(F("COOL "));}
            if(showValPer){
              display.print(map(pid1_Output,0,WindowSize,0,100));
              display.print("%");
            }else{
              display.print(pid1_Output);
            }
            display.setCursor(0,26);
            display.print(F("TEMP: "));
            displayTemp(pid1_Input);
            display.setCursor(0,32);
            display.print(F("SET:  "));
            if (placeMarker==1){ssd1306_negativeMode();}
            displayTemp(pid1_Setpoint);
            if (placeMarker==1){ssd1306_positiveMode();}
            display.setCursor(0,40);
            display.print(F("O/I:  "));
            if (placeMarker==2){ssd1306_negativeMode();}
            if (PID_1.GetMode()==AUTOMATIC){display.print(F("ON"));}else{display.print(F("OFF"));}
            //if (PID_1.GetMode()==MANUAL){display.print(F("OFF"));}
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,48);
            display.print(F("MODE: "));
            if (placeMarker==3){ssd1306_negativeMode();}
            if((!pid1HeatOnly) && (!pid1CoolOnly)){display.print(F("BOTH H&C"));}
            if((pid1HeatOnly) && (!pid1CoolOnly)){display.print(F("HEAT ONLY"));}
            if((!pid1HeatOnly) && (pid1CoolOnly)){display.print(F("COOL ONLY"));}
            if (placeMarker==3){ssd1306_positiveMode();}
            display.setCursor(0,56);
            display.print(F("AUTO START: "));
            if (placeMarker==4){ssd1306_negativeMode();}
            if(pid1AS){display.print(F("ON"));}else{display.print(F("OFF"));}
            if (placeMarker==4){ssd1306_positiveMode();}
          break;
          //===================================
          // PID 1 Tuning
          //===================================
          case pid1t:
            if (pid1AddTune){placeMax=8;}else{placeMax=5;}
            ssd1306_printFixed(30, 0, "PID 1 TUNING", STYLE_BOLD);
            display.setCursor(0,16);
            display.print(F("P on: "));
            if (placeMarker==1){ssd1306_negativeMode();}
            if(pid1PonE){display.print(F("Error"));}else{display.print(F("Measure"));}
            if (placeMarker==1){ssd1306_positiveMode();}
            display.setCursor(0,28);
            display.print(F("Adaptive Tuning: "));
            if (placeMarker==2){ssd1306_negativeMode();}
            if (pid1AddTune){display.print(F("Y"));}else{display.print(F("N"));}
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,36);
            display.print(F("Normal Tuning: "));
            display.setCursor(0,42);
            display.print(F("P="));
            if (placeMarker==3){ssd1306_negativeMode();}
            display.print(pid1_Kp);
            if (placeMarker==3){ssd1306_positiveMode();}
            display.print(F(" I="));
            if (placeMarker==4){ssd1306_negativeMode();}
            display.print(pid1_Ki);
            if (placeMarker==4){ssd1306_positiveMode();}
            display.print(F(" D="));
            if (placeMarker==5){ssd1306_negativeMode();}
            display.print(pid1_Kd);
            if (placeMarker==5){ssd1306_positiveMode();}
            if (pid1AddTune){
              display.setCursor(0,50);
              display.print(F("Aggressive Tuning: "));
              display.setCursor(0,58);
              display.print(F("P="));
              if (placeMarker==6){ssd1306_negativeMode();}
              display.print(pid1_a_Kp);
              if (placeMarker==6){ssd1306_positiveMode();}
              display.print(F(" I="));
              if (placeMarker==7){ssd1306_negativeMode();}
              display.print(pid1_a_Ki);
              if (placeMarker==7){ssd1306_positiveMode();}
              display.print(F(" D="));
              if (placeMarker==8){ssd1306_negativeMode();}
              display.print(pid1_a_Kd);
              if (placeMarker==8){ssd1306_positiveMode();}
            }
          break;
          //===================================
          // PID 2 Settings
          //===================================
          case pid2s:
            placeMax=4;
            ssd1306_printFixed(24, 0, "PID 2 SETTINGS", STYLE_BOLD);
            display.setCursor(0,18);
            display.print(F("OUT:  "));
            if (PID_2.GetDirection()==DIRECT){display.print(F("HEAT "));}else{display.print(F("COOL "));}
            //if (PID_2.GetDirection()==REVERSE){display.print(F("COOL "));}
            if(showValPer){
              display.print(map(pid2_Output,0,WindowSize,0,100));
              display.print("%");
            }else{
              display.print(pid2_Output);
            }
            display.setCursor(0,26);
            display.print(F("TEMP: "));
            displayTemp(pid2_Input);
            display.setCursor(0,32);
            display.print(F("SET:  "));
            if (placeMarker==1){ssd1306_negativeMode();}
            displayTemp(pid2_Setpoint);
            if (placeMarker==1){ssd1306_positiveMode();}
            display.setCursor(0,40);
            display.print(F("O/I:  "));
            if (placeMarker==2){ssd1306_negativeMode();}
            if (PID_2.GetMode()==AUTOMATIC){display.print(F("ON"));}else{display.print(F("OFF"));}
            //if (PID_2.GetMode()==MANUAL){display.print(F("OFF"));}
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,48);
            display.print(F("MODE: "));
            if (placeMarker==3){ssd1306_negativeMode();}
            if((!pid2HeatOnly) && (!pid2CoolOnly)){display.print(F("BOTH H&C"));}
            if((pid2HeatOnly) && (!pid2CoolOnly)){display.print(F("HEAT ONLY"));}
            if((!pid2HeatOnly) && (pid2CoolOnly)){display.print(F("COOL ONLY"));}
            if (placeMarker==3){ssd1306_positiveMode();}
            display.setCursor(0,56);
            display.print(F("AUTO START: "));
            if (placeMarker==4){ssd1306_negativeMode();}
            if(pid2AS){display.print(F("ON"));}else{display.print(F("OFF"));}
            if (placeMarker==4){ssd1306_positiveMode();}
          break;
          //===================================
          // PID 2 Tuning
          //===================================
          case pid2t:
            if (pid2AddTune){placeMax=8;}else{placeMax=5;}
            ssd1306_printFixed(30, 0, "PID 2 TUNING", STYLE_BOLD);
            display.setCursor(0,16);
            display.print(F("P on: "));
            if (placeMarker==1){ssd1306_negativeMode();}
            if(pid2PonE){display.print(F("Error"));}else{display.print(F("Measure"));}
            if (placeMarker==1){ssd1306_positiveMode();}
            display.setCursor(0,28);
            display.print(F("Adaptive Tuning: "));
            if (placeMarker==2){ssd1306_negativeMode();}
            if (pid2AddTune){display.print(F("Y"));}else{display.print(F("N"));}
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,36);
            display.print(F("Normal Tuning: "));
            display.setCursor(0,42);
            display.print(F("P="));
            if (placeMarker==3){ssd1306_negativeMode();}
            display.print(PID_2.GetKp());
            if (placeMarker==3){ssd1306_positiveMode();}
            display.print(F(" I="));
            if (placeMarker==4){ssd1306_negativeMode();}
            display.print(PID_2.GetKi());
            if (placeMarker==4){ssd1306_positiveMode();}
            display.print(F(" D="));
            if (placeMarker==5){ssd1306_negativeMode();}
            display.print(PID_2.GetKd());
            if (placeMarker==5){ssd1306_positiveMode();}
            if (pid2AddTune){
              display.setCursor(0,50);
              display.print(F("Aggressive Tuning: "));
              display.setCursor(0,58);
              display.print(F("P="));
              if (placeMarker==6){ssd1306_negativeMode();}
              display.print(pid2_a_Kp);
              if (placeMarker==6){ssd1306_positiveMode();}
              display.print(F(" I="));
              if (placeMarker==7){ssd1306_negativeMode();}
              display.print(pid2_a_Ki);
              if (placeMarker==7){ssd1306_positiveMode();}
              display.print(F(" D="));
              if (placeMarker==8){ssd1306_negativeMode();}
              display.print(pid2_a_Kd);
              if (placeMarker==8){ssd1306_positiveMode();}
            }
          break;
          //===================================
          // Global PID Settings
          //===================================
          case pidg:
            placeMax=3;
            ssd1306_printFixed(5, 0, "GLOBAL PID SETTINGS", STYLE_BOLD);
            display.setCursor(0,16);
            display.print(F("PID TIME CYCLE: "));
            if (placeMarker==1){ssd1306_negativeMode();}
            display.print(WindowSize/1000);
            display.print(F(" sec"));
            if (placeMarker==1){ssd1306_positiveMode();}                        
            display.setCursor(0,26);
            display.print(F("H/C BUFFER: "));
            if (placeMarker==2){ssd1306_negativeMode();}
            display.print(temp_buffer);
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,36);
            display.print(F("ADAPTIVE GAP: "));
            if (placeMarker==3){ssd1306_negativeMode();}
            display.print(pidGAT);
            if (placeMarker==3){ssd1306_positiveMode();}
          break;
          //===================================
          // PID Tuning Notes
          //===================================
          case pidtn:
            placeMax=3;
            ssd1306_printFixed(14, 0, "PID TUNING NOTES", STYLE_BOLD);
            display.setCursor(0,14);
            if(placeMarker==1){
              display.print(F("P: Determines how    aggressively the PID reacts to the currentamount of error      (Proportional)"));
            }
            if(placeMarker==2){
              display.print(F("I: Determines how    aggressively the PID reacts to error over time (Integral)                    "));
            }
            if(placeMarker==3){
              display.print(F("D: Determines how    aggressively the PID reacts to the change in error (Derivative)              "));
            }
            ssd1306_printFixed(2, 56, "PAGE SELECT: UP/DOWN", STYLE_BOLD);
          break;
          //===================================
          // Settings
          //===================================
          case set:
            placeMax=5;
            ssd1306_printFixed(38, 0, "SETTINGS", STYLE_BOLD);
            display.setCursor(0,16);
            display.print(F("UNIT: "));
            if (placeMarker==1){ssd1306_negativeMode();}
            if(isF){display.print(F("Fahrenheit"));}else{display.print(F("Celsius"));}
            if (placeMarker==1){ssd1306_positiveMode();}
            display.setCursor(0,24);
            display.print(F("STEP: "));
            if (placeMarker==2){ssd1306_negativeMode();}
            display.print(temp_inc);
            if (placeMarker==2){ssd1306_positiveMode();}
            display.setCursor(0,32);
            display.print(F("SHOW TUNING NOTES: "));
            if (placeMarker==3){ssd1306_negativeMode();}
            if(showTuneNote){display.print(F("Y"));}else{display.print(F("N"));}
            if (placeMarker==3){ssd1306_positiveMode();}
            display.setCursor(0,40);
            display.print(F("SHOW PID TUNING: "));
            if (placeMarker==4){ssd1306_negativeMode();}
            if(showPIDTune){display.print(F("Y"));}else{display.print(F("N"));}
            if (placeMarker==4){ssd1306_positiveMode();}
            display.setCursor(0,48);
            display.print(F("OUTPUT AS: "));
            if (placeMarker==5){ssd1306_negativeMode();}
            if(showValPer){display.print(F("PERCENT"));}else{display.print(F("VALUE"));}
            if (placeMarker==5){ssd1306_positiveMode();}
          break;
        }
}
//------------------------------------------------------------------------------------------------------
// This routine is used to determin what happens when the user pushes a button
//------------------------------------------------------------------------------------------------------
void button_handler()
{
    int button=read_buttons();

    if(button!=btnNONE) 
    {
      
      last_press=millis();
      
      //===================================
      // Select Button
      //===================================
      if(button==btnSELECT){
        display.clear();
        placeMarker=1;
        switch(displayState){
          case main:
            displayState=pid1s;
          break;
          case pid1s:
            if(showPIDTune){
              displayState=pid1t;
            }else{
              displayState=pid2s;
            }
            SAVE_SETTINGS();
          break;
          case pid1t:
            displayState=pid2s;
            SAVE_SETTINGS();
          break;
          case pid2s:
            if(showPIDTune){
              displayState=pid2t;
            }else{
              displayState=pidg;
            }
            SAVE_SETTINGS();
          break;
          case pid2t:
            displayState=pidg;
            SAVE_SETTINGS();
          break;
          case pidg:
            if(showTuneNote){
              displayState=pidtn;
            }else{
              displayState=set;
            }
            SAVE_SETTINGS();
          break;
          case pidtn:
            displayState=set;
          break;
          case set:
            displayState=main;
            SAVE_SETTINGS();
          break;
        }
      }
      //===================================
      // Down Button
      //===================================
      if(button==btnDOWN){
        switch(displayState){
          case main:
            pid1_Setpoint -= temp_inc;
            if (pid1_Setpoint<tempMin) {pid1_Setpoint=tempMin;}
          break;
          default:
            placeMarker+=1;
            if (placeMarker>placeMax){placeMarker=1;}
          break;
        }
      } 
      //===================================
      // Up Button
      //===================================
      if(button==btnUP){
        switch(displayState){
          case main:
            pid1_Setpoint += temp_inc;
            if (pid1_Setpoint>tempMax) {pid1_Setpoint=tempMax;}
          break;
          default:
            placeMarker-=1;
            if (placeMarker<1){placeMarker=placeMax;}
          break; 
        }
      }
      //===================================
      // Left Button
      //===================================
      if(button==btnLEFT){
        display.clear();
        switch(displayState){
          case main:
            pid2_Setpoint -= temp_inc;
            if (pid2_Setpoint<tempMin) {pid2_Setpoint=tempMin;}
          break;
          case pid1s:
            if(placeMarker==1){
              pid1_Setpoint -= temp_inc;
              if (pid1_Setpoint<tempMin) {pid1_Setpoint=tempMin;}
            }
            if(placeMarker==2){
              if (PID_1.GetMode()==AUTOMATIC){
                PID_1.SetMode(MANUAL);
              }else{
                PID_1.SetMode(AUTOMATIC);
              }
            }
            if(placeMarker==3){
              if(pid1HeatOnly){
                pid1HeatOnly=false;
              }else{
                pid1HeatOnly=true;
                pid1CoolOnly=false;
              }
            }
            if (placeMarker==4){
              if(pid1AS){pid1AS=false;}else{pid1AS=true;}
            }
          break;
          case pid1t:
            if(placeMarker==1){
              if(pid1PonE){pid1PonE=false;}else{pid1PonE=true;}
            }
            if(placeMarker==2){
              if(pid1AddTune){pid1AddTune=false;}else{pid1AddTune=true;}
              setPIDTuning();
            }
            if(placeMarker==3){
              pid1_Kp-=0.1;
              if(pid1_Kp<0){pid1_Kp=0;}
              setPIDTuning();
            }
            if(placeMarker==4){
              pid1_Ki-=0.1;
              if(pid1_Ki<0){pid1_Ki=0;}
              setPIDTuning();
            }
            if(placeMarker==5){
              pid1_Kd-=0.1;
              if(pid1_Kd<0){pid1_Kd=0;}
              setPIDTuning();
            }
            if(placeMarker==6){pid1_a_Kp-=0.1;if(pid1_a_Kp<0){pid1_a_Kp=0;}}
            if(placeMarker==7){pid1_a_Ki-=0.1;if(pid1_a_Ki<0){pid1_a_Ki=0;}}
            if(placeMarker==8){pid1_a_Kd-=0.1;if(pid1_a_Kd<0){pid1_a_Kd=0;}}
          break;
          case pid2s:
            if(placeMarker==1){
              pid2_Setpoint -= temp_inc;
              if (pid2_Setpoint<tempMin) {pid2_Setpoint=tempMin;}
            }
            if(placeMarker==2){
              if (PID_2.GetMode()==AUTOMATIC){
                PID_2.SetMode(MANUAL);
              }else{
                PID_2.SetMode(AUTOMATIC);
              }
            }
            if(placeMarker==3){
              if(pid2HeatOnly){
                pid2HeatOnly=false;
              }else{
                pid2HeatOnly=true;
                pid2CoolOnly=false;
              }
            }
            if (placeMarker==4){
              if(pid2AS){pid2AS=false;}else{pid2AS=true;}
            }
          break;
          case pid2t:
            if(placeMarker==1){
              if(pid2PonE){pid2PonE=false;}else{pid2PonE=true;}
              setPIDTuning();
            }
            if(placeMarker==2){
              if(pid2AddTune){pid2AddTune=false;}else{pid2AddTune=true;}
              setPIDTuning();
            }
            if(placeMarker==3){
              pid2_Kp-=0.1;
              if(pid2_Kp<0){pid2_Kp=0;}
              setPIDTuning();
            }
            if(placeMarker==4){
              pid2_Ki-=0.1;
              if(pid2_Ki<0){pid2_Ki=0;}
              setPIDTuning();
            }
            if(placeMarker==5){
              pid2_Kd-=0.1;
              if(pid2_Kd<0){pid2_Kd=0;}
              setPIDTuning();
            }
            if(placeMarker==6){pid2_a_Kp-=0.1;if(pid2_a_Kp<0){pid2_a_Kp=0;}}
            if(placeMarker==7){pid2_a_Ki-=0.1;if(pid2_a_Ki<0){pid2_a_Ki=0;}}
            if(placeMarker==8){pid2_a_Kd-=0.1;if(pid2_a_Kd<0){pid2_a_Kd=0;}}
          break;
          case pidg:
            if(placeMarker==1){
              WindowSize-=1000;
              if(WindowSize<1000){WindowSize=1000;}
              setPIDLimit();
            }
            if(placeMarker==2){
              temp_buffer-=temp_inc;
              if(temp_buffer<0.01){temp_buffer=0.01;}
            }
            if(placeMarker==3){
              pidGAT-=temp_inc;
              if(pidGAT<1){temp_buffer=1;}
            }
          break;
          case set:
            if(placeMarker==1){
              if(isF){isF=false;}else{isF=true;}
            }
            if(placeMarker==2){
              temp_inc-=0.01;
              if(temp_inc<0.01){temp_inc=0.01;}
            }
            if(placeMarker==3){
              if(showTuneNote){showTuneNote=false;}else{showTuneNote=true;}
            }
            if(placeMarker==4){
              if(showPIDTune){showPIDTune=false;}else{showPIDTune=true;}
            }
            if(placeMarker==5){
              if(showValPer){showValPer=false;}else{showValPer=true;}
            }
          break;
        }
        
      }
      //===================================
      // Right Button
      //===================================
      if(button==btnRIGHT){
        display.clear();
        switch(displayState){
          case main:
            pid2_Setpoint += temp_inc;
            if (pid2_Setpoint>tempMax) {pid2_Setpoint=tempMax;}
          break;
          case pid1s:
            if(placeMarker==1){
              pid1_Setpoint += temp_inc;
              if (pid1_Setpoint>tempMax) {pid1_Setpoint=tempMax;}
            }
            if(placeMarker==2){
              if (PID_1.GetMode()==AUTOMATIC){
                PID_1.SetMode(MANUAL);
              }else{
                PID_1.SetMode(AUTOMATIC);
              }
            }
            if(placeMarker==3){
              if(pid1CoolOnly){
                pid1CoolOnly=false;
              }else{
                pid1CoolOnly=true;
                pid1HeatOnly=false;
              }
            }
            if (placeMarker==4){
              if(pid1AS){pid1AS=false;}else{pid1AS=true;}
            }
          break;
          case pid1t:
            if(placeMarker==1){
              if(pid1PonE){pid1PonE=false;}else{pid1PonE=true;}
              setPIDTuning();
            }
            if(placeMarker==2){
              if(pid1AddTune){pid1AddTune=false;}else{pid1AddTune=true;}
              setPIDTuning();
            }
            if(placeMarker==3){pid1_Kp+=0.1;setPIDTuning();}
            if(placeMarker==4){pid1_Ki+=0.1;setPIDTuning();}
            if(placeMarker==5){pid1_Kd+=0.1;setPIDTuning();}
            
            if(placeMarker==6){pid1_a_Kp+=0.1;}
            if(placeMarker==7){pid1_a_Ki+=0.1;}
            if(placeMarker==8){pid1_a_Kd+=0.1;}
            
          break;
          case pid2s:
            if(placeMarker==1){
              pid2_Setpoint += temp_inc;
              if (pid2_Setpoint>tempMax) {pid2_Setpoint=tempMax;}
            }
            if(placeMarker==2){
              if (PID_2.GetMode()==AUTOMATIC){
                PID_2.SetMode(MANUAL);
              }else{
                PID_2.SetMode(AUTOMATIC);
              }
            }
            if(placeMarker==3){
              if(pid2CoolOnly){
                pid2CoolOnly=false;
              }else{
                pid2CoolOnly=true;
                pid2HeatOnly=false;
              }
            }
            if (placeMarker==4){
              if(pid2AS){pid2AS=false;}else{pid2AS=true;}
            }
          break;
          case pid2t:
            if(placeMarker==1){
              if(pid2PonE){pid2PonE=false;}else{pid2PonE=true;}
              setPIDTuning();
            }
            if(placeMarker==2){
              if(pid2AddTune){pid2AddTune=false;}else{pid2AddTune=true;}
              setPIDTuning();
            }
            if(placeMarker==3){pid2_Kp+=0.1;setPIDTuning();}
            if(placeMarker==4){pid2_Ki+=0.1;setPIDTuning();}
            if(placeMarker==5){pid2_Kd+=0.1;setPIDTuning();}
            
            if(placeMarker==6){pid2_a_Kp+=0.1;}
            if(placeMarker==7){pid2_a_Ki+=0.1;}
            if(placeMarker==8){pid2_a_Kd+=0.1;}
            
          break;
          case pidg:
            if(placeMarker==1){
              WindowSize+=1000;
              if(WindowSize>10000){WindowSize=10000;}
              setPIDLimit();
            }
            if(placeMarker==2){
              temp_buffer+=temp_inc;
              if(temp_buffer>10.00){temp_buffer=10.00;}
            }
            if(placeMarker==3){
              pidGAT+=temp_inc;
              if(pidGAT>10){temp_buffer=10;}
            }
          break;
          case set:
            if(placeMarker==1){
              if(isF){isF=false;}else{isF=true;}
            }
            if(placeMarker==2){
              temp_inc+=0.01;
              if(temp_inc>1.00){temp_inc=1.00;}
            }
            if(placeMarker==3){
              if(showTuneNote){showTuneNote=false;}else{showTuneNote=true;}
            }
            if(placeMarker==4){
              if(showPIDTune){showPIDTune=false;}else{showPIDTune=true;}
            }
            if(placeMarker==5){
              if(showValPer){showValPer=false;}else{showValPer=true;}
            }
          break;
        }
        
      }
    }    
}
//------------------------------------------------------------------------------------------------------
// This routine is used to determin which button has been pressed.
//------------------------------------------------------------------------------------------------------
int read_buttons() 
{
   // Read The Button Array And Return The Users Input If Any
   
   int button = analogRead(BUTTON_PIN); // Read Button Array
       
   // Approx button values are 0, 76, 191, 473, 676 (1023)
   // Add A Little Extra To Those Values To Be On The Safe Side
   // Check And Retrun Value
   if (button > 1000) return btnNONE; // No button is pressed
   if (button < 50)   return btnSELECT;  // Right pressed
   if (button < 125)  return btnDOWN;  // Up presed
   if (button < 275)  return btnUP;  // Down pressed
   if (button < 525)  return btnLEFT;   // Left pressed
   if (button < 750)  return btnRIGHT;  // Select pressed 
   return btnNONE;  // If no valid response return No button pressed
}
