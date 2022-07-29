//A Process to activate catalyst in system and remove impurities.
//Consists of injecting Hydrogen and Nitrogen
//into the system and raising the temperature over therecourse of several the course

#include <Wire.h> //For I2C comm
#include <Adafruit_GFX.h>
#include <LedHelper.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_LiquidCrystal.h"
#include<SPI.h>//serial peripheral interface
#include<Ethernet3.h>//for ethernet
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>//for modbus tcp/ip
#include<ModbusMaster.h>//for oci417.10 rs485 interface with BMM

#include<pid.h>
#include "DEFINES.h"
#include "GLOBALS.h"

int SR_TUBE_AVG_TEMP;

// OCI417.10 rs485 interface with BMM
ModbusMaster NODE;

//modbus tcp/ip communication declarations
byte mac[] = { 0xdE, 0xAD, 0xBE, 0xEF, 0xED};

IPAddress ip(128, 1, 1, 110); //needs to be similar to Acromag because will not connect

EthernetClient ethClient1;
EthernetClient ethClient2;
EthernetClient ethClient3;
EthernetClient ethClient4;
EthernetClient ethClient5;
EthernetClient ethClient6;
EthernetClient ethClient7;
EthernetClient ethClient8;

ModbusTCPClient modbusTCPClient1(ethClient1);
ModbusTCPClient modbusTCPClient2(ethClient2);
ModbusTCPClient modbusTCPClient3(ethClient3);
ModbusTCPClient modbusTCPClient4(ethClient4);
ModbusTCPClient modbusTCPClient5(ethClient5);
ModbusTCPClient modbusTCPClient6(ethClient6);
ModbusTCPClient modbusTCPClient7(ethClient7);
ModbusTCPClient modbusTCPClient8(ethClient8);

IPAddress serverIOEX1(128, 1, 1, 101); // IP of ioex1 STRIDE SIO-MB04DAS 4CH Analog Output
IPAddress serverIOEX2(128, 1, 1, 102); // IP of ioex2 STRIDE SIO-MB04DAS 4CH Analog Output
IPAddress serverIOEX3(128, 1, 1, 103); // IP of ioex3 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX4(128, 1, 1, 104); // IP of ioex4 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX5(128, 1, 1, 105); // IP of ioex5 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX6(128, 1, 1, 106); // IP of ioex6 MOXA IOLOGIK E1240 8CH Analog Input
IPAddress serverIOEX7(128, 1, 1, 107); // IP of ioex7 MOXA IOLOGIK E1211 16CH Digital Output
IPAddress serverIOEX8(128, 1, 1, 108); // IP of ioex8 MOXA IOLOGIK E1210 16CH Digital Input

//Daughter Board periferals
SmallMatrix smallMatrix[3] = {SmallMatrix(0x70), SmallMatrix(0x71), SmallMatrix(0x72) };
LargeMatrix bigMatrix[3] = {LargeMatrix(0x73), LargeMatrix(0x74), LargeMatrix(0x75) };
Adafruit_LiquidCrystal lcd(0);

//telemetry
union floatToBytes  {
  char  asBytes[4] = {0};
  float asFloat;
};

void setup() {

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode rs485
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600); // for dedbugging

  Serial1.begin(9600);//FOR OCI MODULE rs485
  NODE.begin(1, Serial1); //unit id1 on serial1
  NODE.preTransmission(preTransmission);
  NODE.postTransmission(postTransmission);

  delay(500);//wait for system to boot up

  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
  pinMode(TRACO_24VDC, OUTPUT);
  digitalWrite(TRACO_24VDC, HIGH);
  pinMode(ESTOP_BREAK, OUTPUT);
  digitalWrite(ESTOP_BREAK, HIGH);

  //reseting wifichip
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(50);

  //start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  integrityCheck();
  connect_IO_Expanders();
  readBtn();
  readOCI();

  CURRENT_MILLIS = millis(); //So as to not divide by zero on first looptime.
  LOOP_MILLIS = millis();

  lcd.begin(16, 2);

  SENSOR_INTEGRITY_CHECK = true;

}

void loop() {

  LOOP_TIME = LOOP_MILLIS - CURRENT_MILLIS;
  CURRENT_MILLIS = millis();

  SR_TUBE_AVG_TEMP = ( TT444_SMR_TUBE4_OUT + TT445_SMR_TUBE5_OUT + TT446_SMR_TUBE6_OUT + TT447_SMR_TUBE7_OUT
                       + TT448_SMR_TUBE8_OUT + TT449_SMR_TUBE9_OUT);

  SR_TUBE_AVG_TEMP /= 6;
  Serial.print("FSM_STATE : "); Serial.println(FSM_STATE);
  Serial.print("PREVIOUS_FSM_STATE: "); Serial.println(PREVIOUS_FSM_STATE);
  Serial.print("SR_TUBE_AVG_TEMP: "); Serial.println(SR_TUBE_AVG_TEMP);
  Serial.print("Timing: "); Serial.println(CURRENT_MILLIS - PREVIOUS_MILLIS_1);

  error_Checker();
  connect_IO_Expanders();
  integrityCheck();
  readTCs();
  readBtn();
  readOCI();
  readPTs();
  blinkGRN();
  blinkAMB();
  readOut();
  readOCI();

  modbusTCPClient7.coilWrite(9, 1); //BLWR_EN ON bc opposite
  modbusTCPClient7.coilWrite(7, 1); //xv909
  modbusTCPClient7.coilWrite(4, 1); //XV1100

  GRN_BTN_FLAG = true;

  if (GRN_BTN_FLAG && !ESTOP_FLAG) {

    switch (FSM_STATE) {

      case INITIALIZE: //initialize 0
        if (!COMBUSTION_PRESSURE_SWITCH) {

          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient7.coilWrite(9, 1); //BLWR_EN..The logic is opposite
          break;
        }

        if (DUN_PSL) {
          break;
        }

        modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
        modbusTCPClient7.coilWrite(9, 0); //BLWR_EN..


        if (BMM_PROOF_OF_FLAME) { //get that flame out before proceed

          if (!modbusTCPClient1.connected()) {
            modbusTCPClient1.begin(serverIOEX1, 502);
          }
          if (!modbusTCPClient2.connected()) {
            modbusTCPClient2.begin(serverIOEX2, 502);
          }

          modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 ioex1 (FCV134)
          modbusTCPClient2.holdingRegisterWrite(40, OFF); //write channel 0 ioex2 (FCV141)

          if (!modbusTCPClient7.connected()) {
            modbusTCPClient7.begin(serverIOEX7, 502);
          }
          modbusTCPClient7.coilWrite(6, OFF); //TWV308
          // modbusTCPClient7.coilWrite(4, OFF); //XV1100
          modbusTCPClient7.coilWrite(3, OFF); //XV501
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
          modbusTCPClient7.coilWrite(7, OFF); //TWV901
          break;
        }
        FSM_STATE = BMM_OFF;
        modbusTCPClient1.holdingRegisterWrite(42, FCV205_AT_35_PERCENT); //write channel 2 (FCV205)//made steam at 1000//last 3000//~about 35 percent
        modbusTCPClient1.holdingRegisterWrite(41, 2000); //write channel 1 (WP_Speed) at 50 percent
        modbusTCPClient7.coilWrite(10, ON); //WP_EN..
        modbusTCPClient7.coilWrite(4, ON); //XV1100
        modbusTCPClient7.coilWrite(7, 1); //xv909
        PREVIOUS_MILLIS = millis();
        break;

      case BMM_OFF://bmm off //30 seconds; //1

        modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
        modbusTCPClient1.holdingRegisterWrite(41, 2000); //write channel 1 (WP_Speed) at 50 percent

        if (PREVIOUS_FSM_STATE == BURNER_HOLD_100C && SR_TUBE_AVG_TEMP > 70 ) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)

          if (SR_TUBE_AVG_TEMP > 80 && SR_TUBE_AVG_TEMP < 130) {
            if (initTimer == false) {
              PREVIOUS_MILLIS_1 = millis();
              initTimer = true;
              break;
            }
            if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_100C_TIMER && initTimer == true) {
              FSM_STATE = BURNER_HOLD_250C; PREVIOUS_FSM_STATE = BURNER_HOLD_250C; initTimer = false; break;
            }
          }
          else {
            initTimer = false;
          }
          break;
        }
        if (PREVIOUS_FSM_STATE == BURNER_HOLD_250C && SR_TUBE_AVG_TEMP > 230 ) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)

          if (SR_TUBE_AVG_TEMP > 220 && SR_TUBE_AVG_TEMP < 270) {
            if (initTimer == false) {
              PREVIOUS_MILLIS_1 = millis();
              initTimer = true;
              break;
            }
            if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_350C_TIMER && initTimer == true) {
              FSM_STATE = BURNER_HOLD_350C; PREVIOUS_FSM_STATE = BURNER_HOLD_350C; initTimer = false; break;
            }
          }
          else {
            initTimer = false;
          }
          break;
        }
        if (PREVIOUS_FSM_STATE == BURNER_HOLD_350C && SR_TUBE_AVG_TEMP > 330 ) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)

          if (SR_TUBE_AVG_TEMP > 320 && SR_TUBE_AVG_TEMP < 370) {
            if (initTimer == false) {
              PREVIOUS_MILLIS_1 = millis();
              initTimer = true;
              break;
            }
            if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_350C_TIMER && initTimer == true) {
              FSM_STATE = BURNER_HOLD_450C; PREVIOUS_FSM_STATE = BURNER_HOLD_450C; initTimer = false; break;
            }
          }
          else {
            initTimer = false;
          }
          break;
        }
        if (PREVIOUS_FSM_STATE == BURNER_HOLD_450C && SR_TUBE_AVG_TEMP > 430 ) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)

          if (SR_TUBE_AVG_TEMP > 420 && SR_TUBE_AVG_TEMP < 470) {
            if (initTimer == false) {
              PREVIOUS_MILLIS_1 = millis();
              initTimer = true;
              break;
            }
            if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_450C_TIMER && initTimer == true) {
              FSM_STATE = BURNER_HOLD_600C; PREVIOUS_FSM_STATE = BURNER_HOLD_600C; initTimer = false; break;
            }
          }
          else {
            initTimer = false;
          }
          break;
        }
        if (PREVIOUS_FSM_STATE == BURNER_HOLD_600C && SR_TUBE_AVG_TEMP > 580 ) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)


          if (SR_TUBE_AVG_TEMP > 570 && SR_TUBE_AVG_TEMP < 620) {
            if (initTimer == false) {
              PREVIOUS_MILLIS_1 = millis();
              initTimer = true;
              break;
            }
            if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_600C_TIMER && initTimer == true) {
              FSM_STATE = FINISH; initTimer = false; break;
            }
          }
          else {
            initTimer = false;
          }
          break;
        }


        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_OFF_TIMER) {

          modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2 bmm off
          break;
        }
        FSM_STATE = BMM_ON;
        PREVIOUS_MILLIS = millis();
        break;

      case BMM_ON: //2
        //bmm on //5 seconds
        modbusTCPClient7.coilWrite(8, ON); //BMM_CR2 turn on
        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_ON_TIMER) {
          break;
        }
        PREVIOUS_MILLIS = millis();
        FSM_STATE = BMM_PURGE;
        break;

      case BMM_PURGE://3
        //bmm purge

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_PURGE_TIMER) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
          break;
        }
        PREVIOUS_MILLIS = millis();
        FSM_STATE = BMM_IGNITION;
        modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
        break;

      case BMM_IGNITION: //4
        //BMM ignition
        modbusTCPClient1.holdingRegisterWrite(40, BLOWER_IGNITION_SPEED); //write channel 0 (BLWRSpeed)


        if (CURRENT_MILLIS - PREVIOUS_MILLIS >= BMM_IGNITION_TIMER) {

          if (!BMM_PROOF_OF_FLAME) {
            ERROR = 8;
            FSM_STATE = BMM_OFF;
            break;
          }
          else {
            if (PREVIOUS_FSM_STATE != 0) {
              FSM_STATE = PREVIOUS_FSM_STATE;
              break;
            }
            else {
              // FSM_STATE = BURNER_HOLD_100C;//changed this to skip to different steps
              //  FSM_STATE = BURNER_HOLD_250C;//changed this to skip to different steps
              // FSM_STATE = BURNER_HOLD_350C;//changed this to skip to different steps
              // FSM_STATE = BURNER_HOLD_450C;//changed this to skip to different steps
                FSM_STATE = BURNER_HOLD_600C;//changed this to skip to different steps
             //   FSM_STATE = FINISH;//changed this to skip to different steps
                modbusTCPClient1.holdingRegisterWrite(41, 5000); //write channel 1 (WP_Speed) at 50 percent
              break;
            }

            break;
          }
        }

        break;

      case BURNER_HOLD_100C: //5


        if (SR_TUBE_AVG_TEMP > 80 && SR_TUBE_AVG_TEMP < 130) {


          if (initTimer == false) {
            PREVIOUS_MILLIS_1 = millis();
            initTimer = true;
            break;
          }
          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_100C_TIMER && initTimer == true) {

            FSM_STATE = BURNER_HOLD_250C; initTimer = false;
          }
        }
        else {
          initTimer = false;
        }

        if (SR_TUBE_AVG_TEMP < 70) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_IGNITION_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          break;
        }

        if (SR_TUBE_AVG_TEMP > 70) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          FSM_STATE = BMM_OFF;
          break;
        }

        break;

      case BURNER_HOLD_250C: //6


        if (SR_TUBE_AVG_TEMP > 230 && SR_TUBE_AVG_TEMP < 270) {
          if (initTimer == false) {
            PREVIOUS_MILLIS_1 = millis();
            initTimer = true;
            break;
          }
          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_350C_TIMER && initTimer == true) {
            FSM_STATE = BURNER_HOLD_350C; initTimer = false;
          }
        }
        else {
          initTimer = false;
        }

        if (SR_TUBE_AVG_TEMP < 240) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_IGNITION_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          break;
        }

        if (SR_TUBE_AVG_TEMP > 240) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          FSM_STATE = BMM_OFF;
          break;
        }

        break;

      case BURNER_HOLD_350C:  //7

        if (SR_TUBE_AVG_TEMP > 330 && SR_TUBE_AVG_TEMP < 370) {
          if (initTimer == false) {
            PREVIOUS_MILLIS_1 = millis();
            initTimer = true;
            break;
          }
          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_350C_TIMER && initTimer == true) {
            FSM_STATE = BURNER_HOLD_450C; initTimer = false;
          }
        }
        else {
          initTimer = false;
        }

        if (SR_TUBE_AVG_TEMP < 350) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_IGNITION_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          break;
        }

        if (SR_TUBE_AVG_TEMP > 350) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          FSM_STATE = BMM_OFF;
          break;
        }

        break;

      case BURNER_HOLD_450C: //8


        if (SR_TUBE_AVG_TEMP > 420 && SR_TUBE_AVG_TEMP < 480) {
          if (initTimer == false) {
            PREVIOUS_MILLIS_1 = millis();
            initTimer = true;
            break;
          }
          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_450C_TIMER && initTimer == true) {
            FSM_STATE = BURNER_HOLD_600C; initTimer = false;
          }
        }
        else {
          initTimer = false;
        }

        if (SR_TUBE_AVG_TEMP < 430) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_RAMP_END); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          break;
        }

        if (SR_TUBE_AVG_TEMP > 430) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          FSM_STATE = BMM_OFF;
          break;
        }

        break;

      case BURNER_HOLD_600C: //9


        if (SR_TUBE_AVG_TEMP > 570 && SR_TUBE_AVG_TEMP < 620) {
          if (initTimer == false) {
            PREVIOUS_MILLIS_1 = millis();
            initTimer = true;
            break;
          }
          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= BURNER_HOLD_600C_TIMER && initTimer == true) {
            FSM_STATE = FINISH; initTimer = false;
          }
        }
        else {
          initTimer = false;
        }

        if (SR_TUBE_AVG_TEMP < 580) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_RAMP_END); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          break;
        }

        if (SR_TUBE_AVG_TEMP > 580) {
          modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
          PREVIOUS_FSM_STATE = FSM_STATE;
          FSM_STATE = BMM_OFF;
          break;
        }

        break;

      case FINISH://10
        //should probably cooldown procedure???
        if (SR_TUBE_AVG_TEMP > 100) {
          modbusTCPClient1.holdingRegisterWrite(40, 2000); //wriannel 0 (BLWRSpeed)

        }
        modbusTCPClient1.holdingRegisterWrite(42, 0); //write channel 2 (FCV134)
        break;

    }//end of FSM_STATE
  }//end of ESTOP
}//end of loop()

void connect_IO_Expanders() {

  //CONNECT TO ACROMAGS
  if (!modbusTCPClient1.connected()) {
    modbusTCPClient1.begin(serverIOEX1, 502);
  }
  if (!modbusTCPClient2.connected()) {
    modbusTCPClient2.begin(serverIOEX2, 502);
  }
  if (!modbusTCPClient3.connected()) {
    modbusTCPClient3.begin(serverIOEX3, 502);
  }
  if (!modbusTCPClient4.connected()) {
    modbusTCPClient4.begin(serverIOEX4, 502);
  }
  if (!modbusTCPClient5.connected()) {
    modbusTCPClient5.begin(serverIOEX5, 502);
  }
  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  if (!modbusTCPClient7.connected()) {
    modbusTCPClient7.begin(serverIOEX7, 502);
  }
  if (!modbusTCPClient8.connected()) {
    modbusTCPClient8.begin(serverIOEX8, 502);
  }
}
void readPTs() {

  Serial.println("Reading PTs");
  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  //checking PTs
  modbusTCPClient6.begin(serverIOEX6, 502);
  PT318_HX406_OUTPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(3)*.0004; //read Channel 3 PressureTransducer_318.. 1psi/80counts maximux
  PT213_RO_PRESSURE = modbusTCPClient6.inputRegisterRead(4) * (.0004); // read Channel 4 PT213
  PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient6.inputRegisterRead(5) * (.0004); // read Channel 5 PT420
  PT304_TWV308_INPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(6) * (.0004); // read Channel 6 PT304

  Serial.print("PT318: "); Serial.println(PT318_HX406_OUTPUT_PRESSURE);
  Serial.print("PT213: "); Serial.println(PT213_RO_PRESSURE);
  Serial.print("PT420: "); Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
  Serial.print("PT304: "); Serial.println(PT304_TWV308_INPUT_PRESSURE);

  if (PT318_HX406_OUTPUT_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 3;
    Serial.print("PT318 >240PSI: ");
    Serial.println(PT318_HX406_OUTPUT_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if ( PT213_RO_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 4;
    Serial.print("PT213 >240PSI: ");
    Serial.println(PT213_RO_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if (PT420_STEAM_EJECTOR_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 5;
    Serial.print("PT420 >240PSI: ");
    Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if (PT304_TWV308_INPUT_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 6;
    Serial.print("PT305 >240PSI: ");
    Serial.println(PT304_TWV308_INPUT_PRESSURE);
  }
  else {
    ERROR = 0;
  }

}
void readTCs() {

  if (!modbusTCPClient3.connected()) {
    modbusTCPClient3.begin(serverIOEX3, 502);
  }
  if (!modbusTCPClient4.connected()) {
    modbusTCPClient4.begin(serverIOEX4, 502);
  }
  if (!modbusTCPClient5.connected()) {
    modbusTCPClient5.begin(serverIOEX5, 502);
  }

  //read acromag 4

  switch (TC_CHECK_COUNTER) {
    case 1:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000) {

        TT142_SR_FUEL = modbusTCPClient3.holdingRegisterRead(40) * (0.095);
        TT301_HX406_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(41) * (0.095);
        TT303_HX504_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(42) * (0.095);
        TT306_EJECTOR_STEAM_IN = modbusTCPClient3.holdingRegisterRead(43) * (0.095);
        TT313_HX402_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(44) * (0.095);
        TT319_HX402_STEAM_SYSTEM = modbusTCPClient3.holdingRegisterRead(45) * (0.095);
       // TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
        TT408_HTS_IN_LREF = modbusTCPClient3.holdingRegisterRead(47) * (0.095);

        if (TT301_HX406_STEAM_OUT > 1000) {
          Serial.println("TT301_HX406_STEAM_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 13;
        }
        if (TT303_HX504_STEAM_OUT > 1000) {
          Serial.println("TT303_HX504_STEAM_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 14;
        }
        if (TT306_EJECTOR_STEAM_IN > 1000) {
          Serial.println("TT306_EJECTOR_STEAM_IN > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 15;
        }
        if (TT313_HX402_STEAM_OUT > 1000) {
          Serial.println("TT313_HX402_STEAM_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 16;
        }
        if (TT319_HX402_STEAM_SYSTEM > 1000) {
          Serial.println("TT319_HX402_STEAM_SYSTEM > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 17;
        }
        if (TT407_STEAM_REFORMER_OUT_LREF > 1000) {
          Serial.println("TT407_STEAM_REFORMER_OUT_LREF > 1000C");
       //   FSM_STATE = INITIALIZE;
       //   ERROR = 18;
        }
        if (TT408_HTS_IN_LREF > 1000) {
          Serial.println("TT408_HTS_IN_LREF > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 19;
        }


         // Serial.print("TT142_SR_FUEL: ");Serial.println(TT142_SR_FUEL);
        //  Serial.print("TT301_HX406_STEAM_OUT : ");Serial.println(TT301_HX406_STEAM_OUT);
         // Serial.print("TT303_HX504_STEAM_OUT : ");Serial.println(TT303_HX504_STEAM_OUT);
        //  Serial.print("TT306_EJECTOR_STEAM_IN : ");Serial.println(TT306_EJECTOR_STEAM_IN);
        //  Serial.print("TT313_HX402_STEAM_OUT : ");Serial.println(TT313_HX402_STEAM_OUT);
        //  Serial.print("TT319_HX402_STEAM_SYSTEM : ");Serial.println(TT319_HX402_STEAM_SYSTEM);
          Serial.print("TT407_STEAM_REFORMER_OUT_LREF : ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF);
         // Serial.print("TT408_HTS_IN_LREF : ");Serial.println(TT408_HTS_IN_LREF);


        PREVIOUS_MILLIS_3 = millis();
        TC_CHECK_COUNTER = 2;
        break;
      }
    case 2:
      //read acromag5
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000) {

        TT410_HTS_OUT_LREF = modbusTCPClient4.holdingRegisterRead(40) * (0.095);
        TT411_FPZ_OUT_LREF = modbusTCPClient4.holdingRegisterRead(41) * (0.095);
        TT430_SMR_TUBES_INLET = modbusTCPClient4.holdingRegisterRead(42) * (0.095);
        TT511_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(43) * (0.095);
        TT512_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(44) * (0.095);
        TT513_HX504_IN = modbusTCPClient4.holdingRegisterRead(45) * (0.095);
        TT514_HX504_OUT = modbusTCPClient4.holdingRegisterRead(46) * (0.095);
        TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(47) * (0.095);

        //     if (TT410_HTS_OUT_LREF > 1000) {
        //     Serial.println("TT410_HTS_OUT_LREF > 1000C");
        //   FSM_STATE = INITIALIZE;
        // ERROR = 20;
        //}
        if (TT411_FPZ_OUT_LREF > 1000) {
          Serial.println("TT411_FPZ_OUT_LREF > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 21;
        }
        if (TT430_SMR_TUBES_INLET > 1000) {
          Serial.println("TT430_SMR_TUBES_INLET > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 22;
        }
        if (TT511_SILICON_CARBIDE_OUT > 1000) {
          Serial.println("TT511_SILICON_CARBIDE_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 23;
        }
        //     if (TT512_SILICON_CARBIDE_OUT > 1000) {
        //      Serial.println("TT512_SILICON_CARBIDE_OUT > 1000C");
        //    FSM_STATE = INITIALIZE;
        //  ERROR = 24;
        //}
        if (TT513_HX504_IN > 1000) {
          Serial.println("TT513_HX504_IN > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 25;
        }
        if (TT514_HX504_OUT > 1000) {
          Serial.println("TT514_HX504_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 26;
        }
        if (TT441_SMR_TUBE1_OUT > 800) {
          Serial.println("TT441_SMR_TUBE1_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 27;
        }

        /*
          Serial.print("TT410_HTS_OUT_LREF : ");Serial.println(TT410_HTS_OUT_LREF);
          Serial.print("TT411_FPZ_OUT_LREF : ");Serial.println(TT411_FPZ_OUT_LREF);
          Serial.print("TT430_SMR_TUBES_INLET : ");Serial.println(TT430_SMR_TUBES_INLET);
          Serial.print("TT511_SILICON_CARBIDE_OUT : ");Serial.println(TT511_SILICON_CARBIDE_OUT);
          Serial.print("TT512_SILICON_CARBIDE_OUT : ");Serial.println(TT512_SILICON_CARBIDE_OUT);
          Serial.print("TT513_HX504_IN : ");Serial.println(TT513_HX504_IN);
          Serial.print("TT514_HX504_OUT : ");Serial.println(TT514_HX504_OUT);
          Serial.print("TT441_SMR_TUBE1_OUT : ");Serial.println(TT441_SMR_TUBE1_OUT);
        */

        TC_CHECK_COUNTER = 3;
        PREVIOUS_MILLIS_3 = millis();
        break;
      }
    case 3:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000) {

        TT442_SMR_TUBE2_OUT = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
        TT443_SMR_TUBE3_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
        TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
        TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);
        TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
        TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
        TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
        TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);

        if (TT442_SMR_TUBE2_OUT > 800) {
          Serial.println("TT442_SMR_TUBE2_OUT > 1000C");
          FSM_STATE  = INITIALIZE;
          ERROR = 28;
        }
        if (TT443_SMR_TUBE3_OUT > 800) {
          Serial.println("TT443_SMR_TUBE3_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 29;
        }
        if (TT444_SMR_TUBE4_OUT > 800) {
          Serial.println("TT444_SMR_TUBE4_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 30;
        }
        if (TT445_SMR_TUBE5_OUT > 800) {
          Serial.println("TT445_SMR_TUBE5_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 31;
        }
        if (TT446_SMR_TUBE6_OUT > 800) {
          Serial.println("TT446_SMR_TUBE6_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 32;
        }
        if (TT447_SMR_TUBE7_OUT > 800) {
          Serial.println("TT447_SMR_TUBE7_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 33;
        }
        if (TT448_SMR_TUBE8_OUT > 800) {
          Serial.println("TT448_SMR_TUBE8_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 34;
        }
        if (TT449_SMR_TUBE9_OUT > 800) {
          Serial.println("TT449_SMR_TUBE9_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 35;
        }

        /*
          Serial.print("TT442_SMR_TUBE2_OUT : ");Serial.println(TT442_SMR_TUBE2_OUT);
          Serial.print("TT443_SMR_TUBE3_OUT : ");Serial.println(TT443_SMR_TUBE3_OUT);
          Serial.print("TT444_SMR_TUBE4_OUT : ");Serial.println(TT444_SMR_TUBE4_OUT);
          Serial.print("TT445_SMR_TUBE5_OUT : ");Serial.println(TT445_SMR_TUBE5_OUT);
          Serial.print("TT446_SMR_TUBE6_OUT : ");Serial.println(TT446_SMR_TUBE6_OUT);
          Serial.print("TT447_SMR_TUBE7_OUT : ");Serial.println(TT447_SMR_TUBE7_OUT);
          Serial.print("TT448_SMR_TUBE8_OUT : ");Serial.println(TT448_SMR_TUBE8_OUT);
          Serial.print("TT449_SMR_TUBE9_OUT : ");Serial.println(TT449_SMR_TUBE9_OUT);
        */

        TC_CHECK_COUNTER = 1;
        PREVIOUS_MILLIS_3 = millis();
        //DB_TX();
        break;
      }
  }
}
void readOut() {
  lcd.setBacklight(HIGH);
  switch (READOUT_COUNTER) {

    case 1:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("P1_");
        lcd.print((int)PT304_TWV308_INPUT_PRESSURE);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("P2_");
        lcd.print((int)PT318_HX406_OUTPUT_PRESSURE);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("P3_");
        lcd.print((int)PT420_STEAM_EJECTOR_PRESSURE);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("P4_");
        lcd.print((int)PT213_RO_PRESSURE);//nine tubes
        READOUT_COUNTER = 2;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
    case 2:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S9_");
        lcd.print((int)TT449_SMR_TUBE9_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("H1_");
        lcd.print((int)TT511_SILICON_CARBIDE_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("H2_");
        lcd.print((int)TT513_HX504_IN);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("SH_");
        lcd.print((int)TT301_HX406_STEAM_OUT);//nine tubes
        READOUT_COUNTER = 3;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
    case 3:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.begin(16, 2);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S1_");
        lcd.print((int)TT441_SMR_TUBE1_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("S2_");
        lcd.print((int)TT442_SMR_TUBE2_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S3_");
        lcd.print((int)TT443_SMR_TUBE3_OUT);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("S4_");
        lcd.print((int)TT444_SMR_TUBE4_OUT);//nine tubes
        READOUT_COUNTER = 4;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
    case 4:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S5_");
        lcd.print((int)TT445_SMR_TUBE5_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("S6_");
        lcd.print((int)TT446_SMR_TUBE6_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S7_");
        lcd.print((int)TT447_SMR_TUBE7_OUT);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("S8_");
        lcd.print((int)TT448_SMR_TUBE8_OUT);//nine tubes
        READOUT_COUNTER = 1;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
  }

}
void readBtn() {
  //read if there has been a change for the inputs
  //function for reading buttons and signaling indicators

  //check status of inputs xor last di status woth current di status
  if (!modbusTCPClient8.connected()) {
    modbusTCPClient8.begin(serverIOEX8, 502);
  }
  CURRENT_DI_STATUS_WORD = modbusTCPClient8.inputRegisterRead(48);

  DUN_PSL = bitRead(CURRENT_DI_STATUS_WORD, 4);
  if (DUN_PSL) {
    ERROR = 36;
  }
  DUN_PSH = bitRead(CURRENT_DI_STATUS_WORD, 3);
  if (DUN_PSH) {
    ERROR = 37;
  }
  DUN_ZSL = bitRead(CURRENT_DI_STATUS_WORD, 5);
  if (!DUN_ZSL && FSM_STATE > BMM_IGNITION) {
    ERROR = 38;
  }

  DI_STATUS_CHANGE = CURRENT_DI_STATUS_WORD ^ LAST_DI_STATUS_WORD; //xor to check for a change in inputs
  Serial.print("CURRENT_DI_STATUS_WORD: "); Serial.println(CURRENT_DI_STATUS_WORD);
  Serial.print("DI_STATUS_CHANGE: "); Serial.println(DI_STATUS_CHANGE);
  LAST_DI_STATUS_WORD = CURRENT_DI_STATUS_WORD;


  if (DI_STATUS_CHANGE ) { //check for a change in inputs

    //estop
    if (bitRead(DI_STATUS_CHANGE, 0)) {
      if (bitRead(CURRENT_DI_STATUS_WORD, 0)) { //estop
        ESTOP_FLAG = true; GRN_BTN_FLAG = false; AMB_BTN_FLAG = false; ERROR = 1;
        modbusTCPClient7.coilWrite(0, OFF); //green pilot OFF
        modbusTCPClient7.coilWrite(0, OFF); //AMB pilot OFF
      }
      else {
        ESTOP_FLAG == false;
      }
    }
    if (ESTOP_FLAG == false) {

      //amb button
      /*     if (bitRead(DI_STATUS_CHANGE, 2)) {
              if (bitRead(CURRENT_DI_STATUS_WORD, 2) && FSM_STATE == STABILIZE_MODE) {
                AMB_BTN_FLAG = true;
                //modbusTCPClient7.coilWrite(0,0);//grn pilot light
                modbusTCPClient7.coilWrite(7, 1); //twv901 switch reformate to PSA
                modbusTCPClient7.coilWrite(1, 1); //amb pilot light ON
              }
              else {}
            }*/

      //grn btn
      if (bitRead(DI_STATUS_CHANGE, 1)) {
        if (bitRead(CURRENT_DI_STATUS_WORD, 1)) {
          GRN_BTN_FLAG = true;
          modbusTCPClient7.coilWrite(0, 1); //grn pilot
          //modbusTCPClient7.coilWrite(2,0);//amb pilot
        }
      }
    }
  }
}
void readOCI() {
  // Serial.println("Reading OCI417.");
  OCI_RESULT = NODE.readInputRegisters(OCI_INPUT_STATUS_REGISTER, 2); //address,qty
  //do something with data if read is successful
  if (OCI_RESULT == NODE.ku8MBSuccess)
  {
    OCI_INPUT_STATUS_WORD = NODE.getResponseBuffer(0);
    // Serial.print("OCI INPUT WORD: ");Serial.println(OCI_INPUT_STATUS_WORD,BIN);
    COMBUSTION_PRESSURE_SWITCH = bitRead(OCI_INPUT_STATUS_WORD, 7); //COMBUSTION AIR SW

    OCI_OUTPUT_STATUS_WORD = NODE.getResponseBuffer(1);
    //Serial.print("OCI OUTPUT WORD: ");Serial.println(OCI_OUTPUT_STATUS_WORD,BIN);
    BMM_PROOF_OF_FLAME = bitRead(OCI_OUTPUT_STATUS_WORD, 0);
    if (FSM_STATE > BMM_IGNITION && BMM_PROOF_OF_FLAME == false && FSM_STATE != FINISH) {
      FSM_STATE = INITIALIZE;
      ERROR = 10;
    }
    BMM_ALARM_STATUS = bitRead(OCI_OUTPUT_STATUS_WORD, 1);
    if (FSM_STATE > BMM_IGNITION && BMM_ALARM_STATUS == true) {
      FSM_STATE = INITIALIZE;
      ERROR = 10;
    }
    //OCI_TO_BMM_COM=bitRead(OCI_OUTPUT_STATUS_WORD,2);
    //  if(FSM_STATE>6 && OCI_TO_BMM_COM==true){FSM_STATE=0;ERROR=10;}
  }
}
void error_Checker() {
  //may be redundant but for safety
  //shut gas down if an issue

  if (ERROR) {
    Serial.println("ERROR CHECKER TRIGGERED!");

    GRN_BTN_FLAG = false;
    AMB_BTN_FLAG = false;
    ESTOP_FLAG = true;

    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 (FCV134)
    if (!modbusTCPClient2.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 4 (FCV141)

    if (!modbusTCPClient7.connected()) {
      modbusTCPClient7.begin(serverIOEX7, 502);
    }
    modbusTCPClient7.coilWrite(6, OFF); //TWV308
    // modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(7, OFF); //TWV901

  }
  else {
    ESTOP_FLAG = false;
  }


}
/* void monitor_SR_Tube_Temps() {

   TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(47) * (0.095);
   TT442_SMR_TUBE2_OUT = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
   TT443_SMR_TUBE3_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
   TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
   TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);
   TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
   TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
   TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
   TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);
  /*
   SR_TUBE_AVG_TEMP = (TT441_SMR_TUBE1_OUT + TT442_SMR_TUBE2_OUT + TT443_SMR_TUBE3_OUT
                       + TT444_SMR_TUBE4_OUT + TT445_SMR_TUBE5_OUT + TT446_SMR_TUBE6_OUT + TT447_SMR_TUBE7_OUT
                       + TT448_SMR_TUBE8_OUT + TT449_SMR_TUBE9_OUT) / 9;

  SR_TUBE_AVG_TEMP = ( TT444_SMR_TUBE4_OUT + TT445_SMR_TUBE5_OUT + TT446_SMR_TUBE6_OUT + TT447_SMR_TUBE7_OUT
                       + TT448_SMR_TUBE8_OUT + TT449_SMR_TUBE9_OUT) / 6;

  }*/
void blinkGRN() {

  if (CURRENT_MILLIS - PREVIOUS_MILLIS_5 >= 500 && GRN_BTN_FLAG == false && ESTOP_FLAG == false) {
    PREVIOUS_MILLIS_5 = millis();
    if (GRN_PLT_STATE == false) {
      GRN_PLT_STATE = true;
    }
    else {
      GRN_PLT_STATE = false;
    }
    modbusTCPClient7.coilWrite(0, GRN_PLT_STATE); //grn pilot
  }
}
void blinkAMB() {
  if (CURRENT_MILLIS - PREVIOUS_MILLIS_6 >= 500 && AMB_BTN_FLAG == false && ESTOP_FLAG == false) {
    PREVIOUS_MILLIS_6 = millis();
    if (AMB_PLT_STATE == false) {
      AMB_PLT_STATE = true;
    }
    else {
      AMB_PLT_STATE = false;
    }
    modbusTCPClient7.coilWrite(1, AMB_PLT_STATE); //grn pilot
  }

}
void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
void integrityCheck() {


  if (!SENSOR_INTEGRITY_CHECK) {


    while (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
      Serial.println("CONNECTING IOEX1.");
    }
    while (!modbusTCPClient2.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
      Serial.println("CONNECTING IOEX2.");
    }
    while (!modbusTCPClient3.connected()) {
      modbusTCPClient3.begin(serverIOEX3, 502);
      Serial.println("CONNECTING IOEX3.");
    }
    while (!modbusTCPClient4.connected()) {
      modbusTCPClient4.begin(serverIOEX4, 502);
      Serial.println("CONNECTING IOEX4.");
    }
    while (!modbusTCPClient5.connected()) {
      modbusTCPClient5.begin(serverIOEX5, 502);
      Serial.println("CONNECTING IOEX5.");
    }
    while (!modbusTCPClient7.connected()) {
      modbusTCPClient7.begin(serverIOEX7, 502);
      Serial.println("CONNECTING IOEX7.");
    }
    while (!modbusTCPClient8.connected()) {
      modbusTCPClient8.begin(serverIOEX8, 502);
      Serial.println("CONNECTING IOEX8.");
    }
    while (!modbusTCPClient6.connected()) {
      modbusTCPClient6.begin(serverIOEX6, 502);
      Serial.println("CONNECTING IOEX6.");
    }

    PT318_HX406_OUTPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(3) * (.0004); // read Channel 3 PressureTransducer_318.. 1psi/80counts maximum
    PT213_RO_PRESSURE = modbusTCPClient6.inputRegisterRead(4) * (.0004); // read Channel 4 PT213
    PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient6.inputRegisterRead(5) * (.0004); //read Channel 5 PT420
    PT304_TWV308_INPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(6) * (.0004); // read Channel 6 PT304

    if (PT318_HX406_OUTPUT_PRESSURE >= 240) {
      FSM_STATE = INITIALIZE;
      ERROR = 3;
    }
    else {
      ERROR = 0;
    }
    if (PT213_RO_PRESSURE >= 240) {
      FSM_STATE = INITIALIZE;
      ERROR = 4;
    }
    else {
      ERROR = 0;
    }
    if (PT420_STEAM_EJECTOR_PRESSURE >= 240) {
      FSM_STATE = INITIALIZE;
      ERROR = 5;
    }
    else {
      ERROR = 0;
    }
    if (PT304_TWV308_INPUT_PRESSURE >= 240) {
      FSM_STATE = INITIALIZE;
      ERROR = 6;
    }
    else {
      ERROR = 0;
    }

    TT142_SR_FUEL = modbusTCPClient3.holdingRegisterRead(40) * (0.095);
    TT301_HX406_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(41) * (0.095);
    TT303_HX504_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(42) * (0.095);
    TT306_EJECTOR_STEAM_IN = modbusTCPClient3.holdingRegisterRead(43) * (0.095);
    TT313_HX402_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(44) * (0.095);
    TT319_HX402_STEAM_SYSTEM = modbusTCPClient3.holdingRegisterRead(45) * (0.095);
    TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
    TT408_HTS_IN_LREF = modbusTCPClient3.holdingRegisterRead(47) * (0.095);

    TT410_HTS_OUT_LREF = modbusTCPClient4.holdingRegisterRead(40) * (0.095);
    TT411_FPZ_OUT_LREF = modbusTCPClient4.holdingRegisterRead(41) * (0.095);
    TT430_SMR_TUBES_INLET = modbusTCPClient4.holdingRegisterRead(42) * (0.095);
    TT511_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(43) * (0.095);
    TT512_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(44) * (0.095);
    TT513_HX504_IN = modbusTCPClient4.holdingRegisterRead(45) * (0.095);
    TT514_HX504_OUT = modbusTCPClient4.holdingRegisterRead(46) * (0.095);
    TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(47) * (0.095);

    TT442_SMR_TUBE2_OUT = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
    TT443_SMR_TUBE3_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
    TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
    TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);
    TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
    TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
    TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
    TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);

    //analog outputs
    modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
    modbusTCPClient1.holdingRegisterWrite(41, OFF); //write channel 1 (WP_Speed)
    modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 (FCV134)
    modbusTCPClient1.holdingRegisterWrite(43, OFF); //write channel 3 (FCV205)
    modbusTCPClient2.holdingRegisterWrite(40, OFF); //write channel 0 (FCV141)

    //digital outputs
    modbusTCPClient7.coilWrite(5, OFF); //XV801
    modbusTCPClient7.coilWrite(9, 1); //BLWR_EN ON bc opposite
    modbusTCPClient7.coilWrite(10, 1); //WP_EN ON bc opposite
    modbusTCPClient7.coilWrite(6, OFF); //TWV308
    // modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(7, OFF); //TWV901
    //modbusTCPClient7.coilWrite(2, OFF); //xv909

    modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed) off

  }
}
