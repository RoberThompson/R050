
#include <Wire.h> //For I2C comm
#include <Adafruit_GFX.h>
#include <LedHelper.h>
#include <Adafruit_LEDBackpack.h>
#include<Adafruit_LiquidCrystal.h>
#include<SPI.h>//serial peripheral interface
#include<Ethernet3.h>//for ethernet
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>//for modbus tcp/ip
#include<ModbusMaster.h>//for oci417.10 rs485 interface with BMM
#include<pid.h>

#include "DEFINES.h"
#include "GLOBALS.h"

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

//PID controllers
PID SUPER_HEAT_TT303 = PID(0.2, RO_PUMP_TOP_SPEED, RO_PUMP_AT_10_GRAMS_PER_SEC, 500, 0, 0.5); //dt,max,min,kp,kd,ki
PID SUPER_HEAT_TT306 = PID(0.2, RO_PUMP_TOP_SPEED, RO_PUMP_AT_10_GRAMS_PER_SEC, 500, 0, 0.5); //dt,max,min,kp,kd,ki
PID STEAM_GEN_PID = PID(0.3, BLOWER_TOP_SPEED, BLOWER_RAMP_BEGIN , 60 , 0, 0.3 );//dt,max,min,kp,kd,ki;//0.1,100,-100,0.8,0.01,0.5
//PID STEAM_GEN_PID_soft = PID(0.2, BLOWER_TOP_SPEED, BLOWER_RAMP_BEGIN , 20, 0.3, 0.5);
PID BURNER_FUEL_TEMP_CONTROLLER = PID(0.3, 6100, 3700, 30, 0, 0.5);
//PID BURNER_TEMP_CROSSOVER_PID = PID(0.2, 8000, 6500, 1000, 0, 0); //dt,max,min,kp,kd,ki;//
PID OPEN_SR_FUEL_PID = PID(0.2, 6000, FCV141_SR_FUEL_START_PERCENT, 30, 0, 0);//dt,max,min,kp,kd,ki;//
PID FCV205_PID = PID(0.2, 1000, 5200, 5 ,0, 0);
PID monitorSRTubesPID = PID(0.3, 6100, 4300, 75, 0, 0);

//Daughter Board periferals
SmallMatrix smallMatrix[3] = {SmallMatrix(0x70), SmallMatrix(0x71), SmallMatrix(0x72) };
LargeMatrix bigMatrix[3] = {LargeMatrix(0x73), LargeMatrix(0x74), LargeMatrix(0x75) };
Adafruit_LiquidCrystal lcd(0);

void setup() {

//  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode rs485
 // digitalWrite(MAX485_RE_NEG, 0);
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

  // reseting wifichip
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(50);

  //start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  delay(5);
  //Ethernet.phyMode(FULL_DUPLEX_100_AUTONEG);
  Ethernet.phyMode(FULL_DUPLEX_100);
  // Ethernet.phyMode(HALF_DUPLEX_100);
  //  Ethernet.phyMode(FULL_DUPLEX_10);
  //  Ethernet.phyMode(HALF_DUPLEX_10);

  connect_IO_Expanders();
  integrityCheck();
  readBtn();
  readOCI();

  CURRENT_MILLIS = millis(); //So as to not divide by zero on first looptime.
  LOOP_MILLIS = millis();

  lcd.begin(16, 2);

  //DB_INIT();

  SENSOR_INTEGRITY_CHECK = true;

}

void loop() {

  if (FSM_STATE == 0) {
    FSM_STATE_STRING = "INITIALIZE";
  }
  if (FSM_STATE == 1) {
    FSM_STATE_STRING = "DEPRESSURIZE";
  }
  if (FSM_STATE == 2) {
    FSM_STATE_STRING = "SUPERHEAT.TEST";
  }
  if (FSM_STATE == 3) {
    FSM_STATE_STRING = "BMM.OFF";
  }
  if (FSM_STATE == 4) {
    FSM_STATE_STRING = "BMM.ON";
  }
  if (FSM_STATE == 5) {
    FSM_STATE_STRING = "BMM.PURGE";
  }
  if (FSM_STATE == 6) {
    FSM_STATE_STRING = "BMM.IGNITION";
  }
  if (FSM_STATE == 7) {
    FSM_STATE_STRING = "BURNER.RAMP";
  }
  if (FSM_STATE == 8) {
    FSM_STATE_STRING = "STEAM.GEN";
  }
  if (FSM_STATE == 9) {
    FSM_STATE_STRING = "OPEN.SR.FUEL";
  }
  if (FSM_STATE == 10) {
    FSM_STATE_STRING = "IDLE.MODE";
  }
  if (FSM_STATE == 11) {
    FSM_STATE_STRING = "STABILIZE.MODE";
  }
  if (FSM_STATE == 12) {
    FSM_STATE_STRING = "SHUTDOWN.MODE";
  }

  LOOP_MILLIS -=  CURRENT_MILLIS;
  Serial.write('_'); Serial.write('@'); Serial.println(FSM_STATE_STRING);
  Serial.write('_'); Serial.write('!'); Serial.print(F("LOOP.MILLIS: ")); Serial.println(LOOP_MILLIS);
  //Serial.write('_'); Serial.write('!'); Serial.print(F("tt511.ramp.setpoint ")); Serial.println(BURNER_TEMP_RAMP_END);

  CURRENT_MILLIS = millis();

  //DB_RX();

  error_Checker();
  serialEvent();
  connect_IO_Expanders();
  integrityCheck();
  digitalOutputStatus();
  readBtn();
  readPTs();
  superheatTest();
  blinkGRN();
  blinkAMB();
  readTCs();
  readAI();
  readOCI();
  reportConfigurabes();
  OTFOverrides();

  //readOut();
  //GUI();

  if (GRN_BTN_FLAG && !ESTOP_FLAG) {

    switch (FSM_STATE) {

      case INITIALIZE: //initialize//0

        if (BMM_PROOF_OF_FLAME) { //get that flame out before proceed

          if (!modbusTCPClient1.connected()) {
            modbusTCPClient1.begin(serverIOEX1, 502);
          }
          if (!modbusTCPClient2.connected()) {
            modbusTCPClient2.begin(serverIOEX2, 502);
          }

          modbusTCPClient1.holdingRegisterWrite(42, OFF);
          //write channel 2 ioex1 ()

          modbusTCPClient2.holdingRegisterWrite(40, OFF);
          //write channel 0 ioex2 (FCV141)

          if (!modbusTCPClient7.connected()) {
            modbusTCPClient7.begin(serverIOEX7, 502);
          }

          modbusTCPClient7.coilWrite(5, OFF); //XV801
          modbusTCPClient7.coilWrite(6, OFF); //TWV308
          modbusTCPClient7.coilWrite(4, OFF); //XV1100
          modbusTCPClient7.coilWrite(3, OFF); //XV501
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
          modbusTCPClient7.coilWrite(7, OFF); //TWV901
          modbusTCPClient7.coilWrite(2, ON); //CH4 digital output xv909

          break;

        }

        if (!SENSOR_INTEGRITY_CHECK) {

          break;
        }
        modbusTCPClient3.coilWrite(2, ON); //CH2 digital output xv909//ON
        // Serial.write('_'); Serial.write('!'); Serial.print(F("sensor.integrity.check: ")); Serial.println(SENSOR_INTEGRITY_CHECK);

        if (!COMBUSTION_PRESSURE_SWITCH) {

          Serial.write('_'); Serial.write('!'); Serial.print(F("DPS: ")); Serial.println(COMBUSTION_PRESSURE_SWITCH);
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient7.coilWrite(9, ON); //BLWR_EN
          break;
        }

        if (DUN_PSL) {
          // Serial.print("Break due to DUN_PSL Switch: ");
          Serial.write('_'); Serial.write('!'); Serial.print(F("dun.psl: ")); Serial.println(DUN_PSL);
          // Serial.println(DUN_PSL);
          break;
        }

        //check for RO water???//need another sensor
        modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
        modbusTCPClient7.coilWrite(9, OFF); //BLWR_EN..


        FSM_STATE = DEPRESSURIZE;

      /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
        fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
        twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
        twv901Override = false; xv909Override = false;*/
        Serial.write('~'); Serial.write('~');

        break;

      case DEPRESSURIZE: //depressurise reformer//1

        modbusTCPClient7.coilWrite(6, OFF); //TWV308

        if (PT420_STEAM_EJECTOR_PRESSURE >= 20
            || PT213_RO_PRESSURE >= 170
            || PT318_HX406_OUTPUT_PRESSURE >= 50
            || PT304_TWV308_INPUT_PRESSURE >= 50) {

          modbusTCPClient7.coilWrite(2, ON); //CH2 digital output xv909

          break;
        }

        modbusTCPClient7.coilWrite(2, OFF); //CH2 digital output xv909

        //250 psi
        if (PT213_RO_PRESSURE < 250) {

          //just guessing on the fcv205 flow, as long as we stay beneath 250psi
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV205_AT_35_PERCENT); //write channel 2 (FCV205)//made steam at 1000//last 3000//~about 35 percent
          modbusTCPClient1.holdingRegisterWrite(41, RO_PUMP_AT_10_GRAMS_PER_SEC); //write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000
          modbusTCPClient7.coilWrite(10, ON); //WP_EN..
          modbusTCPClient7.coilWrite(9, ON); //BLWR_EN..
          FSM_STATE = SUPERHEAT_TEST;
          PREVIOUS_MILLIS = millis();

        /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }
        break;

      case SUPERHEAT_TEST://Superheat Test //2

        //make sure we are closed//xv909
        modbusTCPClient7.coilWrite(2, OFF); //CH4 digital output xv909
        //monitor tc 407 & 408
        //need a timer

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= SUPERHEAT_TIMER) {

          //guessing on 300 degrees centigrade
          if (TT303_HX504_STEAM_OUT > 300) {
            ERROR = 7; FSM_STATE = INITIALIZE; break;
          }
          break;
        }
        FSM_STATE = BMM_OFF;
        PREVIOUS_MILLIS = millis();

       /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
        fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
        twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
        twv901Override = false; xv909Override = false;*/

        Serial.write('~'); Serial.write('~');

        break;

      case BMM_OFF://bmm off //30 seconds;

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_OFF_TIMER) {

          modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2 bmm off
          break;
        }
        FSM_STATE = BMM_ON;
        PREVIOUS_MILLIS = millis();

       /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
        fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
        twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
        twv901Override = false; xv909Override = false;*/

        Serial.write('~'); Serial.write('~');

        break;

      case BMM_ON:
        //bmm on //5 seconds
        modbusTCPClient7.coilWrite(8, ON); //BMM_CR2 turn on
        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_START_TIMER) {
          break;
        }
        PREVIOUS_MILLIS = millis();
        FSM_STATE = BMM_PURGE;

        /*blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
        fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
        twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
        twv901Override = false; xv909Override = false;*/

        Serial.write('~'); Serial.write('~');

        break;

      case BMM_PURGE:
        //bmm purge

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_PURGE_TIMER) {
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
          break;
        }
        else {

          PREVIOUS_MILLIS_7 = millis();
          PREVIOUS_MILLIS = millis();
          FSM_STATE = BMM_IGNITION;

          old_TT511 =  modbusTCPClient5.holdingRegisterRead(41) * (0.095);
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_IGNITION_SPEED); //write channel 0 (BLWRSpeed)
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_IGNITION); //write channel 2 (FCV134)

        /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }
        break;

      case BMM_IGNITION:
        //BMM igniti

        current_TT511 =  modbusTCPClient5.holdingRegisterRead(41) * (0.095);


        if (current_TT511 >= (old_TT511 + 30) || BMM_PROOF_OF_FLAME) {

          FSM_STATE = BURNER_RAMP;
          PREVIOUS_MILLIS = millis();
          PREVIOUS_MILLIS_2 = millis();
          modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_RAMP_BEGIN);
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_RAMP_BEGIN);

         /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }


        if (CURRENT_MILLIS - PREVIOUS_MILLIS >= BMM_IGNITION_TIMER) {

          if (!BMM_PROOF_OF_FLAME) {
            ERROR = 8;
            FSM_STATE = INITIALIZE;
            break;
          }
          else {
            FSM_STATE = BURNER_RAMP;
            modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_RAMP_BEGIN);
            modbusTCPClient1.holdingRegisterWrite(40, BLOWER_RAMP_BEGIN);
            PREVIOUS_MILLIS = millis();
            PREVIOUS_MILLIS_2 = millis();

           /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
            fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
            twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
            twv901Override = false; xv909Override = false;*/

            Serial.write('~'); Serial.write('~');

            break;
          }
        }
        break;

      case BURNER_RAMP:
        //start burner ramp//15 minutes//900 seconds
        //burner ramp is 4mins
        //timing to reach 800c is 15 mins

        if (CURRENT_MILLIS - PREVIOUS_MILLIS >= BURNER_REACH_END_TIMER) {

          if (TT511_SILICON_CARBIDE_OUT >= BURNER_TEMP_RAMP_END) {
            FSM_STATE = STEAM_GEN;
            PREVIOUS_MILLIS = millis();
            break;
          }

          if ( TT513_HX504_IN >= BURNER_TEMP_RAMP_END) {
            FSM_STATE = STEAM_GEN;
            PREVIOUS_MILLIS = millis();

           /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
            fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
            twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
            twv901Override = false; xv909Override = false;*/

            Serial.write('~'); Serial.write('~');

            break;
          }

          ERROR = 11; FSM_STATE = INITIALIZE; break;
        }

        if (TT511_SILICON_CARBIDE_OUT >= BURNER_TEMP_RAMP_END) {
          FSM_STATE = STEAM_GEN;
          PREVIOUS_MILLIS = millis();

         /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }

        if ( TT513_HX504_IN >= BURNER_TEMP_RAMP_END) {
          FSM_STATE = STEAM_GEN;
          PREVIOUS_MILLIS = millis();

        /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= BURNER_RAMP_TIMER) {

          if (fcv134Override) {
            modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134);
          }
          else {
            modbusTCPClient1.holdingRegisterWrite(42, FCV134_BURNER_FUEL_FLOW_RAMP_END);
          }

          if (blwrOverride) {
            modbusTCPClient1.holdingRegisterWrite(40, cmdSpeedBlower);
          }
          else {
            modbusTCPClient1.holdingRegisterWrite(40, BLOWER_RAMP_END);
          }

          PREVIOUS_MILLIS_2 = millis();
          break;
        }

        else {
          break;
        }
        break;

      case STEAM_GEN://Steam generation

        //steam gen timeout 30 mins
        if (CURRENT_MILLIS - PREVIOUS_MILLIS >= STEAM_GENERATION_TIMER) {
          ERROR = 9;
          FSM_STATE = INITIALIZE;

         /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }

        //PID control blower to acheive 170PSI pt304
        //for assigning speed at crossover
        BLOWER_FB_SR_FUEL = map(modbusTCPClient6.inputRegisterRead(0), 0, 65535, 0, 10000);

        //reach170PSI
        steamGenPID();

        //try to maintain 800c at burner
        burnerFuelPID(BURNER_TEMP_RAMP_END);

        if (PT304_TWV308_INPUT_PRESSURE <= 185 && PT304_TWV308_INPUT_PRESSURE >= 170  ) {
          if (PSI_INIT_TIMER == false) {
            PREVIOUS_MILLIS_1 = millis();
            PSI_INIT_TIMER = true;
            break;
          }

          if (CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= STEAM_AT_170PSI_TIMER && PSI_INIT_TIMER == true) {
            // BLOWER_SPEED_AT_170PSI = BLOWER_FB_SR_FUEL;
            //modbusTCPClient1.holdingRegisterWrite(40, BLOWER_SPEED_AT_170PSI);

            FSM_STATE = OPEN_SR_FUEL;

           /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
            fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
            twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
            twv901Override = false; xv909Override = false;*/

            Serial.write('~'); Serial.write('~');

            PREVIOUS_MILLIS = millis();
            PSI_INIT_TIMER = false;
            break;
          }
        }
        else {
          PSI_INIT_TIMER = false;  //If not in bounds of 170-psi, reset timer. Turn on
          // modbusTCPClient6.coilWrite(3, OFF);//what is this xv501 on client7
        }
        break;

      case OPEN_SR_FUEL:
        //OPEN SR FUEL

        //Point twv308 to reformer
        if (!modbusTCPClient7.connected()) {
          modbusTCPClient7.begin(serverIOEX7, 502);
        }
        if (twv308Override) {
          modbusTCPClient7.coilWrite(6, cmdTwv308);
        }
        else {
          modbusTCPClient7.coilWrite(6, ON);
        } //TWV308//TWV308 ON//direct steam to reformer

        if (xv801Override) {
          modbusTCPClient7.coilWrite(5, cmdXv801);
        }
        else {
          modbusTCPClient7.coilWrite(5, ON);
        } //XV801 On to induce gas feed to reformer

        //assign percentage to fcv141 sr fuel flow control //fcv205 kicked to 50%
        if (!modbusTCPClient2.connected()) {
          modbusTCPClient2.begin(serverIOEX2, 502);
        }
        modbusTCPClient2.holdingRegisterWrite(40, FCV141_SR_FUEL_START_PERCENT); //write channel 4 (FCV141)

        if (!modbusTCPClient1.connected()) {
          modbusTCPClient1.begin(serverIOEX1, 502);
        }
        modbusTCPClient1.holdingRegisterWrite(43, FCV205_AT_50_PERCENT); //write channel 3 (FCV205)

        //read fcv134 for burner temp control
        if (!modbusTCPClient6.connected()) {
          modbusTCPClient6.begin(serverIOEX6, 502);
        }
        FCV134_BURNER_FUEL_FLOW_FB = map( modbusTCPClient6.inputRegisterRead(7), 100, 65535, 0, 100); //read Channel 7 FCV134

        //GET 170 PSI BACK
        steamGenPID();

        //pid burner control for 880C
        burnerFuelPID(BURNER_TEMP_CROSSOVER);

        //pid srfuel control for 0.3g/second
        SRFuelPID();

        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= OPEN_SR_FUEL_TIMER) {

          //check conformance and restart time if out of bounds
          if ((FT132_NG_FEED_FLOW  > (SR_FUEL_CUT + 3)) || (FT132_NG_FEED_FLOW  < (SR_FUEL_CUT - .02))) {
            PREVIOUS_MILLIS = millis();
            break;
          }

          //reset timing if temp falls out of threshold
   //       if ((BURNER_TEMP_CROSSOVER > (TT511_SILICON_CARBIDE_OUT + 50)) || (BURNER_TEMP_CROSSOVER < (TT511_SILICON_CARBIDE_OUT - 50))) {
     //       PREVIOUS_MILLIS = millis();
       //     break;
         // }

        }

        else {
          if (ERROR == 0) {
            FSM_STATE = IDLE_MODE;
            PREVIOUS_MILLIS = millis();

          /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
            fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
            twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
            twv901Override = false; xv909Override = false;*/

            Serial.write('~'); Serial.write('~');

            break;
          }
          FSM_STATE = OPEN_SR_FUEL; PREVIOUS_MILLIS = millis(); break;
        }

        break;

      case IDLE_MODE:
        //idle hold mode blower/burner pid

        BLOWER_FB_SR_FUEL = map(modbusTCPClient6.inputRegisterRead(0), 0, 65535, 0, 10000); //10volts/10000counts
        FCV134_BURNER_FUEL_FLOW_FB = map(modbusTCPClient6.inputRegisterRead(7), 100, 65535, 0, 100); //ref:43028 read Channel 7 FCV134

        //3 mins idle ????
        if (CURRENT_MILLIS - PREVIOUS_MILLIS <= 18000) {

          //pid burner control for 880C
          burnerFuelPID(BURNER_TEMP_CROSSOVER);

          //blower pid
          steamGenPID();

          //SR fuel to 0.3g/s
          SRFuelPID();

          //check if out of threshold
          if ((TT511_SILICON_CARBIDE_OUT > (BURNER_TEMP_CROSSOVER + 70)) || (TT511_SILICON_CARBIDE_OUT < (BURNER_TEMP_CROSSOVER - 70))) {
            PREVIOUS_MILLIS = millis();
            break;
          }
          if (PT304_TWV308_INPUT_PRESSURE >= 190 && PT304_TWV308_INPUT_PRESSURE <= 170) {
            PREVIOUS_MILLIS = millis();
            break;
          }
        }

        else {
          FSM_STATE = STABILIZE_MODE;

         /* blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
          fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
          twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
          twv901Override = false; xv909Override = false;*/

          Serial.write('~'); Serial.write('~');

          break;
        }
        break;

      case STABILIZE_MODE:
        //reformer stabilize
        BLOWER_FB_SR_FUEL = map(modbusTCPClient6.inputRegisterRead(0), 0, 65535, 0, 10000); //10volts/10000counts
        FCV134_BURNER_FUEL_FLOW_FB = map(modbusTCPClient6.inputRegisterRead(7), 100, 65535, 0, 100); //ref:43028 read Channel 7 FCV134

        //pid burner control for 860C
        //burnerFuelPID(BURNER_TEMP_CROSSOVER);

        //monitorSrTEmps to 650 average
        monitor_SR_Tube_Temps();

        //blower pid
        steamGenPID();

        //srfuel pid
        SRFuelPID();

        //fcv205 for HTS temperature control
        fcv205Control();

        break;

      case SHUTDOWN_MODE:

      /*  blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
        fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
        twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
        twv901Override = false; xv909Override = false;*/

        Serial.write('~'); Serial.write('~');

        modbusTCPClient7.coilWrite(6, OFF); //TWV308

        if (PT420_STEAM_EJECTOR_PRESSURE >= 20
            || PT213_RO_PRESSURE >= 170
            || PT318_HX406_OUTPUT_PRESSURE >= 50
            || PT304_TWV308_INPUT_PRESSURE >= 50) {

          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
          modbusTCPClient7.coilWrite(2, ON); //CH2 digital output xv909
          modbusTCPClient7.coilWrite(3, ON); //XV501 supplemental air for cooling
          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED);
          modbusTCPClient1.holdingRegisterWrite(41, RO_PUMP_AT_10_GRAMS_PER_SEC); //write channel 1 (WP_Speed)
          modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 ioex1 (FCV134)
          modbusTCPClient1.holdingRegisterWrite(43, FCV205_AT_35_PERCENT); //write channel 3 (FCV205)

          break;
        }



        if (CURRENT_MILLIS - PREVIOUS_MILLIS_8 <= SHUTDOWN_TIMER) {
          //open nitrogen purge

          if (!modbusTCPClient7.connected()) {
            modbusTCPClient7.begin(serverIOEX2, 502);
          }
          modbusTCPClient7.coilWrite(2, ON); //CH2 digital output xv909
          modbusTCPClient7.coilWrite(3, ON); //XV501 supplemental air for cooling
          modbusTCPClient7.coilWrite(4, ON); //XV1100 nitrogen purge
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
          modbusTCPClient7.coilWrite(6, OFF); //TWV308
          modbusTCPClient7.coilWrite(10, ON); //WP_EN
          modbusTCPClient7.coilWrite(9, ON); //blwr_en

          //purge burn to turn off burner

          if (!modbusTCPClient1.connected()) {
            modbusTCPClient1.begin(serverIOEX2, 502);
          }

          modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED);
          modbusTCPClient1.holdingRegisterWrite(41, RO_PUMP_AT_10_GRAMS_PER_SEC); //write channel 1 (WP_Speed)
          modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 ioex1 (FCV134)
          modbusTCPClient1.holdingRegisterWrite(43, FCV205_AT_35_PERCENT); //write channel 3 (FCV205)

          if (!modbusTCPClient2.connected()) {
            modbusTCPClient2.begin(serverIOEX2, 502);
          }
          modbusTCPClient2.holdingRegisterWrite(40, OFF); //write channel 0 ioex2 (FCV141)

          //keeps steam system in vent

          //keep running water through system

          //run blower at purge

          //after 10 mins completely depower

          break;
        }
        else {


          if (!modbusTCPClient1.connected()) {
            modbusTCPClient1.begin(serverIOEX2, 502);
          }

          modbusTCPClient1.holdingRegisterWrite(40, OFF);//blower
          modbusTCPClient1.holdingRegisterWrite(41, OFF);//water pump
          modbusTCPClient1.holdingRegisterWrite(42, OFF);//write channel 2 ioex1 (FCV134)
          modbusTCPClient1.holdingRegisterWrite(43, OFF);//fcv205

          if (!modbusTCPClient2.connected()) {
            modbusTCPClient2.begin(serverIOEX2, 502);
          }

          modbusTCPClient2.holdingRegisterWrite(40, OFF);//write channel 0 ioex2 (FCV141)


          if (!modbusTCPClient7.connected()) {
            modbusTCPClient7.begin(serverIOEX2, 502);
          }

          modbusTCPClient7.coilWrite(2, OFF); //CH2 digital output xv909
          modbusTCPClient7.coilWrite(6, OFF); //TWV308
          modbusTCPClient7.coilWrite(4, OFF); //XV1100
          modbusTCPClient7.coilWrite(3, OFF); //XV501
          modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
          modbusTCPClient7.coilWrite(7, OFF); //TWV901
          modbusTCPClient7.coilWrite(5, OFF); //XV801 OFF
          modbusTCPClient7.coilWrite(10, OFF); //WP_EN
          modbusTCPClient7.coilWrite(9, OFF); //blwr_en

          AMB_BTN_FLAG == false; GRN_BTN_FLAG = false; FSM_STATE = INITIALIZE; SHUTDOWN_FLAG = false;
          Serial.write('_'); Serial.write('!'); Serial.println(F("TURN OFF MACHINE"));

          break;
        }


      default:

        break;

    }//end switch

  }//end if(GRN_BTN_FLAG && !ESTOP_FLAG)

  LOOP_MILLIS = millis();
  //end loop
}
