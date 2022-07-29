//this is for pcb revision3 with acromags
//Had to change some of the digital outputs for the new PCB
//

//need to figure out digital inputs for the
//green amber and estop buttons.
//As far as the BMM.ALARM and BMM.POF,
//These can be read on the OCI417.10

#include <Wire.h> //For I2C comm
#include <Adafruit_GFX.h>
#include <LedHelper.h>
#include <EEPROM.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_LiquidCrystal.h"

#include<SPI.h>
#include<Ethernet.h>
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>
#include<ModbusMaster.h>
#include<EEPROM.h>
#include<pid.h>

//For OCI417 module
#define MAX485_DE 12
#define MAX485_RE_NEG 13

//ECU jibber jabber
#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

//TIMING PARAMETERS
unsigned long LOOP_TIME;
unsigned long CURRENT_MILLIS=0;//used to keep track of time
unsigned long PREVIOUS_MILLIS=0;//used to keep track of time
unsigned long PREVIOUS_MILLIS_1=0;//used in case 8
unsigned long PREVIOUS_MILLIS_2=0;//used in case 7
unsigned long PREVIOUS_MILLIS_3=0;//used in readTCs function
unsigned long PREVIOUS_MILLIS_4=0;//used in readOut function
unsigned long LOOP_MILLIS=0;//used to caluclate loop function time.
//timing constants
const long SUPERHEAT_TIMER=50000;
const int BMM_OFF_TIMER=30000;
const int BMM_START_TIMER=5000;
const int BMM_PURGE_TIMER=32000;
const long BMM_IGNITION_TIMER=30000;
const long BURNER_RAMP_TIMER=240000;//4mins
const long BURNER_REACH_END_TIMER=2700000;//SECONDS//45mins
const long STEAM_GENERATION_TIMER=1800000;//30 mins
const int STEAM_AT_170PSI_TIMER=10000;//10 seconds
const long OPEN_SR_FUEL_TIMER=180000;//3 mins

//counter
uint8_t TC_CHECK_COUNTER=1;//used in readTC function
uint8_t READOUT_COUNTER=1;//used in readOut function

//BLOWER VARIABLES
const int BLOWER_PURGE_SPEED=4, BLOWER_IGNITION_SPEED=3407,BLOWER_RAMP_END=7150;//2 to keep the dps triggered .17 * 20043counts
const int BLOWER_TOP_SPEED=12750;
uint16_t BLOWER_SPEED_AT_170PSI,BLOWER_SPEED_FEEDBACK=0;//feedback in counts
uint16_t error_PSI=0;
int BLOWER_CALCULATE_SPEED;
bool BLOWER_ENABLE=false;

//RO PUMP VARIABLES
const int MAX_RO_PRESSURE=250;//psi
const int RO_PUMP_AT_10_GRAMS_PER_SEC=2500 ,RO_PUMP_TOP_SPEED=10000;//counts//3000
uint16_t RO_PUMP_FEEDBACK=0;
int RO_CALCULATE_SPEED;
bool RO_PUMP_ENABLE=false;

//FLOW CONTROL VALVES
uint8_t FCV205_HX406_INLET_FLOW=0;//g/sec
uint16_t FCV205_AT_50_PERCENT=10000;//initiated in case 9
uint16_t FCV205_AT_35_PERCENT=7000;//initiated in case 3??
uint16_t FCV134_BURNER_FUEL_FLOW_FB;//counts
const int FCV134_BURNER_FUEL_FLOW_IGNITION=10079,FCV134_BURNER_FUEL_FLOW_RAMP_END=15320;//5 volts ~40 percent open
uint16_t FCV141_SR_FUEL_START_PERCENT=4000;//In counts .2*20000
uint16_t FT132_NG_FEED_FLOW=0; // In grams per second
const int FT132_PIPE_DIA_CONV=0.5;
const int FT132_COUNTS_TO_G_PER_SEC=.27545;
const int FT132_4MA_OFFSET=-3921;
float FT132_ADJUSTED_MEASURE=0;
bool TWV308_STEAM_REFORMER_FEED=false;//NEED TO DETERMINE DIRECTIONS
bool TWV901_LOW_PRESSURE_REFORMATE=false;//NEED TO DETERMINE SIRECTIONS

uint8_t ATMOSPHERIC_PRESSURE=7;//psi

//INITIALIZE PRESSURE TRANSDUCER VARIABLES
uint16_t PT213_RO_PRESSURE,PT318_HX406_OUTPUT_PRESSURE;//psi
uint16_t PT420_STEAM_EJECTOR_PRESSURE,PT304_TWV308_INPUT_PRESSURE;//psi
uint16_t PT304_TWV308_INPUT_PRESSURE_RAW;

//INITIALIZE THERMOCOUPLE VARIABLES
uint16_t TT142_SR_FUEL,TT301_HX406_STEAM_OUT,TT303_HX504_STEAM_OUT,TT306_EJECTOR_STEAM_IN;
uint16_t TT313_HX402_STEAM_OUT,TT319_HX402_STEAM_SYSTEM,TT407_STEAM_REFORMER_OUT_LREF;
uint16_t TT408_HTS_IN_LREF,TT410_HTS_OUT_LREF,TT411_FPZ_OUT_LREF,TT430_SMR_TUBES_INLET;
uint16_t TT511_SILICON_CARBIDE_OUT,TT512_SILICON_CARBIDE_OUT,TT513_HX504_IN,TT514_HX504_OUT;
uint16_t TT441_SMR_TUBE1_OUT,TT442_SMR_TUBE2_OUT,TT443_SMR_TUBE3_OUT,TT444_SMR_TUBE4_OUT;
uint16_t TT445_SMR_TUBE5_OUT,TT446_SMR_TUBE6_OUT,TT447_SMR_TUBE7_OUT,TT448_SMR_TUBE8_OUT;
uint16_t TT449_SMR_TUBE9_OUT;
uint16_t BURNER_TEMP_RAMP_END=800,BURNER_TEMP_CROSSOVER=880;

//OCI417 MODBUS PARAMETERS
const int OCI_INPUT_STATUS_REGISTER=25;
const int OCI_OUTPUT_STATUS_REGISTER=26;
const int OCI_UNIT_ID=1;
uint16_t OCI_INPUT_STATUS_WORD=0;
uint16_t OCI_OUTPUT_STATUS_WORD=0;
uint8_t OCI_RESULT;
bool COMBUSTION_PRESSURE_SWITCH,BMM_ALARM_STATUS;
bool BMM_PROOF_OF_FLAME,OCI_TO_BMM_COM;
bool DUN_PSH,DUN_PSL,DUN_ZSL;
ModbusMaster NODE;

//FLAGS
bool GRN_BTN_FLAG=false,AMB_BTN_FLAG=false,ESTOP_FLAG=false,SENSOR_INTEGRITY_CHECK=false;
bool WATCHDOG_TIMER_FLAG=false;
bool PSI_INIT_TIMER=false;

uint8_t FSM_STATE=0;

uint8_t ERROR=0;//ERROR 0 means no ERROR.

//Mac Address of controller
byte mac[] = {0xdE,0xAD,0xBE,0xEF,0xED};
//needs to be similar to Acromag because will not connect
IPAddress ip(128,1,1,110);

//PARAMETERS FOR ACROMAG SERVERS
EthernetClient ethClient1;
EthernetClient ethClient2;
EthernetClient ethClient3;
EthernetClient ethClient4;
EthernetClient ethClient5;
EthernetClient ethClient6;
EthernetClient ethClient7;

ModbusTCPClient modbusTCPClient1(ethClient1);
ModbusTCPClient modbusTCPClient2(ethClient2);
ModbusTCPClient modbusTCPClient3(ethClient3);
ModbusTCPClient modbusTCPClient4(ethClient4);
ModbusTCPClient modbusTCPClient5(ethClient5);
ModbusTCPClient modbusTCPClient6(ethClient6);
ModbusTCPClient modbusTCPClient7(ethClient7);

IPAddress serverA1(128,1,1,101); // IP of Acromag1 973EN-4006 analog output
IPAddress serverA2(128,1,1,102); // IP of Acromag2 963EN-4012 analog input
IPAddress serverA3(128,1,1,103); // IP of Acromag3 989EN-4016 digital output
IPAddress serverA4(128,1,1,104); // IP of Acromag4 965EN-4006 tc
IPAddress serverA5(128,1,1,105); // IP of Acromag5 965EN-4006 tc
IPAddress serverA6(128,1,1,106); // IP of Acromag6 965EN-4006 tc
IPAddress serverA7(128,1,1,107); // IP of Acromag7 965EN-4006 tc

//PIDs
int16_t BLOWER_SPEED_OFFSET, RO_SPEED_OFFSET,SR_FUEL_OFFSET;
uint16_t BURNER_FUEL_CUT_OFFSET, BURNER_FUEL_CUT;//for pid in case 9 open sr fuel also case 10
float SR_FUEL_CUT=0.3;//grams per second
PID SUPER_HEAT_TT303 = PID(0.1,RO_PUMP_TOP_SPEED,RO_PUMP_AT_10_GRAMS_PER_SEC,1000,0,0);//dt,max,min,kp,kd,ki
PID SUPER_HEAT_TT306 = PID(0.1,RO_PUMP_TOP_SPEED,RO_PUMP_AT_10_GRAMS_PER_SEC,0.9,0.01,0);//dt,max,min,kp,kd,ki
PID STEAM_GEN_PID = PID(0.5, BLOWER_TOP_SPEED, BLOWER_RAMP_END , 5000, 0, 0);//dt,max,min,kp,kd,ki;//0.1,100,-100,0.8,0.01,0.5
PID BURNER_TEMP_CROSSOVER_PID = PID(0.9,20000,13000, 1000, 0, 0);//dt,max,min,kp,kd,ki;//
PID OPEN_SR_FUEL_PID = PID(0.9, 20000, 0, 1, 0.01, 0);//dt,max,min,kp,kd,ki;//

SmallMatrix smallMatrix[3] = {SmallMatrix(0x70), SmallMatrix(0x71), SmallMatrix(0x72) };
LargeMatrix bigMatrix[3] = {LargeMatrix(0x73), LargeMatrix(0x74), LargeMatrix(0x75) };
Adafruit_LiquidCrystal lcd(0);

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

  pinMode(LED_PWR,OUTPUT);
  digitalWrite(LED_PWR,HIGH);
  pinMode(TRACO_24VDC,OUTPUT);
  digitalWrite(TRACO_24VDC,HIGH);
  pinMode(ESTOP_BREAK,OUTPUT);
  digitalWrite(ESTOP_BREAK,HIGH);

  //reseting wifichip
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);
  delay(50);

  //start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  //CONNECT TO ACROMAGS
  while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
  while(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
  while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
  while(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
  while(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
  while(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
  while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}

  readBtn();
  readOCI();
  integrityCheck();

  modbusTCPClient1.holdingRegisterWrite(0x12,0);//write channel 0 (BLWRSpeed) off

  //get that dang dynamic pressure switch off!!
  BLOWER_SPEED_FEEDBACK=modbusTCPClient2.holdingRegisterRead(3016)*(.0005);//10volts/20000counts
  while(BLOWER_SPEED_FEEDBACK>0){
    BLOWER_SPEED_FEEDBACK=modbusTCPClient2.holdingRegisterRead(3016)*(.0005);//10volts/20000counts
    delay(1);
    }

    CURRENT_MILLIS=millis();//So as to not divide by zero on first looptime.
    LOOP_MILLIS=millis();

    lcd.begin(16, 2);

}

void loop() {

  LOOP_TIME=LOOP_MILLIS-CURRENT_MILLIS;

  error_Checker();
  connect_IO_Expanders();
  superheatTest();

  CURRENT_MILLIS=millis();

  Serial.print("Error:");Serial.println(ERROR);//debugging

  readTCs();
  readBtn();
  readOCI();
  readPTs();

  if(!SENSOR_INTEGRITY_CHECK){integrityCheck();}
  if(DUN_PSL){readOCI();}

  connect_IO_Expanders();
  error_Checker();

  switch(FSM_STATE){

   case 0: //initialize
   Serial.println("In case 0");
   if(!SENSOR_INTEGRITY_CHECK){break;}
   modbusTCPClient3.coilWrite(4,255);//CH4 digital output xv909


   if(!COMBUSTION_PRESSURE_SWITCH){
   modbusTCPClient1.holdingRegisterWrite(0x12,(BLOWER_PURGE_SPEED*2000));//write channel 0 (BLWRSpeed)
   modbusTCPClient3.coilWrite(13,0);//BLWR_EN..The logic is opposite
   break;}

   if(!DUN_PSL){Serial.print("Break due to DUN_PSL Switch: ");Serial.println(DUN_PSL);break;}

   //check for RO water???
   modbusTCPClient1.holdingRegisterWrite(0x12,OFF);//write channel 0 (BLWRSpeed)
   modbusTCPClient3.coilWrite(13,255);//BLWR_EN..The logic is opposite

   if(BMM_PROOF_OF_FLAME){   //get that flame out before proceed

    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x14,OFF);//write channel 2 (FCV134)
    modbusTCPClient1.holdingRegisterWrite(0x16,OFF);//write channel 4 (FCV141)

    if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA2,502);}
    modbusTCPClient3.coilWrite(8,OFF);//TWV308
    modbusTCPClient3.coilWrite(6,OFF);//XV1100
    modbusTCPClient3.coilWrite(5,OFF);//XV501
    modbusTCPClient3.coilWrite(12,OFF);//BMM_CR2
    modbusTCPClient3.coilWrite(9,OFF);//TWV901
    modbusTCPClient3.coilWrite(4,255);//CH4 digital output xv909
    break;

    }
   FSM_STATE=1;
   break;

   case 1: //depressurise reformer
   Serial.println("In case 1");
   modbusTCPClient3.coilWrite(8,OFF);//TWV308

   if(PT420_STEAM_EJECTOR_PRESSURE >= ATMOSPHERIC_PRESSURE){

     PT420_STEAM_EJECTOR_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3022)*(.0125))-47;//ref:43016 read Channel 6 PT420
     // PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient2.holdingRegisterRead(3022);
     modbusTCPClient3.coilWrite(4,255);//CH4 digital output xv909

     break;
   }

   modbusTCPClient3.coilWrite(4,OFF);//CH4 digital output xv909

   //250 psi
   if(PT213_RO_PRESSURE<250){

    //just guessing on the fcv205 flow, as long as we stay beneath 250psi
    modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_PURGE_SPEED*2000);//write channel 0 (BLWRSpeed)
    modbusTCPClient1.holdingRegisterWrite(0x15,FCV205_AT_35_PERCENT);//write channel 3 (FCV205)//made steam at 1000//last 3000//~about 35 percent
    modbusTCPClient1.holdingRegisterWrite(0x13,RO_PUMP_AT_10_GRAMS_PER_SEC);//write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000
    modbusTCPClient3.coilWrite(14,OFF);//WP_EN..Control is opposite
    modbusTCPClient3.coilWrite(13,OFF);//BLWR_EN..Control is opposite
    FSM_STATE=2;
    PREVIOUS_MILLIS=millis();
    break;
   }
   break;

   case 2://Superheat Test
   Serial.println("case 2");
   //make sure we are closed//xv909
   modbusTCPClient3.coilWrite(4,OFF);//CH4 digital output xv909
   //monitor tc 407 & 408
   //need a timer

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= SUPERHEAT_TIMER){

     TT303_HX504_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);
     //guessing on 300 degrees centigrade
     if(TT303_HX504_STEAM_OUT >300){
       ERROR=7;FSM_STATE=0;break;
       }
     break;
    }
   FSM_STATE=3;
   PREVIOUS_MILLIS = millis();
   break;

   case 3://bmm off //30 secondsconnect_IO_Expanders();
   Serial.println("case 3");

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_OFF_TIMER){

    modbusTCPClient1.holdingRegisterWrite(0x12,OFF);//write channel 0 (BLWRSpeed)
    modbusTCPClient3.coilWrite(12,OFF);//BMM_CR2 bmm off
    break;
    }
   FSM_STATE=4;
   PREVIOUS_MILLIS = millis();
   modbusTCPClient3.coilWrite(12,255);//BMM_CR2 turn on
   break;

   case 4://bmm on //5 seconds
   Serial.println("Case 4");

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_START_TIMER){
    break;
    }
   PREVIOUS_MILLIS = millis();
   FSM_STATE=5;
   break;

   case 5://bmm purge
   Serial.println("Case 5");

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= BMM_PURGE_TIMER){
    modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_PURGE_SPEED*2000);//write channel 0 (BLWRSpeed)
    break;
    }
   PREVIOUS_MILLIS = millis();
   FSM_STATE=6;
   break;

   case 6://BMM ignition
   Serial.println("Case 6");

   modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_IGNITION_SPEED);//write channel 0 (BLWRSpeed)
   modbusTCPClient1.holdingRegisterWrite(0x14,FCV134_BURNER_FUEL_FLOW_IGNITION);//write channel 2 (FCV134)

   if(CURRENT_MILLIS - PREVIOUS_MILLIS >= BMM_IGNITION_TIMER){

   if(!BMM_PROOF_OF_FLAME){ERROR=8;FSM_STATE=0;break;}
   else{FSM_STATE=7;PREVIOUS_MILLIS=millis();PREVIOUS_MILLIS_2=millis();break;}
   }
   break;

   case 7://start burner ramp//15 minutes//900 seconds
   Serial.println("Case 7");
   //burner ramp is 4mins
   //timing to reach 800c is 15 mins

   TT511_SILICON_CARBIDE_OUT=(modbusTCPClient5.holdingRegisterRead(3017))*(0.07);
   TT513_HX504_IN=(modbusTCPClient6.holdingRegisterRead(3013))*(0.07);

   Serial.print("TT511: ");Serial.println(TT511_SILICON_CARBIDE_OUT);
   Serial.print("TT513: ");Serial.println(TT513_HX504_IN);
   Serial.print("Burner Ramp Timer 15 mins: ");Serial.println(CURRENT_MILLIS - PREVIOUS_MILLIS);

   if(CURRENT_MILLIS - PREVIOUS_MILLIS >= BURNER_REACH_END_TIMER){

    if(TT511_SILICON_CARBIDE_OUT >= BURNER_TEMP_RAMP_END){
     FSM_STATE=8;
     PREVIOUS_MILLIS = millis();
     break;
     }

    if( TT513_HX504_IN >= BURNER_TEMP_RAMP_END){
     FSM_STATE=8;
     PREVIOUS_MILLIS = millis();
     break;
     }

   ERROR=11;FSM_STATE=0;break;
   }

   if(TT511_SILICON_CARBIDE_OUT >= BURNER_TEMP_RAMP_END){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   if( TT513_HX504_IN >= BURNER_TEMP_RAMP_END){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   BLOWER_SPEED_FEEDBACK = modbusTCPClient2.holdingRegisterRead(3016);//10volts/20000counts //~3400
   FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient2.holdingRegisterRead(3026);//ref:43028 read Channel 10 FCV134//~10079

   Serial.print("Blower FB: ");Serial.println(BLOWER_SPEED_FEEDBACK);
   Serial.print("FCV134 FB: ");Serial.println(FCV134_BURNER_FUEL_FLOW_FB);

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= BURNER_RAMP_TIMER){

    if(CURRENT_MILLIS - PREVIOUS_MILLIS_2 >= 5000){

    //if(BLOWER_SPEED_FEEDBACK<=14030){
    //modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_SPEED_FEEDBACK+220);}//write channel 0 blwr speed//44counts/sec
    //if(FCV134_BURNER_FUEL_FLOW_FB<=16121){
    //modbusTCPClient1.holdingRegisterWrite(0x14,FCV134_BURNER_FUEL_FLOW_FB+125);}//25counts/sec

     modbusTCPClient1.holdingRegisterWrite(0x14,FCV134_BURNER_FUEL_FLOW_RAMP_END);//16121 best thus far, fuel
     modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_RAMP_END);//7100 best thus far, blwr
     PREVIOUS_MILLIS_2 = millis();
     break;
     }
   }
   else{
  //modbusTCPClient1.holdingRegisterWrite(0x14,16121);//25counts/sec
  //modbusTCPClient1.holdingRegisterWrite(0x12,13000);//write channel 0 blwr speed//44counts/sec
    break;
    }
   break;

   case 8://Steam generation
   Serial.println("Case 8");

   //steam gen timeout 30 mins
   if(CURRENT_MILLIS - PREVIOUS_MILLIS >= STEAM_GENERATION_TIMER){ERROR=9;FSM_STATE=0;break;}

   //PID control blower to acheive 170PSI pt304
   BLOWER_SPEED_FEEDBACK=modbusTCPClient2.holdingRegisterRead(3016);//10volts/20000counts

   steamGenPID();

   if(PT304_TWV308_INPUT_PRESSURE <= 172 && PT304_TWV308_INPUT_PRESSURE >= 168){
    if(PSI_INIT_TIMER == false){ PREVIOUS_MILLIS_1 = millis(); PSI_INIT_TIMER = true;break;}

    if(CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= STEAM_AT_170PSI_TIMER && PSI_INIT_TIMER==true){
     BLOWER_SPEED_AT_170PSI=BLOWER_SPEED_FEEDBACK;
     modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_SPEED_AT_170PSI);
     modbusTCPClient3.coilWrite(5,255);//XV501 ON
     FSM_STATE=9; PREVIOUS_MILLIS = millis(); PSI_INIT_TIMER=false; BURNER_TEMP_CROSSOVER=TT513_HX504_IN; break;
     }
   }
   else{PSI_INIT_TIMER=false;modbusTCPClient3.coilWrite(5,0);}//If not in bounds of 170-psi, reset timer. Turn on xv501
   break;

   case 9://OPEN SR FUEL
   Serial.println("Case 9:");

   //Point twv308 to reformer
   if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
   modbusTCPClient3.coilWrite(8,255);//TWV308//TWV308 ON//direct steam to reformer
   modbusTCPClient3.coilWrite(7,255);//XV801 On to induce gas feed to reformer

   //assign percentage to fcv141 sr fuel flow control //fcv205 kicked to 50%
   if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
   modbusTCPClient1.holdingRegisterWrite(0x16,FCV141_SR_FUEL_START_PERCENT);//write channel 4 (FCV141)
   modbusTCPClient1.holdingRegisterWrite(0x15,FCV205_AT_50_PERCENT);//write channel 3 (FCV205)

   //read fcv134 for burner temp control
   if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
   FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient2.holdingRegisterRead(3026);//ref:43028 read Channel 10 FCV134

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= OPEN_SR_FUEL_TIMER){

    //pid burner control for 880C
    BURNER_FUEL_CUT_OFFSET = BURNER_TEMP_CROSSOVER_PID.calc_reverse(BURNER_TEMP_CROSSOVER , TT513_HX504_IN);
    Serial.print("BURNER_FUEL_CUT_OFFSET: ");Serial.println(BURNER_FUEL_CUT_OFFSET);
    Serial.print("TT513_HX504_IN: ");Serial.println(TT513_HX504_IN);
    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x14, BURNER_FUEL_CUT_OFFSET);//write channel 2 (FCV134)

    //pid srfuel control for 0.3g/second
    if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
    FT132_NG_FEED_FLOW = modbusTCPClient2.holdingRegisterRead(3019);
    FT132_ADJUSTED_MEASURE=(FT132_NG_FEED_FLOW*FT132_PIPE_DIA_CONV*FT132_COUNTS_TO_G_PER_SEC)-FT132_4MA_OFFSET;
    SR_FUEL_OFFSET=OPEN_SR_FUEL_PID.calculate(SR_FUEL_CUT,FT132_ADJUSTED_MEASURE); //calc offset for FCV141
    Serial.print("SR_FUEL_OFFSET: ");Serial.println(SR_FUEL_OFFSET);
    Serial.print("FT132_ADJUSTED_MEASURE: ");Serial.println(FT132_ADJUSTED_MEASURE);
    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x16,SR_FUEL_OFFSET);//write channel 4 (FCV141)

    //check conformance and restart time if out of bounds
    if((FT132_ADJUSTED_MEASURE>(SR_FUEL_CUT+0.2)) || (BURNER_TEMP_CROSSOVER<(SR_FUEL_CUT-0.2))){PREVIOUS_MILLIS=millis();break;}

    //reset timing if temp falls out of threshold
    if((BURNER_TEMP_CROSSOVER>(TT513_HX504_IN+5)) || (BURNER_TEMP_CROSSOVER<(TT513_HX504_IN-5))){PREVIOUS_MILLIS=millis();break;}

   }

   else{
     if(ERROR==0){FSM_STATE=10;PREVIOUS_MILLIS=millis();break;}
     FSM_STATE=9; PREVIOUS_MILLIS=millis();break;
   }

   break;

   case 10://idle hold mode blower/burner pid
   Serial.println("Case 10:");
   BLOWER_SPEED_FEEDBACK = modbusTCPClient2.holdingRegisterRead(3016);//10volts/20000counts
   FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient2.holdingRegisterRead(3026);//ref:43028 read Channel 10 FCV134

   //3 mins idle ????
   if(CURRENT_MILLIS-PREVIOUS_MILLIS<=180000){

    //burner fuel PID
    BURNER_FUEL_CUT_OFFSET = OPEN_SR_FUEL_PID.calculate(BURNER_TEMP_CROSSOVER , TT513_HX504_IN);
    BURNER_FUEL_CUT= FCV134_BURNER_FUEL_FLOW_FB + BURNER_FUEL_CUT_OFFSET;
    modbusTCPClient1.holdingRegisterWrite(0x14, BURNER_FUEL_CUT);//write channel 2 (FCV134)

    //blower pid
    steamGenPID();

    //check if out of threshold
    if((BURNER_TEMP_CROSSOVER>(TT513_HX504_IN+5)) || (BURNER_TEMP_CROSSOVER<(TT513_HX504_IN-5))){PREVIOUS_MILLIS=millis();break;}
    if(PT304_TWV308_INPUT_PRESSURE <= 171 && PT304_TWV308_INPUT_PRESSURE >= 169){PREVIOUS_MILLIS=millis();break;}
   }

   else{FSM_STATE=11;break;}
   break;

   case 11://reformer stabilize
   Serial.println("Case 11:");
   break;

   default:
   Serial.println("Default.");
   break;

 }//end switch

 LOOP_MILLIS = millis();
//end loop
}

void steamGenPID(){

  if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
 // PT304_TWV308_INPUT_PRESSURE_RAW = modbusTCPClient2.holdingRegisterRead(3023);//ref:43017 read Channel 7 PT304
  PT304_TWV308_INPUT_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3023)*(.0125))-50;//ref:43017 read Channel 7 PT304

  BLOWER_SPEED_OFFSET = STEAM_GEN_PID.calculate(170 , PT304_TWV308_INPUT_PRESSURE);
  Serial.print("BLOWER offset: ");Serial.println(BLOWER_SPEED_OFFSET);

  if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
  modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_SPEED_OFFSET);

  }

void superheatTest(){

  if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
  TT303_HX504_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);
  TT306_EJECTOR_STEAM_IN=modbusTCPClient4.holdingRegisterRead(3015)*(18.4/265);

  if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
  RO_PUMP_FEEDBACK = modbusTCPClient2.holdingRegisterRead(3017);

  RO_SPEED_OFFSET = SUPER_HEAT_TT303.calc_reverse(130 ,TT303_HX504_STEAM_OUT);

    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x13, RO_SPEED_OFFSET);//write channel 1 (WP_Speed)
    if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
    modbusTCPClient3.coilWrite(14,OFF);//WP_EN..Control is opposite

  Serial.print("RO_SPEED_OFFSET: ");Serial.println(RO_SPEED_OFFSET);
  Serial.print("TT303_HX504_STEAM_OUT: ");Serial.println(TT303_HX504_STEAM_OUT);
  Serial.print("TT306_EJECTOR_STEAM_IN: ");Serial.println(TT306_EJECTOR_STEAM_IN);

  /*if(abs(TT303_HX504_STEAM_OUT-TT306_EJECTOR_STEAM_IN)>=10 || TT303_HX504_STEAM_OUT>150){
    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x13, RO_SPEED_OFFSET);//write channel 1 (WP_Speed)
    if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
    modbusTCPClient3.coilWrite(14,OFF);//WP_EN..Control is opposite

 }*/

}

void connect_IO_Expanders(){

  //CONNECT TO ACROMAGS
  if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
  if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
  if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
  if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
  if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
  if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
  if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}
}

void readPTs() {
  Serial.println("Reading PTs");
 if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
 //checking PTs
PT318_HX406_OUTPUT_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3020)*(.0125))-50;//ref:43017 read Channel 4 PressureTransducer_318.. 1psi/80counts maximux
PT213_RO_PRESSURE=(modbusTCPClient2.holdingRegisterRead(3021)*(.0125))-50;//ref:43028 read Channel 5 PT213
PT420_STEAM_EJECTOR_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3022)*(.0125))-47;//ref:43016 read Channel 6 PT420
PT304_TWV308_INPUT_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3023)*(.0125))-50;//ref:43017 read Channel 7 PT304

//if(PT213_RO_PRESSURE>20){modbusTCPClient1.holdingRegisterWrite(0x13,11000);}//write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000}
//else{modbusTCPClient1.holdingRegisterWrite(0x13,RO_PUMP_AT_10_GRAMS_PER_SEC);}//write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000}}


//raw data
//PT318_HX406_OUTPUT_PRESSURE = modbusTCPClient2.holdingRegisterRead(3020);
//PT213_RO_PRESSURE = modbusTCPClient2.holdingRegisterRead(3021);
//PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient2.holdingRegisterRead(3022);
//PT304_TWV308_INPUT_PRESSURE = modbusTCPClient2.holdingRegisterRead(3023);

Serial.print("PT318: ");Serial.println(PT318_HX406_OUTPUT_PRESSURE);
Serial.print("PT213: ");Serial.println(PT213_RO_PRESSURE);
Serial.print("PT420: ");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
Serial.print("PT304: ");Serial.println(PT304_TWV308_INPUT_PRESSURE);

if(PT318_HX406_OUTPUT_PRESSURE>=240){FSM_STATE=0;ERROR=3;Serial.print("PT318 >240PSI");Serial.println(PT318_HX406_OUTPUT_PRESSURE);}
else{ERROR=0;}
if( PT213_RO_PRESSURE>=240){FSM_STATE=0;ERROR=4;Serial.print("PT213 >240PSI");Serial.println(PT213_RO_PRESSURE);}
else{ERROR=0;}
if(PT420_STEAM_EJECTOR_PRESSURE>=240){FSM_STATE=0;ERROR=5;Serial.print("PT420 >240PSI");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);}
else{ERROR=0;}
if(PT304_TWV308_INPUT_PRESSURE>=240){FSM_STATE=0;ERROR=6;Serial.print("PT305 >240PSI");Serial.println(PT304_TWV308_INPUT_PRESSURE);}
else{ERROR=0;}

}

void readTCs(){

 if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
 if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
 if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
 if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}
 /*
 TT142_SR_FUEL=modbusTCPClient4.holdingRegisterRead(3012)*(18.4/265);
 TT301_HX406_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3013)*(18.4/265);
 TT303_HX504_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);
 TT306_EJECTOR_STEAM_IN=modbusTCPClient4.holdingRegisterRead(3015)*(18.4/265);
 TT313_HX402_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3016)*(18.4/265);
 TT319_HX402_STEAM_SYSTEM=modbusTCPClient4.holdingRegisterRead(3017)*(18.4/265);
 TT407_STEAM_REFORMER_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3012)*(18.4/265);
 TT408_HTS_IN_LREF=modbusTCPClient5.holdingRegisterRead(3013)*(18.4/265);
 TT410_HTS_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3014)*(18.4/265);
 TT411_FPZ_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3015)*(18.4/265);
 TT430_SMR_TUBES_INLET=modbusTCPClient5.holdingRegisterRead(3016)*(18.4/265);
 TT511_SILICON_CARBIDE_OUT=modbusTCPClient5.holdingRegisterRead(3017)*(18.4/265);
 TT512_SILICON_CARBIDE_OUT=modbusTCPClient6.holdingRegisterRead(3012)*(18.4/265);
 TT513_HX504_IN=modbusTCPClient6.holdingRegisterRead(3013)*(18.4/265);
 TT514_HX504_OUT=modbusTCPClient6.holdingRegisterRead(3014)*(18.4/265);
 TT441_SMR_TUBE1_OUT=modbusTCPClient6.holdingRegisterRead(3015)*(18.4/265);
 TT442_SMR_TUBE2_OUT=modbusTCPClient6.holdingRegisterRead(3016)*(18.4/265);
 TT443_SMR_TUBE3_OUT=modbusTCPClient6.holdingRegisterRead(3017)*(18.4/265);
 TT444_SMR_TUBE4_OUT=modbusTCPClient7.holdingRegisterRead(3012)*(18.4/265);
 TT445_SMR_TUBE5_OUT=modbusTCPClient7.holdingRegisterRead(3013)*(18.4/265);
 TT446_SMR_TUBE6_OUT=modbusTCPClient7.holdingRegisterRead(3014)*(18.4/265);
 TT447_SMR_TUBE7_OUT=modbusTCPClient7.holdingRegisterRead(3015)*(18.4/265);
 TT448_SMR_TUBE8_OUT=modbusTCPClient7.holdingRegisterRead(3016)*(18.4/265);
 TT449_SMR_TUBE9_OUT=modbusTCPClient7.holdingRegisterRead(3017)*(18.4/265);

 Serial.print("TT142_SR_FUEL: ");Serial.println(TT142_SR_FUEL);
 Serial.print("TT301_HX406_STEAM_OUT : ");Serial.println(TT301_HX406_STEAM_OUT);
 Serial.print("TT303_HX504_STEAM_OUT : ");Serial.println(TT303_HX504_STEAM_OUT);
 Serial.print("TT306_EJECTOR_STEAM_IN : ");Serial.println(TT306_EJECTOR_STEAM_IN);
 Serial.print("TT313_HX402_STEAM_OUT : ");Serial.println(TT313_HX402_STEAM_OUT);
 Serial.print("TT319_HX402_STEAM_SYSTEM : ");Serial.println(TT319_HX402_STEAM_SYSTEM);
 Serial.print("TT407_STEAM_REFORMER_OUT_LREF : ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF);
 Serial.print("TT408_HTS_IN_LREF : ");Serial.println(TT408_HTS_IN_LREF);
 Serial.print("TT410_HTS_OUT_LREF : ");Serial.println(TT410_HTS_OUT_LREF);
 Serial.print("TT411_FPZ_OUT_LREF : ");Serial.println(TT411_FPZ_OUT_LREF);
 Serial.print("TT430_SMR_TUBES_INLET : ");Serial.println(TT430_SMR_TUBES_INLET);
 Serial.print("TT511_SILICON_CARBIDE_OUT : ");Serial.println(TT511_SILICON_CARBIDE_OUT);
 Serial.print("TT512_SILICON_CARBIDE_OUT : ");Serial.println(TT512_SILICON_CARBIDE_OUT);
 Serial.print("TT513_HX504_IN : ");Serial.println(TT513_HX504_IN);
 Serial.print("TT514_HX504_OUT : ");Serial.println(TT514_HX504_OUT);
 Serial.print("TT441_SMR_TUBE1_OUT : ");Serial.println(TT441_SMR_TUBE1_OUT);
 Serial.print("TT442_SMR_TUBE2_OUT : ");Serial.println(TT442_SMR_TUBE2_OUT);
 Serial.print("TT442_SMR_TUBE2_OUT : ");Serial.println(TT442_SMR_TUBE2_OUT);
 Serial.print("TT443_SMR_TUBE3_OUT : ");Serial.println(TT443_SMR_TUBE3_OUT);
 Serial.print("TT444_SMR_TUBE4_OUT : ");Serial.println(TT444_SMR_TUBE4_OUT);
 Serial.print("TT445_SMR_TUBE5_OUT : ");Serial.println(TT445_SMR_TUBE5_OUT);
 Serial.print("TT446_SMR_TUBE6_OUT : ");Serial.println(TT446_SMR_TUBE6_OUT);
 Serial.print("TT447_SMR_TUBE7_OUT : ");Serial.println(TT447_SMR_TUBE7_OUT);
 Serial.print("TT448_SMR_TUBE8_OUT : ");Serial.println(TT448_SMR_TUBE8_OUT);
 Serial.print("TT449_SMR_TUBE9_OUT : ");Serial.println(TT449_SMR_TUBE9_OUT);*/

//read acromag 4

switch(TC_CHECK_COUNTER){
case 1:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000){

 TT142_SR_FUEL=modbusTCPClient4.holdingRegisterRead(3012)*(18.4/265);
 TT301_HX406_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3013)*(18.4/265);
 TT303_HX504_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);
 TT306_EJECTOR_STEAM_IN=modbusTCPClient4.holdingRegisterRead(3015)*(18.4/265);
 TT313_HX402_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3016)*(18.4/265);
 TT319_HX402_STEAM_SYSTEM=modbusTCPClient4.holdingRegisterRead(3017)*(18.4/265);

 if(TT301_HX406_STEAM_OUT>1000){Serial.println("TT301_HX406_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=13;}
 if(TT303_HX504_STEAM_OUT>1000){Serial.println("TT303_HX504_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=14;}
 if(TT306_EJECTOR_STEAM_IN>1000){Serial.println("TT306_EJECTOR_STEAM_IN > 1000C");FSM_STATE=0;ERROR=15;}
 if(TT313_HX402_STEAM_OUT>1000){Serial.println("TT313_HX402_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=16;}
 if(TT319_HX402_STEAM_SYSTEM>1000){Serial.println("TT319_HX402_STEAM_SYSTEM > 1000C");FSM_STATE=0;ERROR=17;}

/* Serial.print("TT142_SR_FUEL: ");Serial.println(TT142_SR_FUEL);
 Serial.print("TT301_HX406_STEAM_OUT : ");Serial.println(TT301_HX406_STEAM_OUT);
 Serial.print("TT303_HX504_STEAM_OUT : ");Serial.println(TT303_HX504_STEAM_OUT);
 Serial.print("TT306_EJECTOR_STEAM_IN : ");Serial.println(TT306_EJECTOR_STEAM_IN);
 Serial.print("TT313_HX402_STEAM_OUT : ");Serial.println(TT313_HX402_STEAM_OUT);
 Serial.print("TT319_HX402_STEAM_SYSTEM : ");Serial.println(TT319_HX402_STEAM_SYSTEM);*/

 PREVIOUS_MILLIS_3=millis();
 TC_CHECK_COUNTER=2;
 break;
 }

 case 2:
//read acromag5
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000){

 TT407_STEAM_REFORMER_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3012)*(18.4/265);
 TT408_HTS_IN_LREF=modbusTCPClient5.holdingRegisterRead(3013)*(18.4/265);
 TT410_HTS_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3014)*(18.4/265);
 TT411_FPZ_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3015)*(18.4/265);
 TT430_SMR_TUBES_INLET=modbusTCPClient5.holdingRegisterRead(3016)*(18.4/265);
 TT511_SILICON_CARBIDE_OUT=modbusTCPClient5.holdingRegisterRead(3017)*(18.4/265);

 if(TT407_STEAM_REFORMER_OUT_LREF>1000){Serial.println("TT407_STEAM_REFORMER_OUT_LREF > 1000C");FSM_STATE=0;ERROR=18;}
 if(TT408_HTS_IN_LREF>1000){Serial.println("TT408_HTS_IN_LREF > 1000C");FSM_STATE=0;ERROR=19;}
 //if(TT410_HTS_OUT_LREF>1000){Serial.println("TT410_HTS_OUT_LREF > 1000C");FSM_STATE=0;ERROR=20;}
 if(TT411_FPZ_OUT_LREF>1000){Serial.println("TT411_FPZ_OUT_LREF > 1000C");FSM_STATE=0;ERROR=21;} //current issue with reading
 //if(TT430_SMR_TUBES_INLET>1000){Serial.println("TT430_SMR_TUBES_INLET > 1000C");FSM_STATE=0;ERROR=22;} current issue with reading
 if(TT511_SILICON_CARBIDE_OUT>1000){Serial.println("TT511_SILICON_CARBIDE_OUT > 1000C");FSM_STATE=0;ERROR=23;}

/* Serial.print("TT407_STEAM_REFORMER_OUT_LREF : ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF);
 Serial.print("TT408_HTS_IN_LREF : ");Serial.println(TT408_HTS_IN_LREF);
 Serial.print("TT410_HTS_OUT_LREF : ");Serial.println(TT410_HTS_OUT_LREF);
 Serial.print("TT411_FPZ_OUT_LREF : ");Serial.println(TT411_FPZ_OUT_LREF);
 Serial.print("TT430_SMR_TUBES_INLET : ");Serial.println(TT430_SMR_TUBES_INLET);
 Serial.print("TT511_SILICON_CARBIDE_OUT : ");Serial.println(TT511_SILICON_CARBIDE_OUT);*/

 TC_CHECK_COUNTER=3;
 PREVIOUS_MILLIS_3=millis();
 break;
 }

 case 3:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3>=3000){

 TT512_SILICON_CARBIDE_OUT=modbusTCPClient6.holdingRegisterRead(3012)*(18.4/265);
 TT513_HX504_IN=modbusTCPClient6.holdingRegisterRead(3013)*(18.4/265);
 TT514_HX504_OUT=modbusTCPClient6.holdingRegisterRead(3014)*(18.4/265);
 TT441_SMR_TUBE1_OUT=modbusTCPClient6.holdingRegisterRead(3015)*(18.4/265);
 TT442_SMR_TUBE2_OUT=modbusTCPClient6.holdingRegisterRead(3016)*(18.4/265);
 TT443_SMR_TUBE3_OUT=modbusTCPClient6.holdingRegisterRead(3017)*(18.4/265);

 if(TT512_SILICON_CARBIDE_OUT>1000){Serial.println("TT512_SILICON_CARBIDE_OUT > 1000C");FSM_STATE=0;ERROR=24;}
 if(TT513_HX504_IN>1000){Serial.println("TT513_HX504_IN > 1000C");FSM_STATE=0;ERROR=25;}
 if(TT514_HX504_OUT>1000){Serial.println("TT514_HX504_OUT > 1000C");FSM_STATE=0;ERROR=26;}
 if(TT441_SMR_TUBE1_OUT>800){Serial.println("TT441_SMR_TUBE1_OUT > 1000C");FSM_STATE=0;ERROR=27;}
 if(TT442_SMR_TUBE2_OUT>800){Serial.println("TT442_SMR_TUBE2_OUT > 1000C");FSM_STATE=0;ERROR=28;}
 if(TT443_SMR_TUBE3_OUT>800){Serial.println("TT443_SMR_TUBE3_OUT > 1000C");FSM_STATE=0;ERROR=29;}

/* Serial.print("TT512_SILICON_CARBIDE_OUT : ");Serial.println(TT512_SILICON_CARBIDE_OUT);
 Serial.print("TT513_HX504_IN : ");Serial.println(TT513_HX504_IN);
 Serial.print("TT514_HX504_OUT : ");Serial.println(TT514_HX504_OUT);
 Serial.print("TT441_SMR_TUBE1_OUT : ");Serial.println(TT441_SMR_TUBE1_OUT);
 Serial.print("TT442_SMR_TUBE2_OUT : ");Serial.println(TT442_SMR_TUBE2_OUT);
 Serial.print("TT443_SMR_TUBE3_OUT : ");Serial.println(TT443_SMR_TUBE3_OUT);*/

 TC_CHECK_COUNTER=4;
 PREVIOUS_MILLIS_3=millis();
 break;
 }

 case 4:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000){

 TT444_SMR_TUBE4_OUT=modbusTCPClient7.holdingRegisterRead(3012)*(18.4/265);
 TT445_SMR_TUBE5_OUT=modbusTCPClient7.holdingRegisterRead(3013)*(18.4/265);
 TT446_SMR_TUBE6_OUT=modbusTCPClient7.holdingRegisterRead(3014)*(18.4/265);
 TT447_SMR_TUBE7_OUT=modbusTCPClient7.holdingRegisterRead(3015)*(18.4/265);
 TT448_SMR_TUBE8_OUT=modbusTCPClient7.holdingRegisterRead(3016)*(18.4/265);
 TT449_SMR_TUBE9_OUT=modbusTCPClient7.holdingRegisterRead(3017)*(18.4/265);

 if(TT444_SMR_TUBE4_OUT>800){Serial.println("TT444_SMR_TUBE4_OUT > 700C");FSM_STATE=0;ERROR=30;}
 if(TT445_SMR_TUBE5_OUT>800){Serial.println("TT445_SMR_TUBE5_OUT > 700C");FSM_STATE=0;ERROR=31;}
 if(TT446_SMR_TUBE6_OUT>800){Serial.println("TT446_SMR_TUBE6_OUT > 700C");FSM_STATE=0;ERROR=32;}
 if(TT447_SMR_TUBE7_OUT>800){Serial.println("TT447_SMR_TUBE7_OUT > 700C");FSM_STATE=0;ERROR=33;}
 if(TT448_SMR_TUBE8_OUT>800){Serial.println("TT448_SMR_TUBE8_OUT > 700C");FSM_STATE=0;ERROR=34;}
 if(TT449_SMR_TUBE9_OUT>800){Serial.println("TT449_SMR_TUBE9_OUT > 700C");FSM_STATE=0;ERROR=35;}

 Serial.print("TT444_SMR_TUBE4_OUT : ");Serial.println(TT444_SMR_TUBE4_OUT);
 Serial.print("TT445_SMR_TUBE5_OUT : ");Serial.println(TT445_SMR_TUBE5_OUT);
 Serial.print("TT446_SMR_TUBE6_OUT : ");Serial.println(TT446_SMR_TUBE6_OUT);
 Serial.print("TT447_SMR_TUBE7_OUT : ");Serial.println(TT447_SMR_TUBE7_OUT);
 Serial.print("TT448_SMR_TUBE8_OUT : ");Serial.println(TT448_SMR_TUBE8_OUT);
 Serial.print("TT449_SMR_TUBE9_OUT : ");Serial.println(TT449_SMR_TUBE9_OUT);

 PREVIOUS_MILLIS_3=millis();
 TC_CHECK_COUNTER=1;
 break;

  }
 }
}

void readOut(){

  switch(READOUT_COUNTER){
    case 1:
    if(CURRENT_MILLIS-PREVIOUS_MILLIS_4>=3000){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("P1");
      lcd.print((int)PT304_TWV308_INPUT_PRESSURE);//nine tubes
      lcd.setCursor(8, 0);
      lcd.print("P2");
      lcd.print((int)PT318_HX406_OUTPUT_PRESSURE);//nine tubes
      lcd.setCursor(14,0);
      lcd.print(FSM_STATE);
      lcd.setCursor(0, 1);
      lcd.print("P3");
      lcd.print((int)PT420_STEAM_EJECTOR_PRESSURE);//nine tubes
      lcd.setCursor(8, 1);
      lcd.print("P4");
      lcd.print((int)PT213_RO_PRESSURE);//nine tubes
      READOUT_COUNTER=2;
      PREVIOUS_MILLIS_4=millis();
    }
    case 2:
     if(CURRENT_MILLIS-PREVIOUS_MILLIS_4>=3000){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S9");
      lcd.print((int)TT449_SMR_TUBE9_OUT);//nine tubes
      lcd.setCursor(8, 0);
      lcd.print("H1");
      lcd.print((int)TT511_SILICON_CARBIDE_OUT);//nine tubes
      lcd.setCursor(14,0);
      lcd.print(FSM_STATE);
      lcd.setCursor(0, 1);
      lcd.print("H2");
      lcd.print((int)TT513_HX504_IN);//nine tubes
      lcd.setCursor(8, 1);
      lcd.print("SH");
      lcd.print((int)TT301_HX406_STEAM_OUT);//nine tubes
      READOUT_COUNTER=3;
      PREVIOUS_MILLIS_4=millis();
   }
    case 3:
     if(CURRENT_MILLIS-PREVIOUS_MILLIS_4>=3000){
      lcd.begin(16, 2);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S1");
      lcd.print((int)TT441_SMR_TUBE1_OUT);//nine tubes
      lcd.setCursor(8, 0);
      lcd.print("S2");
      lcd.print((int)TT442_SMR_TUBE2_OUT);//nine tubes
      lcd.setCursor(14,0);
      lcd.print(FSM_STATE);
      lcd.setCursor(0, 1);
      lcd.print("S3");
      lcd.print((int)TT443_SMR_TUBE3_OUT);//nine tubes
      lcd.setCursor(8, 1);
      lcd.print("S4");
      lcd.print((int)TT444_SMR_TUBE4_OUT);//nine tubes
      READOUT_COUNTER=4;
      PREVIOUS_MILLIS_4=millis();
     }
    case 4:
     if(CURRENT_MILLIS-PREVIOUS_MILLIS_4>=3000){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S5");
      lcd.print((int)TT445_SMR_TUBE5_OUT);//nine tubes
      lcd.setCursor(8, 0);
      lcd.print("S6");
      lcd.print((int)TT446_SMR_TUBE6_OUT);//nine tubes
      lcd.setCursor(14,0);
      lcd.print(FSM_STATE);
      lcd.setCursor(0, 1);
      lcd.print("S7");
      lcd.print((int)TT447_SMR_TUBE7_OUT);//nine tubes
      lcd.setCursor(8, 1);
      lcd.print("S8");
      lcd.print((int)TT448_SMR_TUBE8_OUT);//nine tubes
      READOUT_COUNTER=1;
      PREVIOUS_MILLIS_4=millis();
    }
   }
  }

void integrityCheck(){
  Serial.println("Checking Signal Integrity.");
 if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
 if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
 if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
 if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
 if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
 if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
 if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}

 PT318_HX406_OUTPUT_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3020)*(.0125))-50;//ref:43017 read Channel 4 PressureTransducer_318.. 1psi/80counts maximux
 PT213_RO_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3021)*(.0125))-50;//ref:43028 read Channel 5 PT213
 PT420_STEAM_EJECTOR_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3022)*(.0125))-47;//ref:43016 read Channel 6 PT420
 PT304_TWV308_INPUT_PRESSURE = (modbusTCPClient2.holdingRegisterRead(3023)*(.0125))-50;//ref:43017 read Channel 7 PT304
// if(PT318_HX406_OUTPUT_PRESSURE>=240){FSM_STATE=0;ERROR=3;}
// else{ERROR=0;}
// if(PT213_RO_PRESSURE>=240){FSM_STATE=0;ERROR=4;}
// else{ERROR=0;}
 if(PT420_STEAM_EJECTOR_PRESSURE>=240){FSM_STATE=0;ERROR=5;}
 else{ERROR=0;}
 if(PT304_TWV308_INPUT_PRESSURE>=240){FSM_STATE=0;ERROR=6;}
 else{ERROR=0;}

 TT142_SR_FUEL=modbusTCPClient4.holdingRegisterRead(3012)*(18.4/265);
 TT301_HX406_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3013)*(18.4/265);
 TT303_HX504_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);
 TT306_EJECTOR_STEAM_IN=modbusTCPClient4.holdingRegisterRead(3015)*(18.4/265);
 TT313_HX402_STEAM_OUT=modbusTCPClient4.holdingRegisterRead(3016)*(18.4/265);
 TT319_HX402_STEAM_SYSTEM=modbusTCPClient4.holdingRegisterRead(3017)*(18.4/265);
 TT407_STEAM_REFORMER_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3012)*(18.4/265);
 TT408_HTS_IN_LREF=modbusTCPClient5.holdingRegisterRead(3013)*(18.4/265);
 TT410_HTS_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3014)*(18.4/265);
 TT411_FPZ_OUT_LREF=modbusTCPClient5.holdingRegisterRead(3015)*(18.4/265);
 TT430_SMR_TUBES_INLET=modbusTCPClient5.holdingRegisterRead(3016)*(18.4/265);
 TT511_SILICON_CARBIDE_OUT=modbusTCPClient5.holdingRegisterRead(3017)*(18.4/265);
 TT512_SILICON_CARBIDE_OUT=modbusTCPClient6.holdingRegisterRead(3012)*(18.4/265);
 TT513_HX504_IN=modbusTCPClient6.holdingRegisterRead(3013)*(18.4/265);
 TT514_HX504_OUT=modbusTCPClient6.holdingRegisterRead(3014)*(18.4/265);
 TT441_SMR_TUBE1_OUT=modbusTCPClient6.holdingRegisterRead(3015)*(18.4/265);
 TT442_SMR_TUBE2_OUT=modbusTCPClient6.holdingRegisterRead(3016)*(18.4/265);
 TT443_SMR_TUBE3_OUT=modbusTCPClient6.holdingRegisterRead(3017)*(18.4/265);
 TT444_SMR_TUBE4_OUT=modbusTCPClient7.holdingRegisterRead(3012)*(18.4/265);
 TT445_SMR_TUBE5_OUT=modbusTCPClient7.holdingRegisterRead(3013)*(18.4/265);
 TT446_SMR_TUBE6_OUT=modbusTCPClient7.holdingRegisterRead(3014)*(18.4/265);
 TT447_SMR_TUBE7_OUT=modbusTCPClient7.holdingRegisterRead(3015)*(18.4/265);
 TT448_SMR_TUBE8_OUT=modbusTCPClient7.holdingRegisterRead(3016)*(18.4/265);
 TT449_SMR_TUBE9_OUT=modbusTCPClient7.holdingRegisterRead(3017)*(18.4/265);

  //analog outputs
 modbusTCPClient1.holdingRegisterWrite(0x12,OFF);//write channel 0 (BLWRSpeed) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(0x13,OFF);//write channel 1 (WP_Speed) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(0x14,OFF);//write channel 2 (FCV134) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(0x15,OFF);//write channel 3 (FCV205) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(0x16,OFF);//write channel 4 (FCV141) to 10 volts

 //digital outputs
 modbusTCPClient3.coilWrite(7,OFF);//XV801
 modbusTCPClient3.coilWrite(13,255);//BLWR_EN ON bc opposite
 modbusTCPClient3.coilWrite(14,255);//WP_EN ON bc opposite
 modbusTCPClient3.coilWrite(8,OFF);//TWV308
 modbusTCPClient3.coilWrite(6,OFF);//XV1100
 modbusTCPClient3.coilWrite(5,OFF);//XV501
 modbusTCPClient3.coilWrite(12,OFF);//BMM_CR2
 modbusTCPClient3.coilWrite(9,OFF);//TWV901
 modbusTCPClient3.coilWrite(4,OFF);//xv909

 FCV134_BURNER_FUEL_FLOW_FB = (modbusTCPClient2.holdingRegisterRead(3026)*(.005))-20;//ref:43028 read Channel 10 FCV134
 if(!(FCV134_BURNER_FUEL_FLOW_FB<0.1)){ERROR=2;FSM_STATE=0;SENSOR_INTEGRITY_CHECK=false;}
 else{SENSOR_INTEGRITY_CHECK=true;}

}

void readOCI(){
 // Serial.println("Reading OCI417.");
  OCI_RESULT=NODE.readInputRegisters(OCI_INPUT_STATUS_REGISTER,2);//address,qty
  //do something with data if read is successful
  if (OCI_RESULT==NODE.ku8MBSuccess)
  {
    OCI_INPUT_STATUS_WORD = NODE.getResponseBuffer(0);
   // Serial.print("OCI INPUT WORD: ");Serial.println(OCI_INPUT_STATUS_WORD,BIN);
    DUN_ZSL = bitRead(OCI_INPUT_STATUS_WORD,1);//PROOF OF CLOSURE
    DUN_PSH = bitRead(OCI_INPUT_STATUS_WORD,4);//PRESS SW VALVE PROVING
    DUN_PSL = bitRead(OCI_INPUT_STATUS_WORD,5);//LOW GAS PRESSURE SWITCH
    COMBUSTION_PRESSURE_SWITCH = bitRead(OCI_INPUT_STATUS_WORD,7);//COMBUSTION AIR SW

    OCI_OUTPUT_STATUS_WORD=NODE.getResponseBuffer(1);
    //Serial.print("OCI OUTPUT WORD: ");Serial.println(OCI_OUTPUT_STATUS_WORD,BIN);
    BMM_PROOF_OF_FLAME=bitRead(OCI_OUTPUT_STATUS_WORD,0);
    if(FSM_STATE>6 && BMM_PROOF_OF_FLAME==false){FSM_STATE=0;ERROR=10;}
    BMM_ALARM_STATUS=bitRead(OCI_OUTPUT_STATUS_WORD,1);
    if(FSM_STATE>6 && BMM_ALARM_STATUS==true){FSM_STATE=0;ERROR=10;}
    //OCI_TO_BMM_COM=bitRead(OCI_OUTPUT_STATUS_WORD,2);
    //  if(FSM_STATE>6 && OCI_TO_BMM_COM==true){FSM_STATE=0;ERROR=10;}
  }
}

void preTransmission(){
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void readBtn(){
//  Serial.println("Reading Buttons.");
  //read if there has been a change for the inputs
  //function for reading buttons and signaling indicators

  if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA2,502);}

  if(modbusTCPClient3.inputRegisterRead(3002)){//check for a change in inputs

    //estop
    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),15)){
      ESTOP_FLAG=true;ERROR=1;
      modbusTCPClient3.coilWrite(1,0);
      modbusTCPClient3.coilWrite(3,0);
    }
  if(ESTOP_FLAG==false){

    //amb button
    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),2)){
      AMB_BTN_FLAG=true;GRN_BTN_FLAG=false;
      modbusTCPClient3.coilWrite(1,0);
      modbusTCPClient3.coilWrite(3,1);
    }

    //grn btn
    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),0)){
      GRN_BTN_FLAG=true;AMB_BTN_FLAG=false;
      modbusTCPClient3.coilWrite(1,1);
      modbusTCPClient3.coilWrite(3,0);
   }
  }
 }
}

void error_Checker(){
  //may be redundant but for safety
  //shut gas down if an issue

  if(ERROR){
    Serial.println("ERROR CHECKER TRIGGERED!");
    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(0x14,OFF);//write channel 2 (FCV134)
    modbusTCPClient1.holdingRegisterWrite(0x16,OFF);//write channel 4 (FCV141)

    if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA2,502);}
    modbusTCPClient3.coilWrite(8,OFF);//TWV308
    modbusTCPClient3.coilWrite(6,OFF);//XV1100
    modbusTCPClient3.coilWrite(5,OFF);//XV501
    modbusTCPClient3.coilWrite(12,OFF);//BMM_CR2
    modbusTCPClient3.coilWrite(9,OFF);//TWV901
    modbusTCPClient3.coilWrite(4,255);//CH4 digital output xv909


  }


}


/*

error map
1-estop
2-fcv134 timeout integrity check
3-pt318 over 250psi
4-pt213 over 250psi
5-pt420 over 250psi
6-pt304 over 250psi
7-super heat error
8-No Flame detected
9-Steam gen timeout
10-Loss of Flame
11-Burner reach 800 c timeout

12-Temperature > 1000C
13-temp error
14-
15-
16-
17-
18-
19-
20-
21-
22-
23-
24-
25-
26-
27-
28-
29-
30-
31-
32-
33-
34-
35-

36-

*/
