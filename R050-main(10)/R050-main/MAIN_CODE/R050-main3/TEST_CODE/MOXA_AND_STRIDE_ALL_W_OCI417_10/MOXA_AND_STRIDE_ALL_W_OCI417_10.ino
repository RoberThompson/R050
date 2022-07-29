////this is for pcb revision4 with MOXAs and STRIDEs
//
//need to figure out digital inputs for the
//green amber and estop buttons.
//As far as the BMM.ALARM and BMM.POF,
//These can be read on the OCI417.10

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
unsigned long LOOP_MILLIS=0;//used to caluclate loop function time.
//timing constants
const long SUPERHEAT_TIMER=50000;
const int BMM_OFF_TIMER=30000;
const int BMM_START_TIMER=5000;
const int BMM_PURGE_TIMER=32000;
const long BMM_IGNITION_TIMER=30000;
const long BURNER_RAMP_TIMER=240000;//4mins
const long BURNER_REACH_815C=1200000;//SECONDS//20mins
const long STEAM_GENERATION_TIMER=1800000;//30 mins
const int STEAM_AT_170PSI_TIMER=10000;//gwrgw
const int OPEN_SR_FUEL_TIMER=180000;//3 mins

//superheat delta constant PARAMETERS
const int SUPERHEAT_DELTA_CONSTANT=0.3;// ~3degreesC per 10ms
uint16_t PREVIOUS_TT407=0,PREVIOUS_TT408=0;

//counter
uint8_t TC_CHECK_COUNTER=1;

//BLOWER VARIABLES
const int BLOWER_PURGE_SPEED=9000, BLOWER_IGNITION_SPEED=1703,BLOWER_RAMP_END=3575;//2 to keep the dps triggered .17 * 10000counts
uint16_t BLOWER_SPEED_AT_170PSI,BLOWER_SPEED_FEEDBACK=0,BLOWER_CALCULATE_SPEED;//feedback in counts
bool BLOWER_ENABLE=false;

//RO PUMP VARIABLES
const int MAX_RO_PRESSURE=250;//psi
const int RO_PUMP_AT_10_GRAMS_PER_SEC=1250;//counts
uint8_t RO_PUMP_SPEED_COMMANDED=0,RO_PUMP_FEEDBACK=0;
bool RO_PUMP_ENABLE=false;

//FLOW CONTROL VALVES
uint8_t FCV205_HX406_INLET_FLOW=0;//g/sec
uint16_t FCV134_BURNER_FUEL_FLOW_FB;//counts
const int FCV134_BURNER_FUEL_FLOW_IGNITION=10079,FCV134_BURNER_FUEL_FLOW_RAMP_END=7660;//5 volts ~40 percent open
uint8_t FCV141_SR_FEED_FLOW=0;//PERCENTAGE
uint8_t FT132_NG_FEED_FLOW=0; // UNSURE OF UNITS??
bool TWV308_STEAM_REFORMER_FEED=false;//NEED TO DETERMINE DIRECTIONS
bool TWV901_LOW_PRESSURE_REFORMATE=false;//NEED TO DETERMINE SIRECTIONS

uint8_t ATMOSPHERIC_PRESSURE=5;//psi

//INITIALIZE PRESSURE TRANSDUCER VARIABLES
uint16_t PT213_RO_PRESSURE,PT318_HX406_OUTPUT_PRESSURE;//psi
uint16_t PT420_STEAM_EJECTOR_PRESSURE,PT304_TWV308_INPUT_PRESSURE;//psi

//INITIALIZE THERMOCOUPLE VARIABLES
uint16_t TT142_SR_FUEL,TT301_HX406_STEAM_OUT,TT303_HX504_STEAM_OUT,TT306_EJECTOR_STEAM_IN;
uint16_t TT313_HX402_STEAM_OUT,TT319_HX402_STEAM_SYSTEM,TT407_STEAM_REFORMER_OUT_LREF;
uint16_t TT408_HTS_IN_LREF,TT410_HTS_OUT_LREF,TT411_FPZ_OUT_LREF,TT430_SMR_TUBES_INLET;
uint16_t TT511_SILICON_CARBIDE_OUT,TT512_SILICON_CARBIDE_OUT,TT513_HX504_IN,TT514_HX504_OUT;
uint16_t TT441_SMR_TUBE1_OUT,TT442_SMR_TUBE2_OUT,TT443_SMR_TUBE3_OUT,TT444_SMR_TUBE4_OUT;
uint16_t TT445_SMR_TUBE5_OUT,TT446_SMR_TUBE6_OUT,TT447_SMR_TUBE7_OUT,TT448_SMR_TUBE8_OUT;
uint16_t TT449_SMR_TUBE9_OUT;
unit16_t HOLD_BURNER_TEMP;

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

//PARAMETERS FOR STRIDE/MOXA SERVERS
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

IPAddress serverIOEX1(128,1,1,101); // IP of ioex1 STRIDE SIO-MB04DAS 4CH Analog Output
IPAddress serverIOEX2(128,1,1,102); // IP of ioex2 STRIDE SIO-MB04DAS 4CH Analog Output
IPAddress serverIOEX3(128,1,1,103); // IP of ioex3 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX4(128,1,1,104); // IP of ioex4 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX5(128,1,1,105); // IP of ioex5 STRIDE SIO-MB04DAS 8CH TC/mV Input
IPAddress serverIOEX6(128,1,1,106); // IP of ioex6 MOXA IOLOGIK E1240 8CH Analog Input
IPAddress serverIOEX7(128,1,1,107); // IP of ioex7 MOXA IOLOGIK E1211 16CH Digital Output
IPAddress serverIOEX8(128,1,1,108); // IP of ioex8 MOXA IOLOGIK E1210 16CH Digital Input

//PIDs
//steam gen pid
PID Steam_Gen_PID = PID(0.1, 1, -1, 0.5, 0.01, 0.5);//dt,max,min,kp,kd,ki;//
PID OPEN_SR_FUEL_PID = PID(0.1, 1000, -1000, 0.5, 0.01, 0.5);//dt,max,min,kp,kd,ki;//
uint8_t BLOWER_SPEED_OFFSET;
uint16_t BURNER_FUEL_CUT_OFFSET, BURNER_FUEL_CUT;//for pid in case 9 open sr fuel also case 10

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

  //CONNECT TO STRIDEs/MOXAs
  while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
  while(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverIOEX2,502);}
  while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverIOEX3,502);}
  while(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverIOEX4,502);}
  while(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverIOEX5,502);}
  while(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverIOEX6,502);}
  while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
  while(!modbusTCPClient8.connected()){modbusTCPClient8.begin(serverIOEX8,502);}

  modbusTCPClient6.holdingRegisterWrite(24,1);//Setting Analog input mode to 4-20mA


  readBtn();
  readOCI();
  integrityCheck();

  modbusTCPClient1.holdingRegisterWrite(40,0);//write channel 0 (BLWRSpeed) off

  //get that dang dynamic pressure switch off!!
  BLOWER_SPEED_FEEDBACK=modbusTCPClient6.holdingRegisterRead(0);//in counts 10000 max counts CH0
  while(BLOWER_SPEED_FEEDBACK>0){
    BLOWER_SPEED_FEEDBACK=modbusTCPClient6.holdingRegisterRead(0);//blwr speed feedback CH0
    delay(1);
    }

    CURRENT_MILLIS=millis();//So as to not divide by zero on first looptime.
    LOOP_MILLIS=millis();

}

void loop() {

  LOOP_TIME=LOOP_MILLIS-CURRENT_MILLIS;
  connect_IO_Expanders();
  superheatTest();
  CURRENT_MILLIS=millis();

  Serial.println(FSM_STATE);//debugging
  Serial.print("Error:");Serial.println(ERROR);//debugging

  readTCs();
  readBtn();
  readOCI();
  readPTs();

  if(!SENSOR_INTEGRITY_CHECK){integrityCheck();}
  if(DUN_PSL){readOCI();}
connect_IO_Expanders();
switch(FSM_STATE){



 case 0: //initialize
  Serial.println("In case 0");
  if(!SENSOR_INTEGRITY_CHECK){break;}
  modbusTCPClient7.coilWrite(2,1);//CH2 digital output xv909 ON

 if(!COMBUSTION_PRESSURE_SWITCH){
  modbusTCPClient1.holdingRegisterWrite(40,BLOWER_PURGE_SPEED);//write channel 0 (BLWRSpeed) 90 percent
  modbusTCPClient7.coilWrite(9,0);//BLWR_EN..The logic is opposite. That is 0 is ON
  break;}

 if(!DUN_PSL){Serial.print("Break due to DUN_PSL Switch: ");Serial.println(DUN_PSL);break;}

 //check for RO water???
 modbusTCPClient1.holdingRegisterWrite(40,OFF);//write channel 0 (BLWRSpeed)
 modbusTCPClient7.holdingRegisterWrite(9,0);// CH9 blower enable opposite logic so 0 is ON
 FSM_STATE=1;break;

 case 1: //depressurise reformer
 Serial.println("In case 1");
  modbusTCPClient7.coilWrite(6,OFF);//TWV308

  if(PT420_STEAM_EJECTOR_PRESSURE >= ATMOSPHERIC_PRESSURE){

    PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient6.inputRegisterRead(5);//read Channel 5 PT420 RAW DATA
    // PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient2.holdingRegisterRead(3022);
    modbusTCPClient7.coilWrite(2,1);//CH2 digital output xv909 ON

    break;
  }

  modbusTCPClient7.coilWrite(2,0);//CH2 digital output xv909 ON

  //250 psi
  if(PT213_RO_PRESSURE<250){

   //just guessing on the fcv205 flow, as long as we stay beneath 250psi
   modbusTCPClient1.holdingRegisterWrite(40,BLOWER_PURGE_SPEED);//write channel 0 (BLWRSpeed) 90 percent
   modbusTCPClient1.holdingRegisterWrite(43,3500);//write channel 3 (FCV205)//made steam at 1000//last 3000//~about 35 percent according to Shawn
   modbusTCPClient1.holdingRegisterWrite(41,RO_PUMP_AT_10_GRAMS_PER_SEC);//write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000
   modbusTCPClient7.coilWrite(10,OFF);//WP_EN..Control is opposite DO10
   modbusTCPClient7.coilWrite(9,OFF);//BLWR_EN..Control is opposite DO9
   FSM_STATE=2;
   PREVIOUS_MILLIS=millis();
   break;
  }

 case 2://Superheat Test
 Serial.println("case 2");
  //make sure we are closed//xv909
  modbusTCPClient7.coilWrite(2,OFF);//CH2 digital output xv909
  //monitor tc 407 & 408
  //need a timer

  if(CURRENT_MILLIS - PREVIOUS_MILLIS <= SUPERHEAT_TIMER){

    TT407_STEAM_REFORMER_OUT_LREF=modbusTCPClient3.holdingRegisterRead(46)*(0.095);//CH6 in celcius
    TT408_HTS_IN_LREF=modbusTCPClient3.holdingRegisterRead(47)*(0.095);//CH7 in celcius
    //guessing on 300 degrees centigrade
    if(TT407_STEAM_REFORMER_OUT_LREF >300){
      ERROR=7;FSM_STATE=0;break;
      }
    if(TT408_HTS_IN_LREF>300){
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

    modbusTCPClient1.holdingRegisterWrite(40,OFF);//write channel 0 (BLWRSpeed)
    modbusTCPClient7.coilWrite(8,OFF);//CH8 digital output BMM_CR2
    break;
   }
  FSM_STATE=4;
  PREVIOUS_MILLIS = millis();
  modbusTCPClient7.coilWrite(8,1);//CH8 digital output BMM_CR2 ON
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

   modbusTCPClient1.holdingRegisterWrite(40,BLOWER_PURGE_SPEED);//write channel 0 (BLWRSpeed)
   break;
   }
  PREVIOUS_MILLIS = millis();
  FSM_STATE=6;
  break;

  case 6://BMM ignition
  Serial.println("Case 6");


  modbusTCPClient1.holdingRegisterWrite(40,BLOWER_IGNITION_SPEED;//write channel 0 (BLWRSpeed)
  modbusTCPClient1.holdingRegisterWrite(42,FCV134_BURNER_FUEL_FLOW_IGNITION);//write channel 2 (FCV134)

  if(CURRENT_MILLIS - PREVIOUS_MILLIS >= BMM_IGNITION_TIMER){

  if(!BMM_PROOF_OF_FLAME){ERROR=8;FSM_STATE=0;break;}
  else{FSM_STATE=7;PREVIOUS_MILLIS=millis();PREVIOUS_MILLIS_2=millis();break;}

  }
  break;

  case 7://start burner ramp//15 minutes//900 seconds
  Serial.println("Case 7");
  //burner ramp is 4mins
  //timing to reach 800c is 15 mins

  TT511_SILICON_CARBIDE_OUT=(modbusTCPClient4.holdingRegisterRead(43))*(0.095);//ch3 celcius
  TT513_HX504_IN=(modbusTCPClient6.holdingRegisterRead(45))*(0.095);//ch5 celcius

  Serial.print("TT511: ");Serial.println(TT511_SILICON_CARBIDE_OUT);
  Serial.print("TT513: ");Serial.println(TT513_HX504_IN);
  Serial.print("Burner Ramp Timer 15 mins: ");Serial.println(CURRENT_MILLIS - PREVIOUS_MILLIS);

  if(CURRENT_MILLIS - PREVIOUS_MILLIS >= BURNER_REACH_815C){

   if(TT511_SILICON_CARBIDE_OUT >= 815){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   if( TT513_HX504_IN >= 815){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   ERROR=11;FSM_STATE=0;break;
   }

   if(TT511_SILICON_CARBIDE_OUT >= 815  ){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   if( TT513_HX504_IN >= 815){
    FSM_STATE=8;
    PREVIOUS_MILLIS = millis();
    break;
    }

   BLOWER_SPEED_FEEDBACK = modbusTCPClient6.holdingRegisterRead(40);//ch0 blwr speed feedback
   FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient6.holdingRegisterRead(47);//ch7 fcv134 feedback

   Serial.print("Blower FB: ");Serial.println(BLOWER_SPEED_FEEDBACK);
   Serial.print("FCV134 FB: ");Serial.println(FCV134_BURNER_FUEL_FLOW_FB);

   if(CURRENT_MILLIS - PREVIOUS_MILLIS <= BURNER_RAMP_TIMER){

    if(CURRENT_MILLIS - PREVIOUS_MILLIS_2 >= 5000){

    //if(BLOWER_SPEED_FEEDBACK<=14030){
    //modbusTCPClient1.holdingRegisterWrite(0x12,BLOWER_SPEED_FEEDBACK+220);}//write channel 0 blwr speed//44counts/sec
    //if(FCV134_BURNER_FUEL_FLOW_FB<=16121){
    //modbusTCPClient1.holdingRegisterWrite(0x14,FCV134_BURNER_FUEL_FLOW_FB+125);}//25counts/sec

     modbusTCPClient1.holdingRegisterWrite(42,FCV134_BURNER_FUEL_FLOW_RAMP_END);//fcv134 ch2
     modbusTCPClient1.holdingRegisterWrite(40,BLOWER_RAMP_END);//blower ch0
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

  Serial.print("Elapsed Time(time 1800ms): ");Serial.println(CURRENT_MILLIS - PREVIOUS_MILLIS);
  //PID control blower to acheive 170PSI pt304
  BLOWER_SPEED_FEEDBACK=modbusTCPClient6.holdingRegisterRead(40);//10volts/20000counts

  if(BLOWER_SPEED_FEEDBACK<8000){

  BLOWER_SPEED_OFFSET = Steam_Gen_PID.calculate(170 , PT304_TWV308_INPUT_PRESSURE);
  BLOWER_CALCULATE_SPEED = BLOWER_SPEED_FEEDBACK + BLOWER_SPEED_OFFSET;
  modbusTCPClient1.holdingRegisterWrite(40,BLOWER_CALCULATE_SPEED);}//write channel 0 (BLWRSpeed)

  if(PT304_TWV308_INPUT_PRESSURE <= 171 && PT304_TWV308_INPUT_PRESSURE >= 169){
  if(PSI_INIT_TIMER == false){ PREVIOUS_MILLIS_1 = millis(); PSI_INIT_TIMER = true; }
  if(CURRENT_MILLIS - PREVIOUS_MILLIS_1 >= STEAM_AT_170PSI_TIMER){
   BLOWER_SPEED_AT_170PSI=BLOWER_SPEED_FEEDBACK;
   modbusTCPClient1.holdingRegisterWrite(40,BLOWER_SPEED_AT_170PSI*1000);
   modbusTCPClient7.coilWrite(3,1);//ch3 XV501 ON
   FSM_STATE=9; PREVIOUS_MILLIS = millis(); PSI_INIT_TIMER=false; =TT513_HX504_IN; break;
  }
 }
 else{PSI_INIT_TIMER=false;}//If not in bounds of 17-psi, reset timer.
 break;

 case 9://OPEN SR FUEL
 Serial.println("Case 9:");

 modbusTCPClient7.coilWrite(6,1);//ch 6 TWV308 ON//channeling right when in front of refromer r050
 modbusTCPClient1.holdingRegisterWrite(43,5000);//write channel 3 (FCV205)//50 percent

 FCV134_BURNER_FUEL_FLOW_FB = (modbusTCPClient6.holdingRegisterRead(47);// read Channel 7 FCV134

 if(CURRENT_MILLIS-PREVIOUS_MILLIS<=180000){

 BURNER_FUEL_CUT_OFFSET = OPEN_SR_FUEL_PID.calculate(HOLD_BURNER_TEMP , TT513_HX504_IN);
 BURNER_FUEL_CUT= FCV134_BURNER_FUEL_FLOW_FB + BURNER_FUEL_CUT_OFFSET;
 modbusTCPClient1.holdingRegisterWrite(42, BURNER_FUEL_CUT);//write channel 2 (FCV134)

//reset timing if temp falls out of threshold
 if((HOLD_BURNER_TEMP>(TT513_HX504_IN+5)) || (HOLD_BURNER_TEMP<(TT513_HX504_IN-5))){PREVIOUS_MILLIS=millis();break;}

 }

 else{
   if(ERROR==0){FSM_STATE=10;PREVIOUS_MILLIS=millis();break;}
   FSM_STATE=9; PREVIOUS_MILLIS=millis();break;
 }

 break;

 case 10://idle hold mode blower/burner pid
 Serial.println("Case 10:");
 BLOWER_SPEED_FEEDBACK = modbusTCPClient6.holdingRegisterRead(40);//10volts/20000counts
 FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient6.holdingRegisterRead(47);//ref:43028 read Channel 10 FCV134

 //3 mins idle
 if(CURRENT_MILLIS-PREVIOUS_MILLIS<=180000){

//burner fuel PID
 BURNER_FUEL_CUT_OFFSET = OPEN_SR_FUEL_PID.calculate(HOLD_BURNER_TEMP , TT513_HX504_IN);
 BURNER_FUEL_CUT= FCV134_BURNER_FUEL_FLOW_FB + BURNER_FUEL_CUT_OFFSET;
 modbusTCPClient1.holdingRegisterWrite(42, BURNER_FUEL_CUT);//write channel 2 (FCV134)

 //blower pid
 BLOWER_SPEED_OFFSET = Steam_Gen_PID.calculate(170 , PT304_TWV308_INPUT_PRESSURE);
 BLOWER_CALCULATE_SPEED = BLOWER_SPEED_FEEDBACK + BLOWER_SPEED_OFFSET;
 modbusTCPClient1.holdingRegisterWrite(40,BLOWER_CALCULATE_SPEED);//write channel 0 (BLWRSpeed)

 //check if out of threshold
 if((HOLD_BURNER_TEMP>(TT513_HX504_IN+5)) || (HOLD_BURNER_TEMP<(TT513_HX504_IN-5))){PREVIOUS_MILLIS=millis();break;}
 if(PT304_TWV308_INPUT_PRESSURE <= 171 && PT304_TWV308_INPUT_PRESSURE >= 169){PREVIOUS_MILLIS=millis();break;}

 }
 else{FSM_STATE=11;break;}
 break;

 case 11://reformer stabilize
 Serial.println("Case 11:");


 default:
 Serial.println("Default.");
 break;

}//end switch

LOOP_MILLIS = millis();
}//end loop

void superheatTest(){

  int DELTA_TT407,DELTA_TT408;

  //These dont seem to be changing enough
  PREVIOUS_TT407=TT407_STEAM_REFORMER_OUT_LREF;
  PREVIOUS_TT408=TT408_HTS_IN_LREF;


  if  (!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverIOEX3,502);Serial.println("Connecting to IOEX3 for superheat test.");}
  TT407_STEAM_REFORMER_OUT_LREF=modbusTCPClient3.holdingRegisterRead(46)*(0.095);//CH6 in celcius
  TT408_HTS_IN_LREF=modbusTCPClient3.holdingRegisterRead(47)*(0.095);//CH6 in celcius

  DELTA_TT407 = (TT407_STEAM_REFORMER_OUT_LREF - PREVIOUS_TT407)/(LOOP_TIME);
  DELTA_TT408 = (TT408_HTS_IN_LREF - PREVIOUS_TT408)/(LOOP_TIME);


  //debug
  Serial.print("CURRENT_TT407: ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF);
  Serial.print("PREVIOUS_TT407: ");Serial.println(PREVIOUS_TT407);
  Serial.print("CURRENT_TT408: ");Serial.println(TT408_HTS_IN_LREF);
  Serial.print("PREVIOUS_TT408: ");Serial.println(PREVIOUS_TT408);
  Serial.print("CURRENT_MILLIS: ");Serial.println(CURRENT_MILLIS);
  Serial.print("LOOP_MILLIS: ");Serial.println(LOOP_MILLIS);
  Serial.print("LOOP TIME: ");Serial.println(LOOP_TIME);
  Serial.print("DELTA_TT407: ");Serial.println(DELTA_TT407);//debugging
  Serial.print("DELTA_TT408: ");Serial.println(DELTA_TT408);//debugging

  if(DELTA_TT407 > SUPERHEAT_DELTA_CONSTANT || DELTA_TT408 > SUPERHEAT_DELTA_CONSTANT){
    modbusTCPClient1.holdingRegisterWrite(41,10000);//write channel 1 (WP_Speed) full speed ahead sire
    modbusTCPClient7.coilWrite(10,OFF);//WP_EN..Control is opposite DO10
  }

  else{
    modbusTCPClient1.holdingRegisterWrite(41,RO_PUMP_AT_10_GRAMS_PER_SEC);//write channel 1 (WP_Speed) full speed ahead sire
    modbusTCPClient7.coilWrite(10,OFF);//WP_EN..Control is opposite DO10
  }

}

void connect_IO_Expanders(){
  //CONNECT TO ACROMAGS
  if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
  if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverIOEX2,502);}
  if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverIOEX3,502);}
  if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverIOEX4,502);}
  if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverIOEX5,502);}
  if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverIOEX6,502);}
  if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
  if(!modbusTCPClient8.connected()){modbusTCPClient8.begin(serverIOEX8,502);}
}

void readPTs() {
  Serial.println("Reading PTs");
 if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverIOEX6,502);}

 //checking PTs
PT318_HX406_OUTPUT_PRESSURE = (modbusTCPClient6.inputRegisterRead(3);//CH3 raw data
PT213_RO_PRESSURE = (modbusTCPClient6.inputRegisterRead(4);//CH4 raw data
PT420_STEAM_EJECTOR_PRESSURE = (modbusTCPClient6.inputRegisterRead(5);//read Channel 5 PT420
PT304_TWV308_INPUT_PRESSURE = (modbusTCPClient6.inputRegisterRead(6);// read Channel 7 PT304

Serial.print("PT318: ");Serial.println(PT318_HX406_OUTPUT_PRESSURE);
Serial.print("PT213: ");Serial.println(PT213_RO_PRESSURE);
Serial.print("PT420: ");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
Serial.print("PT304: ");Serial.println(PT304_TWV308_INPUT_PRESSURE);

if(PT318_HX406_OUTPUT_PRESSURE>=240){FSM_STATE=0;ERROR=3;Serial.print("PT318 >240PSI");Serial.println(PT318_HX406_OUTPUT_PRESSURE);}
else{ERROR=0;}
if(PT213_RO_PRESSURE>=240){FSM_STATE=0;ERROR=4;Serial.print("PT213 >240PSI");Serial.println(PT213_RO_PRESSURE);}
else{ERROR=0;}
if(PT420_STEAM_EJECTOR_PRESSURE>=240){FSM_STATE=0;ERROR=5;Serial.print("PT420 >240PSI");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);}
else{ERROR=0;}
if(PT304_TWV308_INPUT_PRESSURE>=240){FSM_STATE=0;ERROR=6;Serial.print("PT305 >240PSI");Serial.println(PT304_TWV308_INPUT_PRESSURE);}
else{ERROR=0;}

}

void readTCs(){

 if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA4,502);}
 if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA5,502);}
 if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA6,502);}

switch(TC_CHECK_COUNTER){

case 1:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000){

 TT142_SR_FUEL=(modbusTCPClient3.holdingRegisterRead(40))*(0.095);
 TT301_HX406_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(41))*(0.095);
 TT303_HX504_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(42))*(0.095);
 TT306_EJECTOR_STEAM_IN=(modbusTCPClient3.holdingRegisterRead(43))*(0.095);
 TT313_HX402_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(44))*(0.095);
 TT319_HX402_STEAM_SYSTEM=(modbusTCPClient3.holdingRegisterRead(45))*(0.095);
 TT407_STEAM_REFORMER_OUT_LREF=(modbusTCPClient3.holdingRegisterRead(46))*(0.095);
 TT408_HTS_IN_LREF=(modbusTCPClient3.holdingRegisterRead(47))*(0.095);

 if(TT301_HX406_STEAM_OUT>1000){Serial.println("TT301_HX406_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=13;}
 if(TT303_HX504_STEAM_OUT>1000){Serial.println("TT303_HX504_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=14;}
 if(TT306_EJECTOR_STEAM_IN>1000){Serial.println("TT306_EJECTOR_STEAM_IN > 1000C");FSM_STATE=0;ERROR=15;}
 if(TT313_HX402_STEAM_OUT>1000){Serial.println("TT313_HX402_STEAM_OUT > 1000C");FSM_STATE=0;ERROR=16;}
 if(TT319_HX402_STEAM_SYSTEM>1000){Serial.println("TT319_HX402_STEAM_SYSTEM > 1000C");FSM_STATE=0;ERROR=17;}
 if(TT407_STEAM_REFORMER_OUT_LREF>1000){Serial.println("TT407_STEAM_REFORMER_OUT_LREF > 1000C");FSM_STATE=0;ERROR=18;}
 if(TT408_HTS_IN_LREF){Serial.println("TT408_HTS_IN_LREF > 1000C");FSM_STATE=0;ERROR=19;}

 Serial.print("TT142_SR_FUEL: ");Serial.println(TT142_SR_FUEL);
 Serial.print("TT301_HX406_STEAM_OUT : ");Serial.println(TT301_HX406_STEAM_OUT);
 Serial.print("TT303_HX504_STEAM_OUT : ");Serial.println(TT303_HX504_STEAM_OUT);
 Serial.print("TT306_EJECTOR_STEAM_IN : ");Serial.println(TT306_EJECTOR_STEAM_IN);
 Serial.print("TT313_HX402_STEAM_OUT : ");Serial.println(TT313_HX402_STEAM_OUT);
 Serial.print("TT319_HX402_STEAM_SYSTEM : ");Serial.println(TT319_HX402_STEAM_SYSTEM);
 Serial.print("TT407_STEAM_REFORMER_OUT_LREF : ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF;
 Serial.print("TT408_HTS_IN_LREF : ");Serial.println(TT408_HTS_IN_LREF);

 PREVIOUS_MILLIS_3=millis();
 TC_CHECK_COUNTER=2;
 break;
 }

 case 2:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 3000){

 TT410_HTS_OUT_LREF=modbusTCPClient4.holdingRegisterRead(40)*(0.095);
 TT411_FPZ_OUT_LREF=modbusTCPClient4.holdingRegisterRead(41)*(0.095);
 TT430_SMR_TUBES_INLET=modbusTCPClient4.holdingRegisterRead(42)*(0.095);
 TT511_SILICON_CARBIDE_OUT=modbusTCPClient4.holdingRegisterRead(43)*(0.095);
 TT512_SILICON_CARBIDE_OUT=modbusTCPClient4.holdingRegisterRead(44)*(0.095);
 TT513_HX504_IN=modbusTCPClient4.holdingRegisterRead(45)*(0.095);
 TT514_HX504_OUT=modbusTCPClient4.holdingRegisterRead(46)*(0.095);
 TT441_SMR_TUBE1_OUT=modbusTCPClient4.holdingRegisterRead(47)*(0.095);

 if(TT410_HTS_OUT_LREF>1000){Serial.println("TT410_HTS_OUT_LREF > 1000C");FSM_STATE=0;ERROR=18;}
 if(TT411_FPZ_OUT_LREF>1000){Serial.println("TT411_FPZ_OUT_LREF > 1000C");FSM_STATE=0;ERROR=20;}
 if(TT430_SMR_TUBES_INLET>1000){Serial.println("TT430_SMR_TUBES_INLET > 1000C");FSM_STATE=0;ERROR=21;}
 if(TT511_SILICON_CARBIDE_OUT>1000){Serial.println("TT511_SILICON_CARBIDE_OUT> 1000C");FSM_STATE=0;ERROR=22;} //current issue with reading
 if(TT512_SILICON_CARBIDE_OUT>1000){Serial.println("TT512_SILICON_CARBIDE_OUT> 1000C");FSM_STATE=0;ERROR=23;} //current issue with reading
 if(TT513_HX504_IN>1000){Serial.println("TT513_HX504_IN > 1000C");FSM_STATE=0;ERROR=24;}
 if(TT514_HX504_OUT>1000){Serial.println("TT514_HX504_OUT > 1000C");FSM_STATE=0;ERROR=25;}
 if(TT441_SMR_TUBE1_OUT>1000){Serial.println("TT441_SMR_TUBE1_OUT > 1000C");FSM_STATE=0;ERROR=26;}

 Serial.print("TT410_HTS_OUT_LREF : ");Serial.println(TT410_HTS_OUT_LREF);
 Serial.print("TT411_FPZ_OUT_LREF : ");Serial.println(TT411_FPZ_OUT_LREF);
 Serial.print("TT430_SMR_TUBES_INLET : ");Serial.println(TT430_SMR_TUBES_INLET);
 Serial.print("TT511_SILICON_CARBIDE_OUT : ");Serial.println(TT511_SILICON_CARBIDE_OUT);
 Serial.print("TT512_SILICON_CARBIDE_OUT: ");Serial.println(TT512_SILICON_CARBIDE_OUT;
 Serial.print("TT513_HX504_IN : ");Serial.println(TT513_HX504_IN);
 Serial.print("TT514_HX504_OUT: ");Serial.println(TT514_HX504_OUT);
 Serial.print("TT441_SMR_TUBE1_OUT : ");Serial.println(TT441_SMR_TUBE1_OUT;

 TC_CHECK_COUNTER=3;
 PREVIOUS_MILLIS_3=millis();
 break;
 }

 case 3:
 if(CURRENT_MILLIS - PREVIOUS_MILLIS_3>=3000){

 TT442_SMR_TUBE2_OUT=modbusTCPClient5.holdingRegisterRead(40)*(0.95);
 TT443_SMR_TUBE3_OUT=modbusTCPClient5.holdingRegisterRead(41)*(0.95);
 TT444_SMR_TUBE4_OUT=modbusTCPClient5.holdingRegisterRead(42)*(0.95);
 TT445_SMR_TUBE5_OUT=modbusTCPClient5.holdingRegisterRead(43)*(0.95);
 TT446_SMR_TUBE6_OUT=modbusTCPClient5.holdingRegisterRead(44)*(0.95);
 TT447_SMR_TUBE7_OUT=modbusTCPClient5.holdingRegisterRead(45)*(0.95);
 TT448_SMR_TUBE8_OUT=modbusTCPClient5.holdingRegisterRead(46)*(0.95);
 TT449_SMR_TUBE9_OUT=modbusTCPClient5.holdingRegisterRead(47)*(0.95);

 if(TT442_SMR_TUBE2_OUT>1000){Serial.println("TT442_SMR_TUBE2_OUT > 1000C");FSM_STATE=0;ERROR=27;}
 if(TT443_SMR_TUBE3_OUT>1000){Serial.println("TT443_SMR_TUBE3_OUT > 1000C");FSM_STATE=0;ERROR=28;}
 if(TT444_SMR_TUBE4_OUT>1000){Serial.println("TT444_SMR_TUBE4_OUT > 1000C");FSM_STATE=0;ERROR=29;}
 if(TT445_SMR_TUBE5_OUT>1000){Serial.println("TT445_SMR_TUBE5_OUT> 1000C");FSM_STATE=0;ERROR=30;}
 if(TT446_SMR_TUBE6_OUT>1000){Serial.println("TT446_SMR_TUBE6_OUT> 1000C");FSM_STATE=0;ERROR=31;}
 if(TT447_SMR_TUBE7_OUT>1000){Serial.println("TT447_SMR_TUBE7_OUT > 1000C");FSM_STATE=0;ERROR=32;}
 if(TT448_SMR_TUBE8_OUT>1000){Serial.println("TT448_SMR_TUBE8_OUT> 1000C");FSM_STATE=0;ERROR=32;}
 if(TT449_SMR_TUBE9_OUT>1000){Serial.println("TT449_SMR_TUBE9_OUT > 1000C");FSM_STATE=0;ERROR=32;}

 Serial.print("TT442_SMR_TUBE2_OUT : ");Serial.println(TT442_SMR_TUBE2_OUT);
 Serial.print("TT443_SMR_TUBE3_OUT: ");Serial.println(TT443_SMR_TUBE3_OUT);
 Serial.print("TT444_SMR_TUBE4_OUT : ");Serial.println(TT444_SMR_TUBE4_OUT);
 Serial.print("TT445_SMR_TUBE5_OUT: ");Serial.println(TT445_SMR_TUBE5_OUT);
 Serial.print("TT446_SMR_TUBE6_OUT: ");Serial.println(TT446_SMR_TUBE6_OUT);
 Serial.print("TT447_SMR_TUBE7_OUT: ");Serial.println(TT447_SMR_TUBE7_OUT);
 Serial.print("TT448_SMR_TUBE8_OUT : ");Serial.println(TT448_SMR_TUBE8_OUT);
 Serial.print("TT449_SMR_TUBE9_OUT : ");Serial.println(TT449_SMR_TUBE9_OUT);

 TC_CHECK_COUNTER=0;
 PREVIOUS_MILLIS_3=millis();
 break;
 }

}

TT407_STEAM_REFORMER_OUT_LREF=(modbusTCPClient3.holdingRegisterRead(46))*(0.095);
TT408_HTS_IN_LREF=(modbusTCPClient3.holdingRegisterRead(47))*(0.095);

}

void integrityCheck(){
 Serial.println("Checking Signal Integrity.");
 if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
 if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverIOEX2,502);}
 if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverIOEX3,502);}
 if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverIOEX4,502);}
 if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverIOEX5,502);}
 if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverIOEX6,502);}
 if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
 if(!modbusTCPClient8.connected()){modbusTCPClient8.begin(serverIOEX8,502);}

 //checking PTs
PT318_HX406_OUTPUT_PRESSURE = (modbusTCPClient6.inputRegisterRead(3);//CH3 raw data
PT213_RO_PRESSURE = (modbusTCPClient6.inputRegisterRead(4);//CH4 raw data
PT420_STEAM_EJECTOR_PRESSURE = (modbusTCPClient6.inputRegisterRead(5);//read Channel 5 PT420
PT304_TWV308_INPUT_PRESSURE = (modbusTCPClient6.inputRegisterRead(6);// read Channel 7 PT304

Serial.print("PT318: ");Serial.println(PT318_HX406_OUTPUT_PRESSURE);
Serial.print("PT213: ");Serial.println(PT213_RO_PRESSURE);
Serial.print("PT420: ");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
Serial.print("PT304: ");Serial.println(PT304_TWV308_INPUT_PRESSURE);

if(PT318_HX406_OUTPUT_PRESSURE>=240){FSM_STATE=0;ERROR=3;Serial.print("PT318 >240PSI");Serial.println(PT318_HX406_OUTPUT_PRESSURE);}
else{ERROR=0;}
if(PT213_RO_PRESSURE>=240){FSM_STATE=0;ERROR=4;Serial.print("PT213 >240PSI");Serial.println(PT213_RO_PRESSURE);}
else{ERROR=0;}
if(PT420_STEAM_EJECTOR_PRESSURE>=240){FSM_STATE=0;ERROR=5;Serial.print("PT420 >240PSI");Serial.println(PT420_STEAM_EJECTOR_PRESSURE);}
else{ERROR=0;}
if(PT304_TWV308_INPUT_PRESSURE>=240){FSM_STATE=0;ERROR=6;Serial.print("PT305 >240PSI");Serial.println(PT304_TWV308_INPUT_PRESSURE);}
else{ERROR=0;}


 TT142_SR_FUEL=(modbusTCPClient3.holdingRegisterRead(40))*(0.095);
 TT301_HX406_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(41))*(0.095);
 TT303_HX504_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(42))*(0.095);
 TT306_EJECTOR_STEAM_IN=(modbusTCPClient3.holdingRegisterRead(43))*(0.095);
 TT313_HX402_STEAM_OUT=(modbusTCPClient3.holdingRegisterRead(44))*(0.095);
 TT319_HX402_STEAM_SYSTEM=(modbusTCPClient3.holdingRegisterRead(45))*(0.095);
 TT407_STEAM_REFORMER_OUT_LREF=(modbusTCPClient3.holdingRegisterRead(46))*(0.095);
 TT408_HTS_IN_LREF=(modbusTCPClient3.holdingRegisterRead(47))*(0.095);

 TT410_HTS_OUT_LREF=modbusTCPClient4.holdingRegisterRead(40)*(0.095);
 TT411_FPZ_OUT_LREF=modbusTCPClient4.holdingRegisterRead(41)*(0.095);
 TT430_SMR_TUBES_INLET=modbusTCPClient4.holdingRegisterRead(42)*(0.095);
 TT511_SILICON_CARBIDE_OUT=modbusTCPClient4.holdingRegisterRead(43)*(0.095);
 TT512_SILICON_CARBIDE_OUT=modbusTCPClient4.holdingRegisterRead(44)*(0.095);
 TT513_HX504_IN=modbusTCPClient4.holdingRegisterRead(45)*(0.095);
 TT514_HX504_OUT=modbusTCPClient4.holdingRegisterRead(46)*(0.095);
 TT441_SMR_TUBE1_OUT=modbusTCPClient4.holdingRegisterRead(47)*(0.095);

 TT442_SMR_TUBE2_OUT=modbusTCPClient5.holdingRegisterRead(40)*(0.95);
 TT443_SMR_TUBE3_OUT=modbusTCPClient5.holdingRegisterRead(41)*(0.95);
 TT444_SMR_TUBE4_OUT=modbusTCPClient5.holdingRegisterRead(42)*(0.95);
 TT445_SMR_TUBE5_OUT=modbusTCPClient5.holdingRegisterRead(43)*(0.95);
 TT446_SMR_TUBE6_OUT=modbusTCPClient5.holdingRegisterRead(44)*(0.95);
 TT447_SMR_TUBE7_OUT=modbusTCPClient5.holdingRegisterRead(45)*(0.95);
 TT448_SMR_TUBE8_OUT=modbusTCPClient5.holdingRegisterRead(46)*(0.95);
 TT449_SMR_TUBE9_OUT=modbusTCPClient5.holdingRegisterRead(47)*(0.95);

  //analog outputs
 modbusTCPClient1.holdingRegisterWrite(40,OFF);//write channel 0 (BLWRSpeed) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(41,OFF);//write channel 1 (WP_Speed) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(42,OFF);//write channel 2 (FCV134) to 10 volts
 modbusTCPClient1.holdingRegisterWrite(43,OFF);//write channel 3 (FCV205) to 10 volts
 modbusTCPClient2.holdingRegisterWrite(40,OFF);//write channel 0 (FCV141) to 10 volts

 //digital outputs
 modbusTCPClient7.holdingRegisterWrite(32,0b0000011000000000);//blwr & wp are inverse logic

 FCV134_BURNER_FUEL_FLOW_FB = (modbusTCPClient6.holdingRegisterRead(47);// read Channel 7 FCV134
 if(!(FCV134_BURNER_FUEL_FLOW_FB<2000)){ERROR=2;FSM_STATE=0;SENSOR_INTEGRITY_CHECK=false;}
 else{SENSOR_INTEGRITY_CHECK=true;}

}

void readOCI(){
  Serial.println("Reading OCI417.");
  OCI_RESULT=NODE.readInputRegisters(OCI_INPUT_STATUS_REGISTER,2);//address,qty
  //do something with data if read is successful
  if (OCI_RESULT==NODE.ku8MBSuccess)
  {
    OCI_INPUT_STATUS_WORD = NODE.getResponseBuffer(0);
    Serial.print("OCI INPUT WORD: ");Serial.println(OCI_INPUT_STATUS_WORD,BIN);
    DUN_ZSL = bitRead(OCI_INPUT_STATUS_WORD,1);//PROOF OF CLOSURE
    DUN_PSH = bitRead(OCI_INPUT_STATUS_WORD,4);//PRESS SW VALVE PROVING
    DUN_PSL = bitRead(OCI_INPUT_STATUS_WORD,5);//LOW GAS PRESSURE SWITCH
    COMBUSTION_PRESSURE_SWITCH = bitRead(OCI_INPUT_STATUS_WORD,7);//COMBUSTION AIR SW

    OCI_OUTPUT_STATUS_WORD=NODE.getResponseBuffer(1);
    Serial.print("OCI OUTPUT WORD: ");Serial.println(OCI_OUTPUT_STATUS_WORD,BIN);
    BMM_PROOF_OF_FLAME=bitRead(OCI_OUTPUT_STATUS_WORD,0);
    if(FSM_STATE>6 && BMM_PROOF_OF_FLAME==false){FSM_STATE=0;ERROR=10;}
    BMM_ALARM_STATUS=bitRead(OCI_OUTPUT_STATUS_WORD,1);
    OCI_TO_BMM_COM=bitRead(OCI_OUTPUT_STATUS_WORD,2);
  }
}

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void readBtn(){
  Serial.println("Reading Buttons.");
  //read if there has been a change for the inputs
  //function for reading buttons and signaling indicators

  if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA2,502);}

  if(modbusTCPClient3.inputRegisterRead(3002)){

    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),15)){
      ESTOP_FLAG=true;ERROR=1;
      modbusTCPClient3.coilWrite(1,0);
      modbusTCPClient3.coilWrite(3,0);
    }
  if(ESTOP_FLAG==false){

    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),2)){
      AMB_BTN_FLAG=true;GRN_BTN_FLAG=false;
      modbusTCPClient3.coilWrite(1,0);
      modbusTCPClient3.coilWrite(3,1);
    }

    if(bitRead(modbusTCPClient3.holdingRegisterRead(26),0)){
      GRN_BTN_FLAG=true;AMB_BTN_FLAG=false;
      modbusTCPClient3.coilWrite(1,1);
      modbusTCPClient3.coilWrite(3,0);
   }
  }
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
