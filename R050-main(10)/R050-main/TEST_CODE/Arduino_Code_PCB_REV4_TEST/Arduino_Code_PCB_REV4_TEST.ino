//this is for pcb revision4 with MOXAs/STRIDEs

#include<SPI.h>
#include<Ethernet.h>
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>
#include<ModbusMaster.h>
#include<EEPROM.h>

#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

//For OCI417 module
#define MAX485_DE 12
#define MAX485_RE_NEG 13
//OCI417 MODBUS PARAMETERS
const int OCI_INPUT_STATUS_REGISTER=25;
const int OCI_OUTPUT_STATUS_REGISTER=26;
const int OCI_UNIT_ID=1;
uint16_t OCI_INPUT_STATUS_WORD=0;
uint16_t OCI_OUTPUT_STATUS_WORD=0;
uint8_t OCI_RESULT;
uint16_t DATI[2];
ModbusMaster NODE;
uint8_t DUN_ZSL,DUN_PSH,DUN_PSL,COMBUSTION_PRESSURE_SWITCH;



long txGUI[49]={0,0,0,0,0,0,0,0,0,0,0,0,//tcs
               0,0,0,0,0,0,0,0,0,0,0,0,//tcs
               0,0,0,0,0,0,0,0,0,0,0,0,//analog inputs
               0,0,0,0,0,0,0,0,0,0,0,0,0};//these ones are DAOs

boolean dataBool=false;
boolean eepromBool=false;
char dataChar;

//Mac Address of controller
byte mac[] = { 0xdE, 0xAD, 0xBE, 0xEF, 0xED};

IPAddress ip(128,1,1,100);//needs to be similar to Acromag because will not connect

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


void setup() {

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial1.begin(9600);//FOR OCI MODULE
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

  pinMode(7,OUTPUT);//reseting wifichip
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

  //assign EEPROM data to array
  for(int i=36;i<=48;i++){
    txGUI[i]=EEPROM.read(i-36);
    }

   //analogs
  modbusTCPClient1.holdingRegisterWrite(40,(txGUI[36])*1000);//write channel 0 (BLWRSpeed) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(41,(txGUI[37])*1000);//write channel 1 (WP_Speed) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(42,(txGUI[38])*1000);//write channel 2 (FCV134) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(43,(txGUI[39])*1000);//write channel 3 (FCV205) to 10 volts
  modbusTCPClient2.holdingRegisterWrite(40,(txGUI[40])*1000);//write channel 4 (FCV141) to 10 volts
   //digitals
  modbusTCPClient7.coilWrite(5,(txGUI[41]));//XV801
  modbusTCPClient7.coilWrite(9,(txGUI[42]));//BLWR_EN
  modbusTCPClient7.coilWrite(10,(txGUI[43]));//WP_EN
  modbusTCPClient7.coilWrite(6,(txGUI[44]));//TWV308
  modbusTCPClient7.coilWrite(4,(txGUI[45]));//XV1100
  modbusTCPClient7.coilWrite(3,(txGUI[46]));//XV501
  modbusTCPClient7.coilWrite(8,(txGUI[47]));//BMM_CR2//! because control wired opposite
  modbusTCPClient7.coilWrite(7,(txGUI[48]));//TWV901
  //modbusTCPClient7.coilWrite(2,(txGUI[49]));//xv909//need to add a spot in array.

}

void loop() {

  readOCI();
  GUI();

    //Simple function to write values to GUI
    Serial.write('@');
     for(int i=0;i<=48;i++){
      Serial.println(txGUI[i]);
    }

  GUI();

  if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverIOEX3,502);}
  txGUI[0]=(modbusTCPClient3.holdingRegisterRead(40))*(0.095);//tt142the 0.095 is for converting to celcius
  GUI();
  txGUI[1]=(modbusTCPClient3.holdingRegisterRead(41))*(0.095);//tt301
  GUI();
  txGUI[2]=(modbusTCPClient3.holdingRegisterRead(42))*(0.095);//tt303
  GUI();
  txGUI[3]=(modbusTCPClient3.holdingRegisterRead(43))*(0.095);//tt306
  GUI();
  txGUI[4]=(holdingRegisterRead(44))*(0.095);//tt313
  GUI();
  txGUI[5]=(modbusTCPClient3.holdingRegisterRead(45))*(0.095);//tt319
  GUI();
  txGUI[6]=(modbusTCPClient3.holdingRegisterRead(46))*(0.095);//tt407
  GUI();
  txGUI[7]=(modbusTCPClient3.holdingRegisterRead(47))*(0.095);//tt408

  if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverIOEX4,502);}
  txGUI[8]=modbusTCPClient4.holdingRegisterRead(40)*(0.095);//tt410
  GUI();
  txGUI[9]=modbusTCPClient4.holdingRegisterRead(41)*(0.095);//tt411
  GUI();
  txGUI[10]=modbusTCPClient4.holdingRegisterRead(42)*(0.095);//tt430
  GUI();
  txGUI[11]=modbusTCPClient4.holdingRegisterRead(43)*(0.095);//tt511
  GUI();
  txGUI[12]=modbusTCPClient4.holdingRegisterRead(44)*(0.095);//tt512
  GUI();
  txGUI[13]=modbusTCPClient4.holdingRegisterRead(45)*(0.095);//tt513
  GUI();
  txGUI[14]=modbusTCPClient4.holdingRegisterRead(46)*(0.095);//tt514
  GUI();
  txGUI[15]=modbusTCPClient4.holdingRegisterRead(47)*(0.095);//tt441
  GUI();

  if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverIOEX5,502);}
  txGUI[16]=modbusTCPClient5.holdingRegisterRead(40)*(0.095);//tt442
  GUI();
  txGUI[17]=modbusTCPClient5.holdingRegisterRead(41)*(0.095);//tt443
  GUI();
  txGUI[18]=modbusTCPClient5.holdingRegisterRead(42)*(0.095);//tt444
  GUI();
  txGUI[19]=modbusTCPClient5.holdingRegisterRead(43)*(0.095);//tt445
  GUI();
  txGUI[20]=modbusTCPClient5.holdingRegisterRead(44)*(0.095);//tt446
  GUI();
  txGUI[21]=modbusTCPClient5.holdingRegisterRead(45)*(0.095);//tt447
  GUI();
  txGUI[22]=modbusTCPClient5.holdingRegisterRead(46)*(0.095);//tt448
  GUI();
  txGUI[23]=modbusTCPClient5.holdingRegisterRead(47)*(0.095);//tt449
  GUI();

  if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverIOEX6,502);}
  txGUI[24] = modbusTCPClient6.inputRegisterRead(0);// read Channel 0 BLWR508
  GUI();
  txGUI[25] = modbusTCPClient6.inputRegisterRead(1);//ref:43017 read Channel 1 WP204 analog input
  GUI();
  txGUI[26]=COMBUSTION_PRESSURE_SWITCH;//for reading status on DPS
  GUI();
  txGUI[27] = modbusTCPClient6.inputRegisterRead(2);//read Channel 2 // this is now ft132
  GUI();
  txGUI[28] = modbusTCPClient6.inputRegisterRead(3);//read Channel 3 PressureTransducer_318
  GUI();
  txGUI[29] = modbusTCPClient6.inputRegisterRead(4);//read Channel 4 PT213
  GUI();
  txGUI[30] = modbusTCPClient6.inputRegisterRead(5);//read Channel 5 PT420
  GUI();
  txGUI[31] = modbusTCPClient6.inputRegisterRead(6);// read Channel 6 PT304
  GUI();
  txGUI[32]=DUN_PSH;
  GUI();
  txGUI[33]=DUN_PSL;
  GUI();
  txGUI[34] = modbusTCPClient6.inputRegisterRead(7);//read Channel 7 FCV134..10%/5619counts-20 because 2-10v control
  GUI();
  txGUI[35]=DUN_ZSL;
  GUI();
}

void readOCI(){

  OCI_RESULT=NODE.readInputRegisters(OCI_INPUT_STATUS_REGISTER,1);//address,qty
  //do something with data if read is successful
  if (OCI_RESULT==NODE.ku8MBSuccess)
  {
    OCI_INPUT_STATUS_WORD = NODE.getResponseBuffer(0);
    DUN_ZSL = bitRead(OCI_INPUT_STATUS_WORD,1);//PROOF OF CLOSURE
    DUN_PSH = bitRead(OCI_INPUT_STATUS_WORD,4);//PRESS SW VALVE PROVING
    DUN_PSL = bitRead(OCI_INPUT_STATUS_WORD,5);//LOW GAS PRESSURE SWITCH
    COMBUSTION_PRESSURE_SWITCH = bitRead(OCI_INPUT_STATUS_WORD,7);//COMBUSTION AIR SW

  }

/*  OCI_RESULT=NODE.readInputregister(OCI_OUPUT_STATUS_REGISTER,1);
  if(OCI_RESULT==NODE.ku8MBSuccess)
  {
    OCI_OUTPUT_STATUS_WORD = NODE.getResponseBuffer(0);
    if(bitRead(OCI_OUTPUT_STATUS_WORD,0)){BMM_PROOF_OF_FLAME=true;}
    else{BMM_PROOF_OF_FLAME=false;}
    if (bitRead(OCI_OUTPUT_STATUS_WORD,1)) {BMM_ALARM_STATUS=true;}
    else{BMM_ALARM_STATUS=false;}
    if(bitRead(OCI_OUTPUT_STATUS_WORD,2)){OCI_TO_BMM_COM=true}
    else{OCI_TO_BMM_COM=false;}
  }*/
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

void GUI(){

  while(Serial.available()>0){
   dataChar=(char)Serial.read();

   if(dataChar=='@'){dataBool=true;}

   if(dataChar=='~'){
      EEPROM.update(0,txGUI[36]);
      EEPROM.update(1,txGUI[37]);
      EEPROM.update(2,txGUI[38]);
      EEPROM.update(3,txGUI[39]);
      EEPROM.update(4,txGUI[40]);
      EEPROM.update(5,txGUI[41]);
      EEPROM.update(6,txGUI[42]);
      EEPROM.update(7,txGUI[43]);
      EEPROM.update(8,txGUI[44]);
      EEPROM.update(9,txGUI[45]);
      EEPROM.update(10,txGUI[46]);
      EEPROM.update(11,txGUI[47]);
      EEPROM.update(12,txGUI[48]); }

   if(dataBool==true){

    dataChar=(char)Serial.read();

    switch(dataChar){

     case 'A':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
      txGUI[36]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(40,(txGUI[36])*1000);//write channel 0 (BLWRSpeed) to 10 volts
      break;

     case 'B':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
      txGUI[37]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(41,(txGUI[37])*1000);//write channel 1 (WP_Speed) to 10 volts
      break;

     case 'C':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX2,502);}
      txGUI[38]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(42,(txGUI[38])*1000);//write channel 2 (FCV134) to 10 volts
      break;

     case 'D':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverIOEX1,502);}
      txGUI[39]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(43,(txGUI[39])*1000);//write channel 3 (FCV205) to 10 volts

      break;
     case 'E':
      while(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverIOEX2,502);}
      txGUI[40]=Serial.read();
      modbusTCPClient2.holdingRegisterWrite(40,(txGUI[40])*1000);//write channel 0 (FCV141) to 10 volts
      break;

     case 'F':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[41]=Serial.read();
      if(txGUI[41]){txGUI[41]=1;}
      modbusTCPClient7.coilWrite(5,(txGUI[41]));//XV801
      break;

     case 'G':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[42]=Serial.read();
      if(txGUI[42]){txGUI[42]=1;}
      modbusTCPClient7.coilWrite(9,(txGUI[42]));//BLWR_EN
      break;

     case 'H':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[43]=Serial.read();
      if(txGUI[43]){txGUI[43]=1;}
      modbusTCPClient7.coilWrite(10,(txGUI[43]));//WP_EN
      break;

     case 'I':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[44]=Serial.read();
      if(txGUI[44]){txGUI[44]=1;}
      modbusTCPClient7.coilWrite(6,(txGUI[44]));//TWV308
      break;

     case 'J':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[45]=Serial.read();
      if(txGUI[45]){txGUI[45]=1;}
      modbusTCPClient7.coilWrite(3,(txGUI[45]));//XV1100
      break;

     case 'K':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[46]=Serial.read();
      if(txGUI[46]){txGUI[46]=1;}
      modbusTCPClient7.coilWrite(3,(txGUI[46]));//XV501
      break;

     case 'L':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[47]=Serial.read();
      if(txGUI[47]){txGUI[47]=1;}
      modbusTCPClient7.coilWrite(8,(txGUI[47]));//BMM_CR2//! because control wired opposite

      break;
     case 'M':
      while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverIOEX7,502);}
      txGUI[48]=Serial.read();
      if(txGUI[48]){txGUI[48]=1;}
      modbusTCPClient7.coilWrite(7,(txGUI[48]));//TWV901
      break;

     case '#':
        dataBool=false;
        break;

     default:
      break;
     }
    }
   }
  }
