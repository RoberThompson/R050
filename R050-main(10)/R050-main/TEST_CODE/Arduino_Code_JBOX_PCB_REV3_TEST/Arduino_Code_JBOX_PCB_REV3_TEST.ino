//Had to change some of the digital outputs for the new PCB
#include<SPI.h>
#include<Ethernet.h>
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>
#include<ModbusMaster.h>
#include<EEPROM.h>

#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

int txGUI[50]={0,0,0,0,0,0,0,0,0,0,0,0,//tcs
               0,0,0,0,0,0,0,0,0,0,0,0,//tcs
               0,0,0,0,0,0,0,0,0,0,0,0,//analog inputs
               0,0,0,0,0,0,0,0,0,0,0,0,0,0};//these ones are DAOs

boolean dataBool=false;
boolean eepromBool=false;
char dataChar;

//Mac Address of controller
byte mac[] = { 0xdE, 0xAD, 0xBE, 0xEF, 0xED};

IPAddress ip(128,1,1,110);//needs to be similar to Acromag because will not connect

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

IPAddress serverA1(128,1,1,101); // IP of Acromag1 973EN-4006
IPAddress serverA2(128,1,1,102); // IP of Acromag2 963EN-4012
IPAddress serverA3(128,1,1,103); // IP of Acromag3 989EN-4016
IPAddress serverA4(128,1,1,104); // IP of Acromag4 965EN-4006
IPAddress serverA5(128,1,1,105); // IP of Acromag5 965EN-4006
IPAddress serverA6(128,1,1,106); // IP of Acromag6 965EN-4006
IPAddress serverA7(128,1,1,107); // IP of Acromag7 965EN-4006

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

  while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
  while(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
  while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
  while(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
  while(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
  while(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
  while(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}

  //assign EEPROM data to array
  for(int i=36;i<=48;i++){
    txGUI[i]=EEPROM.read(i-36);
    }

   //analogs
  modbusTCPClient1.holdingRegisterWrite(0x12,(txGUI[36])*2000);//write channel 0 (BLWRSpeed) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(0x13,(txGUI[37])*2000);//write channel 1 (WP_Speed) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(0x14,(txGUI[38])*2000);//write channel 2 (FCV134) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(0x15,(txGUI[39])*2000);//write channel 3 (FCV205) to 10 volts
  modbusTCPClient1.holdingRegisterWrite(0x16,(txGUI[40])*2000);//write channel 4 (FCV141) to 10 volts
   //digitals
  modbusTCPClient3.coilWrite(7,!(txGUI[41]*255));//XV801
  modbusTCPClient3.coilWrite(13,!(txGUI[42]*255));//BLWR_EN
  modbusTCPClient3.coilWrite(14,!(txGUI[43]*255));//WP_EN
  modbusTCPClient3.coilWrite(8,(txGUI[44]*255));//TWV308
  modbusTCPClient3.coilWrite(6,!(txGUI[45]*255));//XV1100
  modbusTCPClient3.coilWrite(5,!(txGUI[46]*255));//XV501
  modbusTCPClient3.coilWrite(12,(txGUI[47])*255);//BMM_CR2//! because control wired opposite
  modbusTCPClient3.coilWrite(9,(txGUI[48]*255));//TWV901
  modbusTCPClient3.coilWrite(4,(txGUI[49]*255));//xv909

}

void loop() {

  readOCI();
  GUI();

    //Simple function to write values to GUI
    Serial.write('@');
     for(int i=0;i<=49;i++){
      Serial.println(txGUI[i]);
    }

  GUI();

  if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
  txGUI[0]=modbusTCPClient4.holdingRegisterRead(3012)*(18.4/265);//IN0+ 0x000c..the 18.4/265 is for converting to celcius
  GUI();
  txGUI[1]=modbusTCPClient4.holdingRegisterRead(3013)*(18.4/265);//IN1+ 0x000d
  GUI();
  txGUI[2]=modbusTCPClient4.holdingRegisterRead(3014)*(18.4/265);//IN2+ 0x000e
  GUI();
  txGUI[3]=modbusTCPClient4.holdingRegisterRead(3015)*(18.4/265);//IN3+ 0x000f
  GUI();
  txGUI[4]=modbusTCPClient4.holdingRegisterRead(3016)*(18.4/265);//IN4+ 0x0010
  GUI();
  txGUI[5]=modbusTCPClient4.holdingRegisterRead(3017)*(18.4/265);//IN5+ 0x0010
  GUI();

  if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
  txGUI[6]=modbusTCPClient5.holdingRegisterRead(3012)*(18.4/265);//IN0+ 0x000c
  GUI();
  txGUI[7]=modbusTCPClient5.holdingRegisterRead(3013)*(18.4/265);//IN1+ 0x000d
  GUI();
  txGUI[8]=modbusTCPClient5.holdingRegisterRead(3014)*(18.4/265);//IN2+ 0x000e
  GUI();
  txGUI[9]=modbusTCPClient5.holdingRegisterRead(3015)*(18.4/265);//IN3+ 0x000f
  GUI();
  txGUI[10]=modbusTCPClient5.holdingRegisterRead(3016)*(18.4/265);//IN4+ 0x0010
  GUI();
  txGUI[11]=modbusTCPClient5.holdingRegisterRead(3017)*(18.4/265);//IN5+ 0x0010
  GUI();

  if(!modbusTCPClient6.connected()){modbusTCPClient6.begin(serverA6,502);}
  txGUI[12]=modbusTCPClient6.holdingRegisterRead(3012)*(18.4/265);//IN0+ 0x000c
  GUI();
  txGUI[13]=modbusTCPClient6.holdingRegisterRead(3013)*(18.4/265);//IN1+ 0x000d
  GUI();
  txGUI[14]=modbusTCPClient6.holdingRegisterRead(3014)*(18.4/265);//IN2+ 0x000e
  GUI();
  txGUI[15]=modbusTCPClient6.holdingRegisterRead(3015)*(18.4/265);//IN3+ 0x000f
  GUI();
  txGUI[16]=modbusTCPClient6.holdingRegisterRead(3016)*(18.4/265);//IN4+ 0x0010
  GUI();
  txGUI[17]=modbusTCPClient6.holdingRegisterRead(3017)*(18.4/265);//IN5+ 0x0010
  GUI();

  if(!modbusTCPClient7.connected()){modbusTCPClient7.begin(serverA7,502);}
  txGUI[18]=modbusTCPClient7.holdingRegisterRead(3012)*(18.4/265);//IN0+ 0x000c
  GUI();
  txGUI[19]=modbusTCPClient7.holdingRegisterRead(3013)*(18.4/265);//IN1+ 0x000d
  GUI();
  txGUI[20]=modbusTCPClient7.holdingRegisterRead(3014)*(18.4/265);//IN2+ 0x000e
  GUI();
  txGUI[21]=modbusTCPClient7.holdingRegisterRead(3015)*(18.4/265);//IN3+ 0x000f
  GUI();
  txGUI[22]=modbusTCPClient7.holdingRegisterRead(3016)*(18.4/265);//IN4+ 0x0010
  GUI();
  txGUI[23]=modbusTCPClient7.holdingRegisterRead(3017)*(18.4/265);//IN5+ 0x001
  GUI();

  if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
  txGUI[24] = modbusTCPClient2.holdingRegisterRead(3016)*(.005);//ref:43016 read Channel 0 BLWR508 analog input..10%/2000counts counts
  GUI();
  txGUI[25] = modbusTCPClient2.holdingRegisterRead(3017)*(.005);//ref:43017 read Channel 1 WP204 analog input
  GUI();
//  txGUI[26] = modbusTCPClient2.holdingRegisterRead(3018);//ref:43028 read Channel 2 //not used//was fcv134 on OLD PCB// moved to CH10
  txGUI[26]=COMBUSTION_PRESSURE_SWITCH;//for reading status on DPS
  GUI();
  txGUI[27] = modbusTCPClient2.holdingRegisterRead(3019);//ref:43028 read Channel 3 ft132
  GUI();
  txGUI[28] = modbusTCPClient2.holdingRegisterRead(3020)*(.0125)-50;//ref:43017 read Channel 4 PressureTransducer_318
  GUI();
  txGUI[29] = modbusTCPClient2.holdingRegisterRead(3021)*(.0125)-50;//ref:43028 read Channel 5 PT213
  GUI();
  txGUI[30] = modbusTCPClient2.holdingRegisterRead(3022)*(.0125)-47;//ref:43016 read Channel 6 PT420
  GUI();
  txGUI[31] = modbusTCPClient2.holdingRegisterRead(3023)*(.0125)-50;//ref:43017 read Channel 7 PT304
  GUI();
//  txGUI[32] = modbusTCPClient2.holdingRegisterRead(3024);//ref:43028 read Channel 8 DUN_PSH
  txGUI[32]=DUN_PSH;
  GUI();
  txGUI[33] = modbusTCPClient2.holdingRegisterRead(3025);//ref:43016 read Channel 9 psl
  GUI();
  txGUI[34] = (modbusTCPClient2.holdingRegisterRead(3026)*(.005))-20;//ref:43028 read Channel 10 FCV134..10%/5619counts-20 because 2-10v control
  GUI();
//  txGUI[35] = modbusTCPClient2.holdingRegisterRead(3027);//ref:43028 read Channel 11 DUN_ZSL
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
      EEPROM.update(12,txGUI[48]);
      EEPROM.update(13,txGUI[49]);
      }

   if(dataBool==true){

    dataChar=(char)Serial.read();

    switch(dataChar){

     case 'A':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
      txGUI[36]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(0x12,(txGUI[36])*2000);//write channel 0 (BLWRSpeed) to 10 volts
      break;

     case 'B':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
      txGUI[37]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(0x13,(txGUI[37])*2000);//write channel 1 (WP_Speed) to 10 volts
      break;

     case 'C':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
      txGUI[38]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(0x14,(txGUI[38])*2000);//write channel 2 (FCV134) to 10 volts
      break;

     case 'D':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
      txGUI[39]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(0x15,(txGUI[39])*2000);//write channel 0 (FCV205) to 10 volts

      break;
     case 'E':
      while(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
      txGUI[40]=Serial.read();
      modbusTCPClient1.holdingRegisterWrite(0x16,(txGUI[40])*2000);//write channel 0 (FCV141) to 10 volts
      break;

     case 'F':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[41]=Serial.read();
      modbusTCPClient3.coilWrite(7,!(txGUI[41]*255));//XV801
      break;

     case 'G':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[42]=Serial.read();
      modbusTCPClient3.coilWrite(13,!(txGUI[42]*255));//BLWR_EN
      break;

     case 'H':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[43]=Serial.read();
      modbusTCPClient3.coilWrite(14,!(txGUI[43]*255));//WP_EN
      break;

     case 'I':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[44]=Serial.read();
      modbusTCPClient3.coilWrite(8,(txGUI[44]*255));//TWV308
      break;

     case 'J':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[45]=Serial.read();
      modbusTCPClient3.coilWrite(6,!(txGUI[45]*255));//XV1100
      break;

     case 'K':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[46]=Serial.read();
      modbusTCPClient3.coilWrite(5,!(txGUI[46]*255));//XV501
      break;

     case 'L':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[47]=Serial.read();
      modbusTCPClient3.coilWrite(12,(txGUI[47])*255);//BMM_CR2//! because opposite for control
      break;

     case 'M':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[48]=Serial.read();
      modbusTCPClient3.coilWrite(9,(txGUI[48]*255));//TWV901
      break;

     case 'N':
      while(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
      txGUI[49]=Serial.read();
      modbusTCPClient3.coilWrite(4,(txGUI[49]*255));//XV909
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
