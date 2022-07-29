//Written by Mabel Elliot - 2/9/2022
//This is a program for configuring
//Modbus-TCP/IP to utilize
//Stride SIO-MB04DAS(4CH ANALOG OUTPUT)(192.168.1.101)
//STRIDE SIO-MB08THMS(8CH THERMOCOUPLE INPUT )(192.168.1.102)
//MOXA IOLOGIK E1211(16CH DIGITAL OUTPUT)(192.168.1.103)
//MOXA IOLOGIK E1240(8CH ANALOG INPUT)(192.168.1.104)
//MOXA IOLOGIK E1210(16CH DIGITAL INPUT)(192.168.1.105)
//with a Robodyn 2560 w/ Ethernet.
//All units are included the Robodyn 2560 is connected to an ethernet switch.
//All the IPs were changed from default to respective IPs stated above
//utilizing an internet browser (Mozilla Firefox).
//password  user and settings were changed for Strides to
//password1(both types) admin1(both types) 10v settings (analog output only)
//If not using a switch you must use a cross over cable for Strides
//for connection to Robodyn unsure about moxas.
//This works when used with a switch
//Can use P2P from moxas greatly reducing ethernet ports used on switch.
//

#include<Ethernet.h>
#include<ArduinoRS485.h> //ArduinoModbus depends on the ArduinoRS485 lib
#include<ArduinoModbus.h>

#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

byte mac[]={0xDE,0xAD, 0xBE, 0xEF, 0xED};//mac address of the Arduino

IPAddress ip(192,168,1,110);//IP of arduino needs to be similar to device connecting to.

EthernetClient ethClient1;//for modbusTCPClient1 so we dont have to stop connection each time
EthernetClient ethClient2;//to connect with another server.
EthernetClient ethClient3;
EthernetClient ethClient4;
EthernetClient ethClient5;

ModbusTCPClient modbusTCPClient1(ethClient1);//instance of ModbusTCPClient
ModbusTCPClient modbusTCPClient2(ethClient2);
ModbusTCPClient modbusTCPClient3(ethClient3);
ModbusTCPClient modbusTCPClient4(ethClient4);
ModbusTCPClient modbusTCPClient5(ethClient5);

IPAddress serverA1(192,168,1,101); //ip of stride SIO-MB04DAS
IPAddress serverA2(192,168,1,102); //ip of stride SIO-MB08THMS
IPAddress serverA3(192,168,1,103); //ip of moxa iologic E1211
IPAddress serverA4(192,168,1,104); //ip of moxa iologic E1240
IPAddress serverA5(192,168,1,105); //ip of moxa iologic E1210

void setup() {

  Serial.begin(9600);

  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
  pinMode(TRACO_24VDC, OUTPUT);
  digitalWrite(TRACO_24VDC, HIGH);
  pinMode(ESTOP_BREAK,OUTPUT);
  digitalWrite(ESTOP_BREAK, HIGH);

  pinMode(7,OUTPUT);//reseting wifichip
  digitalWrite(7,HIGH);
  delay(50);

  Ethernet.begin(mac, ip);

  //when not connected try to begin
  while(!modbusTCPClient1.connected()){
    Serial.println(". . . Connecting . . .");
    modbusTCPClient1.begin(serverA1,502);}
    Serial.println("Connected to Stride-SIO - MB04DAS - Analog Output !");

  //when not connected try to begin
  while(!modbusTCPClient2.connected()){
    Serial.println(". . . Connecting . . .");
    modbusTCPClient2.begin(serverA2,502);}
    Serial.println("Connected to Stride-SIO - MB08THMS - Thermocouple Input !");

  //when not connected try to begin
  while(!modbusTCPClient3.connected()){
    Serial.println(". . . Connecting . . .");
    modbusTCPClient3.begin(serverA3,502);}
    Serial.println("Connected to Moxa iologic e1211 - 16CH Digital Output !");

  //when not connected try to begin
  while(!modbusTCPClient4.connected()){
    Serial.println(". . . Connecting . . .");
    modbusTCPClient4.begin(serverA4,502);}
    Serial.println("Connected to Moxa iologic e1240 - 6CH Ananlog Input !");

  //when not connected try to begin
  while(!modbusTCPClient5.connected()){
    Serial.println(". . . Connecting . . .");
    modbusTCPClient5.begin(serverA5,502);}
    Serial.println("Connected to Moxa iologic e1210 - 16CH Digital Input !");

}

void loop() {

    if(!modbusTCPClient1.connected()){modbusTCPClient1.begin(serverA1,502);}
    modbusTCPClient1.holdingRegisterWrite(40,0);//Register 40041(Channel 0), 0 Volts
    delay(5000);
    modbusTCPClient1.holdingRegisterWrite(40,10000);//Register 40041(Channel 0), 10 Volts
    delay(5000);

    if(!modbusTCPClient2.connected()){modbusTCPClient2.begin(serverA2,502);}
    Serial.print("Thermocouple Reading CH1: ");Serial.println(modbusTCPClient2.holdingRegisterRead(40));
    Serial.print("Thermocouple Reading CH2: ");Serial.println(modbusTCPClient2.holdingRegisterRead(41));
    Serial.print("Thermocouple Reading CH3: ");Serial.println(modbusTCPClient2.holdingRegisterRead(42));
    Serial.print("Thermocouple Reading CH4: ");Serial.println(modbusTCPClient2.holdingRegisterRead(43));
    Serial.print("Thermocouple Reading CH5: ");Serial.println(modbusTCPClient2.holdingRegisterRead(44));
    Serial.print("Thermocouple Reading CH6: ");Serial.println(modbusTCPClient2.holdingRegisterRead(45));
    Serial.print("Thermocouple Reading CH7: ");Serial.println(modbusTCPClient2.holdingRegisterRead(46));
    Serial.print("Thermocouple Reading CH8: ");Serial.println(modbusTCPClient2.holdingRegisterRead(47));

    if(!modbusTCPClient3.connected()){modbusTCPClient3.begin(serverA3,502);}
    Serial.print("p2p safemode Status: ");Serial.println(modbusTCPClient3.inputRegisterRead(4112),BIN);
    modbusTCPClient3.coilWrite(1,1);
    Serial.print("Digital Output Status: ");Serial.println(modbusTCPClient3.holdingRegisterRead(32),BIN);
    delay(3000);
    modbusTCPClient3.coilWrite(1,0);
    Serial.print("Digital Output Status: ");Serial.println(modbusTCPClient3.holdingRegisterRead(32),BIN);

    if(!modbusTCPClient4.connected()){modbusTCPClient4.begin(serverA4,502);}
    modbusTCPClient4.holdingRegisterWrite(24,0);//Setting Analog input mode to 0-10VDC
    Serial.print("Analog Input CH0: ");Serial.println(modbusTCPClient4.inputRegisterRead(0));
    Serial.print("Analog Input CH1: ");Serial.println(modbusTCPClient4.inputRegisterRead(1));
    Serial.print("Analog Input CH2: ");Serial.println(modbusTCPClient4.inputRegisterRead(2));
    Serial.print("Analog Input CH3: ");Serial.println(modbusTCPClient4.inputRegisterRead(3));
    Serial.print("Analog Input CH4: ");Serial.println(modbusTCPClient4.inputRegisterRead(4));
    Serial.print("Analog Input CH5: ");Serial.println(modbusTCPClient4.inputRegisterRead(5));
    Serial.print("Analog Input CH6: ");Serial.println(modbusTCPClient4.inputRegisterRead(6));
    Serial.print("Analog Input CH7: ");Serial.println(modbusTCPClient4.inputRegisterRead(7));

    if(!modbusTCPClient5.connected()){modbusTCPClient5.begin(serverA5,502);}
    Serial.print("Digital Inputs: ");
    Serial.println(modbusTCPClient5.inputRegisterRead(48),BIN);//

}
