#include <Arduino.h>
#include <R050.h>

void R050::steamGenPID(){

  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  // PT304_TWV308_INPUT_PRESSURE_RAW = modbusTCPClient2.holdingRegisterRead(3023);//ref:43017 read Channel 7 PT304
  PT304_TWV308_INPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(6);// read Channel 6 PT304

  BLOWER_SPEED_OFFSET = STEAM_GEN_PID.calculate(170 , PT304_TWV308_INPUT_PRESSURE);
  Serial.print("BLOWER offset: "); Serial.println(BLOWER_SPEED_OFFSET);

  if (!modbusTCPClient1.connected()) {
    modbusTCPClient1.begin(serverIOEX1, 502);
  }
  modbusTCPClient1.holdingRegisterWrite(40, BLOWER_SPEED_OFFSET);

}
void R050::superheatTest(){

  if (!modbusTCPClient3.connected()) {
    modbusTCPClient3.begin(serverIOEX3, 502);
  }
  TT303_HX504_STEAM_OUT = (modbusTCPClient3.holdingRegisterRead(42)) * (0.095); //tt303
  TT306_EJECTOR_STEAM_IN = (modbusTCPClient3.holdingRegisterRead(43)) * (0.095); //tt306

  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  RO_PUMP_FEEDBACK = modbusTCPClient6.inputRegisterRead(1);//Ch1 ro water fb

  RO_SPEED_OFFSET = SUPER_HEAT_TT303.calc_reverse(130 , TT303_HX504_STEAM_OUT);

  if (!modbusTCPClient1.connected()) {
    modbusTCPClient1.begin(serverIOEX1, 502);
  }
  modbusTCPClient1.holdingRegisterWrite(41, RO_SPEED_OFFSET);//write channel 1 (WP_Speed)
  if (!modbusTCPClient7.connected()) {
    modbusTCPClient7.begin(serverIOEX7, 502);
  }
  modbusTCPClient7.coilWrite(10, ON); //WP_EN..Control is opposite

  Serial.print("RO_SPEED_OFFSET: "); Serial.println(RO_SPEED_OFFSET);
  Serial.print("TT303_HX504_STEAM_OUT: "); Serial.println(TT303_HX504_STEAM_OUT);
  Serial.print("TT306_EJECTOR_STEAM_IN: "); Serial.println(TT306_EJECTOR_STEAM_IN);

}
void R050::connect_IO_Expanders(){

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
void R050::readPTs(){

  Serial.println("Reading PTs");
  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  //checking PTs
  PT318_HX406_OUTPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(3);//read Channel 3 PressureTransducer_318.. 1psi/80counts maximux
  PT213_RO_PRESSURE = modbusTCPClient6.inputRegisterRead(4);// read Channel 4 PT213
  PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient6.inputRegisterRead(5);// read Channel 5 PT420
  PT304_TWV308_INPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(6);// read Channel 6 PT304

  Serial.print("PT318: "); Serial.println(PT318_HX406_OUTPUT_PRESSURE);
  Serial.print("PT213: "); Serial.println(PT213_RO_PRESSURE);
  Serial.print("PT420: "); Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
  Serial.print("PT304: "); Serial.println(PT304_TWV308_INPUT_PRESSURE);

  if (PT318_HX406_OUTPUT_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 3;
    Serial.print("PT318 >240PSI");
    Serial.println(PT318_HX406_OUTPUT_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if ( PT213_RO_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 4;
    Serial.print("PT213 >240PSI");
    Serial.println(PT213_RO_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if (PT420_STEAM_EJECTOR_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 5;
    Serial.print("PT420 >240PSI");
    Serial.println(PT420_STEAM_EJECTOR_PRESSURE);
  }
  else {
    ERROR = 0;
  }
  if (PT304_TWV308_INPUT_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 6;
    Serial.print("PT305 >240PSI");
    Serial.println(PT304_TWV308_INPUT_PRESSURE);
  }
  else {
    ERROR = 0;
  }

}
void R050::readTCs(){

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
        TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
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
          FSM_STATE = INITIALIZE;
          ERROR = 18;
        }
        if (TT408_HTS_IN_LREF > 1000) {
          Serial.println("TT408_HTS_IN_LREF > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 19;
        }

        /*
          Serial.print("TT142_SR_FUEL: ");Serial.println(TT142_SR_FUEL);
          Serial.print("TT301_HX406_STEAM_OUT : ");Serial.println(TT301_HX406_STEAM_OUT);
          Serial.print("TT303_HX504_STEAM_OUT : ");Serial.println(TT303_HX504_STEAM_OUT);
          Serial.print("TT306_EJECTOR_STEAM_IN : ");Serial.println(TT306_EJECTOR_STEAM_IN);
          Serial.print("TT313_HX402_STEAM_OUT : ");Serial.println(TT313_HX402_STEAM_OUT);
          Serial.print("TT319_HX402_STEAM_SYSTEM : ");Serial.println(TT319_HX402_STEAM_SYSTEM);
          Serial.print("TT407_STEAM_REFORMER_OUT_LREF : ");Serial.println(TT407_STEAM_REFORMER_OUT_LREF);
          Serial.print("TT408_HTS_IN_LREF : ");Serial.println(TT408_HTS_IN_LREF);
        */

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

        if (TT410_HTS_OUT_LREF > 1000) {
          Serial.println("TT410_HTS_OUT_LREF > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 20;
        }
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
        if (TT512_SILICON_CARBIDE_OUT > 1000) {
          Serial.println("TT512_SILICON_CARBIDE_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 24;
        }
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
          FSM_STATE = INITIALIZE;
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
        DB_TX();
        break;
      }

  }}
void R050::readOut(){
  lcd.setBacklight(HIGH);
  switch (READOUT_COUNTER) {

    case 1:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("P1");
        lcd.print((int)PT304_TWV308_INPUT_PRESSURE);//nine tubes
        lcd.setCursor(8, 0);
        lcd.print("P2");
        lcd.print((int)PT318_HX406_OUTPUT_PRESSURE);//nine tubes
        //  lcd.setCursor(14,0);
        //  lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("P3");
        lcd.print((int)PT420_STEAM_EJECTOR_PRESSURE);//nine tubes
        lcd.setCursor(8, 1);
        lcd.print("P4");
        lcd.print((int)PT213_RO_PRESSURE);//nine tubes
        READOUT_COUNTER = 2;
        PREVIOUS_MILLIS_4 = millis();
      }
    case 2:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S9");
        lcd.print((int)TT449_SMR_TUBE9_OUT);//nine tubes
        lcd.setCursor(8, 0);
        lcd.print("H1");
        lcd.print((int)TT511_SILICON_CARBIDE_OUT);//nine tubes
        //lcd.setCursor(14,0);
        //lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("H2");
        lcd.print((int)TT513_HX504_IN);//nine tubes
        lcd.setCursor(8, 1);
        lcd.print("SH");
        lcd.print((int)TT301_HX406_STEAM_OUT);//nine tubes
        READOUT_COUNTER = 3;
        PREVIOUS_MILLIS_4 = millis();
      }
    case 3:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.begin(16, 2);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S1");
        lcd.print((int)TT441_SMR_TUBE1_OUT);//nine tubes
        lcd.setCursor(8, 0);
        lcd.print("S2");
        lcd.print((int)TT442_SMR_TUBE2_OUT);//nine tubes
        //lcd.setCursor(14,0);
        //lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S3");
        lcd.print((int)TT443_SMR_TUBE3_OUT);//nine tubes
        lcd.setCursor(8, 1);
        lcd.print("S4");
        lcd.print((int)TT444_SMR_TUBE4_OUT);//nine tubes
        READOUT_COUNTER = 4;
        PREVIOUS_MILLIS_4 = millis();
      }
    case 4:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S5");
        lcd.print((int)TT445_SMR_TUBE5_OUT);//nine tubes
        lcd.setCursor(8, 0);
        lcd.print("S6");
        lcd.print((int)TT446_SMR_TUBE6_OUT);//nine tubes
        //lcd.setCursor(14,0);
        //lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S7");
        lcd.print((int)TT447_SMR_TUBE7_OUT);//nine tubes
        lcd.setCursor(8, 1);
        lcd.print("S8");
        lcd.print((int)TT448_SMR_TUBE8_OUT);//nine tubes
        READOUT_COUNTER = 1;
        PREVIOUS_MILLIS_4 = millis();
      }
  }

}
void R050::integrityCheck(){

  if (!SENSOR_INTEGRITY_CHECK) {
    Serial.println("Checking Signal Integrity.");
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

    PT318_HX406_OUTPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(3);// read Channel 3 PressureTransducer_318.. 1psi/80counts maximum
    PT213_RO_PRESSURE = modbusTCPClient6.inputRegisterRead(4);// read Channel 4 PT213
    PT420_STEAM_EJECTOR_PRESSURE = modbusTCPClient6.inputRegisterRead(5);//read Channel 5 PT420
    PT304_TWV308_INPUT_PRESSURE = modbusTCPClient6.inputRegisterRead(6);// read Channel 6 PT304

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
    modbusTCPClient7.coilWrite(9, 255); //BLWR_EN ON bc opposite
    modbusTCPClient7.coilWrite(10, 255); //WP_EN ON bc opposite
    modbusTCPClient7.coilWrite(6, OFF); //TWV308
    modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(7, OFF); //TWV901
    modbusTCPClient7.coilWrite(2, OFF); //xv909

    FCV134_BURNER_FUEL_FLOW_FB = modbusTCPClient6.inputRegisterRead(7);// read Channel 7 FCV134
    if (!(FCV134_BURNER_FUEL_FLOW_FB < 0.1)) {
      ERROR = 2;
      FSM_STATE = INITIALIZE;
      SENSOR_INTEGRITY_CHECK = false;
    }
    else {
      SENSOR_INTEGRITY_CHECK = true;
    }

    modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed) off

    //get that dang dynamic pressure switch off!!
    BLOWER_SPEED_FEEDBACK = modbusTCPClient6.holdingRegisterRead(0) * (.001); //10volts/10000counts
    while (BLOWER_SPEED_FEEDBACK > 0) {
      modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed) off
      BLOWER_SPEED_FEEDBACK = modbusTCPClient6.holdingRegisterRead(0) * (.001); //10volts/20000counts
      delay(1);
    }
  }
}
void R050::readOCI(){
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
    if (FSM_STATE > BMM_IGNITION && BMM_PROOF_OF_FLAME == false) {
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
void R050::readBtn(){
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
    }
    if (ESTOP_FLAG == false) {

      //amb button
      if (bitRead(DI_STATUS_CHANGE, 2)) {
        if (bitRead(CURRENT_DI_STATUS_WORD, 2) && FSM_STATE == STABILIZE_MODE) {
          AMB_BTN_FLAG = true;
          //modbusTCPClient7.coilWrite(0,0);//grn pilot light
          modbusTCPClient7.coilWrite(7, 1); //twv901 switch reformate to PSA
          modbusTCPClient7.coilWrite(1, 1); //amb pilot light ON
        }
        else {}
      }

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
void R050::error_Checker(){
  //may be redundant but for safety
  //shut gas down if an issue

  if (ERROR) {
    Serial.println("ERROR CHECKER TRIGGERED!");

    GRN_BTN_FLAG = false; AMB_BTN_FLAG = false; ESTOP_FLAG = true;

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
    modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(7, OFF); //TWV901
    modbusTCPClient7.coilWrite(2, ON); //CH4 digital output xv909


  }


}
void R050::steamPressureLow(){

}
void R050::monitor_SR_Tube_Temps(){


}
void R050::blinkGRN(){

  if (CURRENT_MILLIS - PREVIOUS_MILLIS_5 >= 1000 && GRN_BTN_FLAG == false) {
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
void R050::blinkAMB(){
  if (CURRENT_MILLIS - PREVIOUS_MILLIS_6 >= 1000 && AMB_BTN_FLAG == false) {
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
void R050::DB_RX(){
  regRX = telGetValue(TEL_ADDR, REGRX);
  if (regRX) {
    telWriteValue(TEL_ADDR, REGRX, 0x00);

    regRX = telGetValue(TEL_ADDR, REGRX);
    SUPERHEAT_TIMER = telGetValue(TEL_ADDR, SHTMR);
    BMM_OFF_TIMER = telGetValue(TEL_ADDR, BOTMR);
    BMM_START_TIMER = telGetValue(TEL_ADDR, BSTMR);
    BMM_PURGE_TIMER = telGetValue(TEL_ADDR, BPTMR);
    BMM_IGNITION_TIMER = telGetValue(TEL_ADDR, BCTMR);
    BURNER_RAMP_TIMER = telGetValue(TEL_ADDR, BRTMR);
    BURNER_REACH_END_TIMER = telGetValue(TEL_ADDR, BETMR);
    STEAM_GENERATION_TIMER = telGetValue(TEL_ADDR, SGTMR);
    STEAM_AT_170PSI_TIMER = telGetValue(TEL_ADDR, SPTMR);
    OPEN_SR_FUEL_TIMER = telGetValue(TEL_ADDR, SRTMR);
    BLOWER_PURGE_SPEED = telGetValue(TEL_ADDR, BLPSD);
    BLOWER_IGNITION_SPEED = telGetValue(TEL_ADDR, BLISD);
    BLOWER_RAMP_END = telGetValue(TEL_ADDR, BLEND);
    BLOWER_TOP_SPEED = telGetValue(TEL_ADDR, BLTSD);
    RO_PUMP_AT_10_GRAMS_PER_SEC = telGetValue(TEL_ADDR, WP10G);
    RO_PUMP_TOP_SPEED = telGetValue(TEL_ADDR, WPTSD);
    FCV205_AT_35_PERCENT = telGetValue(TEL_ADDR, F205S);
    FCV205_AT_50_PERCENT = telGetValue(TEL_ADDR, F205E);
    FCV134_BURNER_FUEL_FLOW_IGNITION = telGetValue(TEL_ADDR, BFIGN);
    FCV134_BURNER_FUEL_FLOW_RAMP_END = telGetValue(TEL_ADDR, BFRED);
    FCV141_SR_FUEL_START_PERCENT = telGetValue(TEL_ADDR, SRFST);
    FT132_PIPE_DIA_CONV = telGetValue(TEL_ADDR, FTDIA);
    FT132_COUNTS_TO_G_PER_SEC = telGetValue(TEL_ADDR, FTGPS);
    FT132_4MA_OFFSET = telGetValue(TEL_ADDR, FT4MA);
    BURNER_TEMP_RAMP_END = telGetValue(TEL_ADDR, BTRED);
    BURNER_TEMP_CROSSOVER = telGetValue(TEL_ADDR, BTCOV);
    SR_FUEL_CUT = telGetValue(TEL_ADDR, SRFCT);

  }

}
void R050::DB_TX(){

  //motor fb,flow fb,fcv position fb
  telWriteValue(TEL_ADDR, BL_FB, BLOWER_SPEED_FEEDBACK);
  telWriteValue(TEL_ADDR, WP_FB, RO_PUMP_FEEDBACK);
  telWriteValue(TEL_ADDR, BF_FB, FCV134_BURNER_FUEL_FLOW_FB);
  telWriteValue(TEL_ADDR, SR_FB, FT132_ADJUSTED_MEASURE);
  //pressure transducers
  telWriteValue(TEL_ADDR, PT213, PT213_RO_PRESSURE);
  telWriteValue(TEL_ADDR, PT318, PT318_HX406_OUTPUT_PRESSURE);
  telWriteValue(TEL_ADDR, PT420, PT420_STEAM_EJECTOR_PRESSURE);
  telWriteValue(TEL_ADDR, PT304, PT304_TWV308_INPUT_PRESSURE);
  //thermocouples
  telWriteValue(TEL_ADDR, TT142, TT142_SR_FUEL);
  telWriteValue(TEL_ADDR, TT301, TT301_HX406_STEAM_OUT);
  telWriteValue(TEL_ADDR, TT303, TT303_HX504_STEAM_OUT);
  telWriteValue(TEL_ADDR, TT306, TT306_EJECTOR_STEAM_IN);
  telWriteValue(TEL_ADDR, TT313, TT313_HX402_STEAM_OUT);
  telWriteValue(TEL_ADDR, TT319, TT319_HX402_STEAM_SYSTEM);
  telWriteValue(TEL_ADDR, TT407, TT407_STEAM_REFORMER_OUT_LREF);
  telWriteValue(TEL_ADDR, TT408, TT408_HTS_IN_LREF);
  telWriteValue(TEL_ADDR, TT410, TT410_HTS_OUT_LREF);
  telWriteValue(TEL_ADDR, TT411, TT411_FPZ_OUT_LREF);
  telWriteValue(TEL_ADDR, TT430, TT430_SMR_TUBES_INLET);
  telWriteValue(TEL_ADDR, TT511, TT511_SILICON_CARBIDE_OUT);
  telWriteValue(TEL_ADDR, TT512, TT512_SILICON_CARBIDE_OUT);
  telWriteValue(TEL_ADDR, TT513, TT513_HX504_IN);
  telWriteValue(TEL_ADDR, TT514, TT514_HX504_OUT);
  telWriteValue(TEL_ADDR, TT441, TT441_SMR_TUBE1_OUT);
  telWriteValue(TEL_ADDR, TT442, TT442_SMR_TUBE2_OUT);
  telWriteValue(TEL_ADDR, TT443, TT443_SMR_TUBE3_OUT);
  telWriteValue(TEL_ADDR, TT444, TT444_SMR_TUBE4_OUT);
  telWriteValue(TEL_ADDR, TT445, TT445_SMR_TUBE5_OUT);
  telWriteValue(TEL_ADDR, TT446, TT446_SMR_TUBE6_OUT);
  telWriteValue(TEL_ADDR, TT447, TT447_SMR_TUBE7_OUT);
  telWriteValue(TEL_ADDR, TT448, TT448_SMR_TUBE8_OUT);
  telWriteValue(TEL_ADDR, TT449, TT449_SMR_TUBE9_OUT);
  //oci to bmm interface
  telWriteValue(TEL_ADDR, OCIIN, OCI_INPUT_STATUS_WORD);
  telWriteValue(TEL_ADDR, OCIOT, OCI_OUTPUT_STATUS_WORD);

}
void R050::DB_INIT(){

  telWriteValue(TEL_ADDR, REGRX, regRX);
  telWriteValue(TEL_ADDR, SHTMR, SUPERHEAT_TIMER);
  telWriteValue(TEL_ADDR, BOTMR, BMM_OFF_TIMER);
  telWriteValue(TEL_ADDR, BSTMR, BMM_START_TIMER);
  telWriteValue(TEL_ADDR, BPTMR, BMM_PURGE_TIMER);
  telWriteValue(TEL_ADDR, BCTMR, BMM_IGNITION_TIMER);
  telWriteValue(TEL_ADDR, BRTMR, BURNER_RAMP_TIMER);
  telWriteValue(TEL_ADDR, BETMR, BURNER_REACH_END_TIMER);
  telWriteValue(TEL_ADDR, SGTMR, STEAM_GENERATION_TIMER);
  telWriteValue(TEL_ADDR, SPTMR, STEAM_AT_170PSI_TIMER);
  telWriteValue(TEL_ADDR, SRTMR, OPEN_SR_FUEL_TIMER);
  telWriteValue(TEL_ADDR, BLPSD, BLOWER_PURGE_SPEED);
  telWriteValue(TEL_ADDR, BLISD, BLOWER_IGNITION_SPEED);
  telWriteValue(TEL_ADDR, BLEND, BLOWER_RAMP_END);
  telWriteValue(TEL_ADDR, BLTSD, BLOWER_TOP_SPEED);
  telWriteValue(TEL_ADDR, WP10G, RO_PUMP_AT_10_GRAMS_PER_SEC);
  telWriteValue(TEL_ADDR, WPTSD, RO_PUMP_TOP_SPEED);
  telWriteValue(TEL_ADDR, F205S, FCV205_AT_35_PERCENT);
  telWriteValue(TEL_ADDR, F205E, FCV205_AT_50_PERCENT);
  telWriteValue(TEL_ADDR, BFIGN, FCV134_BURNER_FUEL_FLOW_IGNITION);
  telWriteValue(TEL_ADDR, BFRED, FCV134_BURNER_FUEL_FLOW_RAMP_END);
  telWriteValue(TEL_ADDR, SRFST, FCV141_SR_FUEL_START_PERCENT);
  telWriteValue(TEL_ADDR, FTDIA, FT132_PIPE_DIA_CONV);
  telWriteValue(TEL_ADDR, FTGPS, FT132_COUNTS_TO_G_PER_SEC);
  telWriteValue(TEL_ADDR, FT4MA, FT132_4MA_OFFSET);
  telWriteValue(TEL_ADDR, BTRED, BURNER_TEMP_RAMP_END);
  telWriteValue(TEL_ADDR, BTCOV, BURNER_TEMP_CROSSOVER);
  telWriteValue(TEL_ADDR, SRFCT, SR_FUEL_CUT);

}
