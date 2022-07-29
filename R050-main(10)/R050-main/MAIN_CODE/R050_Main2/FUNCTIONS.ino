void OTFOverrides() {

  if (blwrOverride) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(40, cmdSpeedBlower);
  }

  if (wpOverride) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(41, cmdSpeedWp);
  }

  if (fcv141Override) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
    }
    modbusTCPClient2.holdingRegisterWrite(40, cmdPercentFcv141); //write channel 4 (FCV141)
  }

  if (fcv205Override) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(43, cmdPercentFcv205 ); //fcv205cmdPercentFcv205
  }

  if (fcv134Override) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134 );
  }


}
void steamGenPID() {

  if (blwrOverride) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(40, cmdSpeedBlower);
  }


  else {
    //BLOWER_SPEED_OFFSET = STEAM_GEN_PID.calculate(PT304_SP_STEAMGEN , PT304_TWV308_INPUT_PRESSURE);

    cmdSpeedBlower = STEAM_GEN_PID.calculate(PT304_SP_STEAMGEN , PT304_TWV308_INPUT_PRESSURE);

    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(40, cmdSpeedBlower);
  }
}

void burnerFuelPID( int x ) {
  if (fcv134Override) {
    modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134 );
  }
  else {
    cmdPercentFcv134 = BURNER_FUEL_TEMP_CONTROLLER.calc_reverse(x, TT511_SILICON_CARBIDE_OUT);
    //BURNER_FUEL_CUT = BURNER_FUEL_TEMP_CONTROLLER.calc_reverse(x, TT511_SILICON_CARBIDE_OUT);

    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134 );//write channel 2 (FCV134)
  }
}

void SRFuelPID() {
  if (fcv141Override) {
    modbusTCPClient2.holdingRegisterWrite(40, cmdPercentFcv141); //write channel 4 (FCV141)
  }
  else {
    // SR_FUEL_OFFSET = OPEN_SR_FUEL_PID.calculate(SR_FUEL_CUT, FT132_NG_FEED_FLOW); //calc offset for FCV141
    cmdPercentFcv141 = OPEN_SR_FUEL_PID.calculate(SR_FUEL_CUT, FT132_NG_FEED_FLOW); //calc offset for FCV141

    if (!modbusTCPClient2.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
    }
    modbusTCPClient2.holdingRegisterWrite(40, cmdPercentFcv141); //write channel 4 (FCV141)

  }
}

void superheatTest() {

  if (wpOverride) {
    //modbusTCPClient1.holdingRegisterWrite(41, cmdSpeedWp);
  }

  else {
    if (abs( TT303_HX504_STEAM_OUT - TT306_EJECTOR_STEAM_IN) >= 20 && TT303_HX504_STEAM_OUT > 50
        || TT306_EJECTOR_STEAM_IN > 70) {

      cmdSpeedWp = SUPER_HEAT_TT303.calc_reverse( 170, TT303_HX504_STEAM_OUT); //calc to 170 degrees centigrade
      // RO_SPEED_OFFSET = SUPER_HEAT_TT303.calc_reverse( 170, TT303_HX504_STEAM_OUT);//calc to 170 degrees centigrade
      //FCV205_OFFSET = FCV205_PID.calc_reverse(180, TT303_HX504_STEAM_OUT);//this is used for tt408 tt410

      modbusTCPClient1.holdingRegisterWrite(41, cmdSpeedWp);//write channel 1 (WP_Speed)
      // modbusTCPClient1.holdingRegisterWrite(43,FCV205_OFFSET );//fcv205
      //modbusTCPClient7.coilWrite(10, ON); //WP_EN..Control is opposite
    }

    if (TT306_EJECTOR_STEAM_IN > 200 || TT303_HX504_STEAM_OUT > 200) {
      cmdSpeedWp = SUPER_HEAT_TT303.calc_reverse( 170, TT303_HX504_STEAM_OUT);//calc to 170 degrees centigrade
      //FCV205_OFFSET = FCV205_PID.calc_reverse(180, TT303_HX504_STEAM_OUT);//this is used for tt408 tt410

      modbusTCPClient1.holdingRegisterWrite(41, cmdSpeedWp);//write channel 1 (WP_Speed)
      // modbusTCPClient1.holdingRegisterWrite(43,FCV205_OFFSET );//fcv205
      // modbusTCPClient7.coilWrite(10, ON); //WP_EN..Control is opposite}
    }
  }
}

void reportConfigurabes() {
  //report volatiles to GUI upon startup and if there is a change
  //volatile(gui)
  //reporting
  // BURNER_TEMP_RAMP_END(tt511sp.ramp),BURNER_TEMP_CROSSOVER(tt511.sp.c/o), PT304_SP_STEAMGEN(pt304.sp),SUPERHEAT_TIMER(superheattimer),
  //BMM_OFF_TIMER(bmmofftimer), BMM_START_TIMER(bmmstarttimer), BMM_PURGE_TIMER(bmmpurge), BMM_IGNITION_TIMER(bmmignitiontimer), BURNER_REACH_END_TIMER(burnerreachendtimer)
  //STEAM_GENERATION_TIMER(steamgentimer), STEAM_AT_170PSI_TIMER(steampressuretimer), OPEN_SR_FUEL_TIMER(opensrfueltimer), SHUTDOWN_TIMER(shutdowntimer),
  //BLOWER_PURGE_SPEED(blowerpurgespeed), BLOWER_TOP_SPEED(blowertopspeed), RO_PUMP_AT_10_GRAMS_PER_SEC(wpspeed10gps), RO_PUMP_TOP_SPEED(wptopspeed),
  //FCV205_AT_35_PERCENT(fcv205min),FCV205_AT_50_PERCENT(fcv205Max), FCV134_BURNER_FUEL_FLOW_IGNITION(fcv134ignition), FCV134_BURNER_FUEL_FLOW_RAMP_END(fcv134rampend),
  //FCV134_BURNER_FUEL_FLOW_RAMP_BEGIN(fcv134rampbegin), FCV141__START_PERCENT(fcv141begin)



  if (firstRun) {

    for (int i = 0; i <= 23; i++) {

      Serial.write('#'); Serial.write(myChars[i]); Serial.println(oldConfigs[i]);
    }
    firstRun = false;
  }
}

void fcv205Control() {
  //tt410 watch for 340 degrees C
  //adjust fcv205 with slight kp gain.
  //need around 220
  if (fcv205Override) {
    modbusTCPClient1.holdingRegisterWrite(43, cmdPercentFcv205 ); //fcv205cmdPercentFcv205
  }

  else {
    if (TT410_HTS_OUT_LREF >= 340) {
      cmdPercentFcv205 = FCV205_PID.calculate(320, TT410_HTS_OUT_LREF);
      modbusTCPClient1.holdingRegisterWrite(43, cmdPercentFcv205 ); //fcv205
    }
    if (TT410_HTS_OUT_LREF <= 220 && FSM_STATE >= OPEN_SR_FUEL) {

      cmdPercentFcv205 = FCV205_AT_50_PERCENT;
      modbusTCPClient1.holdingRegisterWrite(43, cmdPercentFcv205 ); //fcv205
    }

    if (FSM_STATE < OPEN_SR_FUEL && FSM_STATE >= DEPRESSURIZE) {

      cmdPercentFcv205 = FCV205_AT_35_PERCENT;
      modbusTCPClient1.holdingRegisterWrite(43, cmdPercentFcv205 );
    }

  }

}

void connect_IO_Expanders() {

  //CONNECT TO ACROMAGS
  if (!modbusTCPClient1.connected()) {
    modbusTCPClient1.begin(serverIOEX1, 502);

  }
  //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex1  "));
  if (!modbusTCPClient2.connected()) {
    modbusTCPClient2.begin(serverIOEX2, 502);

  }
  //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex2  "));
  if (!modbusTCPClient3.connected()) {
    modbusTCPClient3.begin(serverIOEX3, 502);

  }
  //Serial.write('_'); Serial.write('!'); Serial.println("ioex3  ");
  if (!modbusTCPClient4.connected()) {
    modbusTCPClient4.begin(serverIOEX4, 502);

  }
  // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex4  "));
  if (!modbusTCPClient5.connected()) {
    modbusTCPClient5.begin(serverIOEX5, 502);

  }
  // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex5  "));
  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);

  }
  //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex6  "));
  if (!modbusTCPClient7.connected()) {
    modbusTCPClient7.begin(serverIOEX7, 502);

  }
  //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex7  "));
  if (!modbusTCPClient8.connected()) {
    modbusTCPClient8.begin(serverIOEX8, 502);

  }
  // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex8  "));
}
void readPTs() {

  if (!modbusTCPClient6.connected()) {
    modbusTCPClient6.begin(serverIOEX6, 502);
  }
  //checking PTs
  modbusTCPClient6.begin(serverIOEX6, 502);
  PT318_HX406_OUTPUT_PRESSURE = map(modbusTCPClient6.inputRegisterRead(3), 0, 65535, 0, 250); //read Channel 3 PressureTransducer_318.. 1psi/80counts maximux
  PT213_RO_PRESSURE = map(modbusTCPClient6.inputRegisterRead(4), 0, 65535, 0, 250); // read Channel 4 PT213
  PT420_STEAM_EJECTOR_PRESSURE = map( modbusTCPClient6.inputRegisterRead(5), 0, 65535, 0, 250); // read Channel 5 PT420
  PT304_TWV308_INPUT_PRESSURE = map(modbusTCPClient6.inputRegisterRead(6), 0, 65535, 0, 250); // read Channel 6 PT304

  txGUI[28] = PT318_HX406_OUTPUT_PRESSURE;
  Serial.write('_'); Serial.write(myChars[28]); Serial.println(txGUI[28]);
  txGUI[29] = PT213_RO_PRESSURE;
  Serial.write('_'); Serial.write(myChars[29]); Serial.println(txGUI[29]);
  txGUI[30] = PT420_STEAM_EJECTOR_PRESSURE;
  Serial.write('_'); Serial.write(myChars[30]); Serial.println(txGUI[30]);
  txGUI[31] = PT304_TWV308_INPUT_PRESSURE ;
  Serial.write('_'); Serial.write(myChars[31]); Serial.println(txGUI[31]);

  if (PT318_HX406_OUTPUT_PRESSURE >= 240) {
    FSM_STATE = INITIALIZE;
    ERROR = 3;
  }
  else {
    ERROR = 0;
  }
  if ( PT213_RO_PRESSURE >= 240) {
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

}
void readTCs() {

  /* if (!modbusTCPClient3.connected()) {
     modbusTCPClient3.begin(serverIOEX3, 502);
    }

    //read acromag 4
    TT142_SR_FUEL = modbusTCPClient3.holdingRegisterRead(40) * (0.095);
    TT301_HX406_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(41) * (0.095);
    TT303_HX504_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(42) * (0.095);
    TT306_EJECTOR_STEAM_IN = modbusTCPClient3.holdingRegisterRead(43) * (0.095);
    TT313_HX402_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(44) * (0.095);
    TT319_HX402_STEAM_SYSTEM = modbusTCPClient3.holdingRegisterRead(45) * (0.095);
    TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
    TT513_HX504_IN  = modbusTCPClient3.holdingRegisterRead(47) * (0.095);

    if (!modbusTCPClient4.connected()) {
     modbusTCPClient4.begin(serverIOEX4, 502);
    }

    TT410_HTS_OUT_LREF = modbusTCPClient4.holdingRegisterRead(40) * (0.095);
    TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(41) * (0.095);
    TT442_SMR_TUBE2_OUT = modbusTCPClient4.holdingRegisterRead(42) * (0.095);
    //
    TT512_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(44) * (0.095);
    TT408_HTS_IN_LREF  = modbusTCPClient4.holdingRegisterRead(45) * (0.095);
    TT514_HX504_OUT = modbusTCPClient4.holdingRegisterRead(46) * (0.095);
    TT411_FPZ_OUT_LREF = modbusTCPClient4.holdingRegisterRead(47) * (0.095);

    if (!modbusTCPClient5.connected()) {
     modbusTCPClient5.begin(serverIOEX5, 502);
    }

    TT430_SMR_TUBES_INLET = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
    TT511_SILICON_CARBIDE_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
    TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
    TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);
    TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
    TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
    TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
    TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);

    txGUI[0] = TT142_SR_FUEL;
    txGUI[1] = TT301_HX406_STEAM_OUT;
    txGUI[2] = TT303_HX504_STEAM_OUT;
    txGUI[3] = TT306_EJECTOR_STEAM_IN;
    txGUI[4] = TT313_HX402_STEAM_OUT;
    txGUI[5] = TT319_HX402_STEAM_SYSTEM;
    txGUI[6] = TT407_STEAM_REFORMER_OUT_LREF;
    txGUI[7] = TT408_HTS_IN_LREF;
    txGUI[8] = TT410_HTS_OUT_LREF;
    txGUI[9] = TT411_FPZ_OUT_LREF;
    txGUI[10] = TT430_SMR_TUBES_INLET;
    txGUI[11] = TT511_SILICON_CARBIDE_OUT;
    txGUI[12] = TT512_SILICON_CARBIDE_OUT;
    txGUI[13] = TT513_HX504_IN;
    txGUI[14] = TT514_HX504_OUT;
    txGUI[15] = TT441_SMR_TUBE1_OUT;
    txGUI[16] = TT442_SMR_TUBE2_OUT;
    txGUI[17] = TT443_SMR_TUBE3_OUT;
    txGUI[18] = TT444_SMR_TUBE4_OUT;
    txGUI[19] = TT445_SMR_TUBE5_OUT;
    txGUI[20] = TT446_SMR_TUBE6_OUT;
    txGUI[21] = TT447_SMR_TUBE7_OUT;
    txGUI[22] = TT448_SMR_TUBE8_OUT;
    txGUI[23] = TT449_SMR_TUBE9_OUT;*/

  switch (TC_CHECK_COUNTER) {
    case 1:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {

        if (!modbusTCPClient3.connected()) {
          modbusTCPClient3.begin(serverIOEX3, 502);
        }

        TT142_SR_FUEL = modbusTCPClient3.holdingRegisterRead(40) * (0.095);
        TT301_HX406_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(41) * (0.095);
        TT303_HX504_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(42) * (0.095);
        TT306_EJECTOR_STEAM_IN = modbusTCPClient3.holdingRegisterRead(43) * (0.095);

        txGUI[0] = TT142_SR_FUEL;
        Serial.write('_'); Serial.write(myChars[0]); Serial.println(txGUI[0]);
        txGUI[1] = TT301_HX406_STEAM_OUT;
        Serial.write('_'); Serial.write(myChars[1]); Serial.println(txGUI[1]);
        txGUI[2] = TT303_HX504_STEAM_OUT;
        Serial.write('_'); Serial.write(myChars[2]); Serial.println(txGUI[2]);
        txGUI[3] = TT306_EJECTOR_STEAM_IN;
        Serial.write('_'); Serial.write(myChars[3]); Serial.println(txGUI[3]);

        if (TT301_HX406_STEAM_OUT > 1350) {
          //Serial.println("TT301_HX406_STEAM_OUT > 1000C");
          // FSM_STATE = INITIALIZE;
          //  ERROR = 13;
        }
        if (TT303_HX504_STEAM_OUT > 1000) {
          // Serial.println("TT303_HX504_STEAM_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 14;
        }
        if (TT306_EJECTOR_STEAM_IN > 1000) {
          // Serial.println("TT306_EJECTOR_STEAM_IN > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 15;
        }
        if (TT313_HX402_STEAM_OUT > 1000) {
          // Serial.println("TT313_HX402_STEAM_OUT > 1000C");
        //  FSM_STATE = INITIALIZE;
          //ERROR = 16;
        }

        PREVIOUS_MILLIS_3 = millis();
        TC_CHECK_COUNTER = 2;
        break;

      }
    case 2:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {
        TT313_HX402_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(44) * (0.095);
        TT319_HX402_STEAM_SYSTEM = modbusTCPClient3.holdingRegisterRead(45) * (0.095);
        TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
        TT513_HX504_IN  = modbusTCPClient3.holdingRegisterRead(47) * (0.095);

        txGUI[4] = TT313_HX402_STEAM_OUT;
        Serial.write('_'); Serial.write(myChars[4]); Serial.println(txGUI[4]);
        txGUI[5] = TT319_HX402_STEAM_SYSTEM;
        Serial.write('_'); Serial.write(myChars[5]); Serial.println(txGUI[5]);
        txGUI[6] = TT407_STEAM_REFORMER_OUT_LREF;
        Serial.write('_'); Serial.write(myChars[6]); Serial.println(txGUI[6]);
        txGUI[12] = TT513_HX504_IN;
        Serial.write('_'); Serial.write(myChars[12]); Serial.println(txGUI[12]);

        if (TT319_HX402_STEAM_SYSTEM > 1000) {
          // Serial.println("TT319_HX402_STEAM_SYSTEM > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 17;
        }
        if (TT407_STEAM_REFORMER_OUT_LREF > 1350) {
          //  Serial.println("TT407_STEAM_REFORMER_OUT_LREF > 1000C");
          //  FSM_STATE = INITIALIZE;
          //   ERROR = 18;
        }
        if (TT408_HTS_IN_LREF > 1000) {
          //  Serial.println("TT408_HTS_IN_LREF > 1000C");
          //   FSM_STATE = INITIALIZE;
          // ERROR = 19;
        }

        PREVIOUS_MILLIS_3 = millis();
        TC_CHECK_COUNTER = 3;
        break;
      }

    case 3:
      //read acromag5
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {

        if (!modbusTCPClient4.connected()) {
          modbusTCPClient4.begin(serverIOEX4, 502);
        }

        TT410_HTS_OUT_LREF = modbusTCPClient4.holdingRegisterRead(40) * (0.095);
        TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(41) * (0.095);
        TT442_SMR_TUBE2_OUT = modbusTCPClient4.holdingRegisterRead(42) * (0.095);
        TT443_SMR_TUBE3_OUT = modbusTCPClient4.holdingRegisterRead(43) * (0.095);

        txGUI[8] = TT410_HTS_OUT_LREF;
        Serial.write('_'); Serial.write(myChars[8]); Serial.println(txGUI[8]);
        txGUI[15] = TT441_SMR_TUBE1_OUT;
        Serial.write('_'); Serial.write(myChars[15]); Serial.println(txGUI[15]);
        txGUI[16] = TT442_SMR_TUBE2_OUT;
        Serial.write('_'); Serial.write(myChars[16]); Serial.println(txGUI[16]);
        txGUI[13] = TT512_SILICON_CARBIDE_OUT;
        Serial.write('_'); Serial.write(myChars[13]); Serial.println(txGUI[13]);

        //     if (TT410_HTS_OUT_LREF > 1000) {
        //     Serial.println("TT410_HTS_OUT_LREF > 1000C");
        //   FSM_STATE = INITIALIZE;
        // ERROR = 20;
        //}
        if (TT411_FPZ_OUT_LREF > 1000) {
          //Serial.println("TT411_FPZ_OUT_LREF > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 21;
        }
        if (TT430_SMR_TUBES_INLET > 1000) {
          // Serial.println("TT430_SMR_TUBES_INLET > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 22;
        }
        if (TT511_SILICON_CARBIDE_OUT > 1200) {
          //  Serial.println("TT511_SILICON_CARBIDE_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 23;
        }

        TC_CHECK_COUNTER = 4;
        PREVIOUS_MILLIS_3 = millis();
        break;
      }

    case 4:
      //read acromag5
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {

        if (!modbusTCPClient4.connected()) {
          modbusTCPClient4.begin(serverIOEX4, 502);
        }

        TT512_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(44) * (0.095);
        TT408_HTS_IN_LREF  = modbusTCPClient4.holdingRegisterRead(45) * (0.095);
        TT514_HX504_OUT = modbusTCPClient4.holdingRegisterRead(46) * (0.095);
        TT411_FPZ_OUT_LREF = modbusTCPClient4.holdingRegisterRead(47) * (0.095);

        txGUI[17] = TT443_SMR_TUBE3_OUT;
        Serial.write('_'); Serial.write(myChars[17]); Serial.println(txGUI[17]);
        txGUI[7] = TT408_HTS_IN_LREF;
        Serial.write('_'); Serial.write(myChars[7]); Serial.println(txGUI[7]);
        txGUI[14] = TT514_HX504_OUT;
        Serial.write('_'); Serial.write(myChars[14]); Serial.println(txGUI[14]);
        txGUI[9] = TT411_FPZ_OUT_LREF;
        Serial.write('_'); Serial.write(myChars[9]); Serial.println(txGUI[9]);

        //     if (TT512_SILICON_CARBIDE_OUT > 1000) {
        //      Serial.println("TT512_SILICON_CARBIDE_OUT > 1000C");
        //    FSM_STATE = INITIALIZE;
        //  ERROR = 24;
        //}
        if (TT513_HX504_IN > 1200) {
          //  Serial.println("TT513_HX504_IN > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 25;
        }
        if (TT514_HX504_OUT > 1000) {
          //  Serial.println("TT514_HX504_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 26;
        }
        if (TT441_SMR_TUBE1_OUT > 800) {
          //  Serial.println("TT441_SMR_TUBE1_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 27;
        }

        TC_CHECK_COUNTER = 5;
        PREVIOUS_MILLIS_3 = millis();
        break;
      }

    case 5:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {

        if (!modbusTCPClient5.connected()) {
          modbusTCPClient5.begin(serverIOEX5, 502);
        }

        TT430_SMR_TUBES_INLET = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
        TT511_SILICON_CARBIDE_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
        TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
        TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);

        txGUI[10] = TT430_SMR_TUBES_INLET;
        Serial.write('_'); Serial.write(myChars[10]); Serial.println(txGUI[10]);
        txGUI[11] = TT511_SILICON_CARBIDE_OUT;
        Serial.write('_'); Serial.write(myChars[11]); Serial.println(txGUI[11]);
        txGUI[18] = TT444_SMR_TUBE4_OUT;
        Serial.write('_'); Serial.write(myChars[18]); Serial.println(txGUI[18]);
        txGUI[19] = TT445_SMR_TUBE5_OUT;
        Serial.write('_'); Serial.write(myChars[19]); Serial.println(txGUI[19]);

        if (TT442_SMR_TUBE2_OUT > 800) {
          // Serial.println("TT442_SMR_TUBE2_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 28;
        }
        if (TT443_SMR_TUBE3_OUT > 800) {
          //  Serial.println("TT443_SMR_TUBE3_OUT > 1000C");
          FSM_STATE = INITIALIZE;
          ERROR = 29;
        }
        if (TT444_SMR_TUBE4_OUT > 800) {
          // Serial.println("TT444_SMR_TUBE4_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 30;
        }
        if (TT445_SMR_TUBE5_OUT > 800) {
          // Serial.println("TT445_SMR_TUBE5_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 31;
        }

        TC_CHECK_COUNTER = 6;
        PREVIOUS_MILLIS_3 = millis();
        DB_TX();
        break;
      }

    case 6:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_3 >= 10) {

        if (!modbusTCPClient5.connected()) {
          modbusTCPClient5.begin(serverIOEX5, 502);
        }

        TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
        TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
        TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
        TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);

        txGUI[20] = TT446_SMR_TUBE6_OUT;
        Serial.write('_'); Serial.write(myChars[20]); Serial.println(txGUI[20]);
        txGUI[21] = TT447_SMR_TUBE7_OUT;
        Serial.write('_'); Serial.write(myChars[21]); Serial.println(txGUI[21]);
        txGUI[22] = TT448_SMR_TUBE8_OUT;
        Serial.write('_'); Serial.write(myChars[22]); Serial.println(txGUI[21]);
        txGUI[23] = TT449_SMR_TUBE9_OUT;
        Serial.write('_'); Serial.write(myChars[23]); Serial.println(txGUI[21]);


        if (TT446_SMR_TUBE6_OUT > 800) {
          // Serial.println("TT446_SMR_TUBE6_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 32;
        }
        if (TT447_SMR_TUBE7_OUT > 800) {
          //  Serial.println("TT447_SMR_TUBE7_OUT > 700C");
          FSM_STATE = INITIALIZE;
          ERROR = 33;
        }
        if (TT448_SMR_TUBE8_OUT > 800) {
          //  Serial.println("TT448_SMR_TUBE8_OUT > 700C");
          //  FSM_STATE = INITIALIZE;
          //ERROR = 34;
        }
        if (TT449_SMR_TUBE9_OUT > 800) {
          //  Serial.println("TT449_SMR_TUBE9_OUT > 700C");
          //      FSM_STATE = INITIALIZE;
          //    ERROR = 35;
        }

        TC_CHECK_COUNTER = 1;
        PREVIOUS_MILLIS_3 = millis();
        DB_TX();
        break;
      }

  }
}
/*void readOut() {
  lcd.setBacklight(HIGH);
  switch (READOUT_COUNTER) {

    case 1:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("P1.");
        lcd.print((int)PT304_TWV308_INPUT_PRESSURE);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("P2.");
        lcd.print((int)PT318_HX406_OUTPUT_PRESSURE);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("P3.");
        lcd.print((int)PT420_STEAM_EJECTOR_PRESSURE);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("P4.");
        lcd.print((int)PT213_RO_PRESSURE);//nine tubes
        READOUT_COUNTER = 2;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
    case 2:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S9.");
        lcd.print((int)TT449_SMR_TUBE9_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("H1.");
        lcd.print((int)TT511_SILICON_CARBIDE_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("H2.");
        lcd.print((int)TT513_HX504_IN);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("SH.");
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
        lcd.print("S1.");
        lcd.print((int)TT441_SMR_TUBE1_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("S2.");
        lcd.print((int)TT442_SMR_TUBE2_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S3.");
        lcd.print((int)TT443_SMR_TUBE3_OUT);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("S4.");
        lcd.print((int)TT444_SMR_TUBE4_OUT);//nine tubes
        READOUT_COUNTER = 4;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
    case 4:
      if (CURRENT_MILLIS - PREVIOUS_MILLIS_4 >= 3000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("S5.");
        lcd.print((int)TT445_SMR_TUBE5_OUT);//nine tubes
        lcd.setCursor(7, 0);
        lcd.print("S6.");
        lcd.print((int)TT446_SMR_TUBE6_OUT);//nine tubes
        lcd.setCursor(14, 0);
        lcd.print(FSM_STATE);
        lcd.setCursor(0, 1);
        lcd.print("S7.");
        lcd.print((int)TT447_SMR_TUBE7_OUT);//nine tubes
        lcd.setCursor(7, 1);
        lcd.print("S8.");
        lcd.print((int)TT448_SMR_TUBE8_OUT);//nine tubes
        READOUT_COUNTER = 1;
        PREVIOUS_MILLIS_4 = millis();
        break;
      }
  }

  }*/

void digitalOutputStatus() {

  digiOutStatus = modbusTCPClient7.holdingRegisterRead(32);
  Serial.write('-'); Serial.write('+'); Serial.println(digiOutStatus, BIN);

}

void integrityCheck() {


  if (!SENSOR_INTEGRITY_CHECK) {


    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
      // Serial.println("CONNECTING IOEX1.");
    }
    // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex1~    "));
    if (!modbusTCPClient2.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
      //  Serial.println("CONNECTING IOEX2.");
    }
    // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex2~  "));
    if (!modbusTCPClient3.connected()) {
      modbusTCPClient3.begin(serverIOEX3, 502);
      //  Serial.println("CONNECTING IOEX3.");
    }
    // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex3~  "));
    if (!modbusTCPClient4.connected()) {
      modbusTCPClient4.begin(serverIOEX4, 502);
      //  Serial.println("CONNECTING IOEX4.");
    }
    // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex4~  "));
    if (!modbusTCPClient5.connected()) {
      modbusTCPClient5.begin(serverIOEX5, 502);
      //  Serial.println("CONNECTING IOEX5.");
    }
    //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex5~  "));
    if (!modbusTCPClient7.connected()) {
      modbusTCPClient7.begin(serverIOEX7, 502);
      //  Serial.println("CONNECTING IOEX7.");
    }
    //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex7~  "));
    if (!modbusTCPClient8.connected()) {
      modbusTCPClient8.begin(serverIOEX8, 502);
      //  Serial.println("CONNECTING IOEX8.");
    }
    //Serial.write('_'); Serial.write('!'); Serial.println(F("ioex8~  "));
    if (!modbusTCPClient6.connected()) {
      modbusTCPClient6.begin(serverIOEX6, 502);
      //  Serial.println("CONNECTING IOEX6.");
    }
    // Serial.write('_'); Serial.write('!'); Serial.println(F("ioex6~  "));

    PT318_HX406_OUTPUT_PRESSURE = map(modbusTCPClient6.inputRegisterRead(3), 0, 65535, 0, 250); // read Channel 3 PressureTransducer_318.. 1psi/80counts maximum
    PT213_RO_PRESSURE = map( modbusTCPClient6.inputRegisterRead(4), 0, 65535, 0, 250 ); // read Channel 4 PT213
    PT420_STEAM_EJECTOR_PRESSURE = map(modbusTCPClient6.inputRegisterRead(5), 0, 65535, 0, 250 ); //read Channel 5 PT420
    PT304_TWV308_INPUT_PRESSURE = map(modbusTCPClient6.inputRegisterRead(6), 0, 65535, 0, 250); // read Channel 6 PT304

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
    //TT303_HX504_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(42) * (0.095);
    TT306_EJECTOR_STEAM_IN = modbusTCPClient3.holdingRegisterRead(43) * (0.095);
    TT313_HX402_STEAM_OUT = modbusTCPClient3.holdingRegisterRead(44) * (0.095);
    TT319_HX402_STEAM_SYSTEM = modbusTCPClient3.holdingRegisterRead(45) * (0.095);
    TT407_STEAM_REFORMER_OUT_LREF = modbusTCPClient3.holdingRegisterRead(46) * (0.095);
    TT513_HX504_IN = modbusTCPClient3.holdingRegisterRead(47) * (0.095);

    TT410_HTS_OUT_LREF = modbusTCPClient4.holdingRegisterRead(40) * (0.095);
    TT411_FPZ_OUT_LREF = modbusTCPClient4.holdingRegisterRead(41) * (0.095);
    TT430_SMR_TUBES_INLET = modbusTCPClient4.holdingRegisterRead(42) * (0.095);
    TT443_SMR_TUBE3_OUT = modbusTCPClient4.holdingRegisterRead(43) * (0.095);
    TT512_SILICON_CARBIDE_OUT = modbusTCPClient4.holdingRegisterRead(44) * (0.095);
    TT408_HTS_IN_LREF = modbusTCPClient4.holdingRegisterRead(45) * (0.095);
    TT514_HX504_OUT = modbusTCPClient4.holdingRegisterRead(46) * (0.095);
    TT441_SMR_TUBE1_OUT = modbusTCPClient4.holdingRegisterRead(47) * (0.095);

    TT442_SMR_TUBE2_OUT = modbusTCPClient5.holdingRegisterRead(40) * (0.095);
    TT511_SILICON_CARBIDE_OUT = modbusTCPClient5.holdingRegisterRead(41) * (0.095);
    TT444_SMR_TUBE4_OUT = modbusTCPClient5.holdingRegisterRead(42) * (0.095);
    TT445_SMR_TUBE5_OUT = modbusTCPClient5.holdingRegisterRead(43) * (0.095);
    TT446_SMR_TUBE6_OUT = modbusTCPClient5.holdingRegisterRead(44) * (0.095);
    TT447_SMR_TUBE7_OUT = modbusTCPClient5.holdingRegisterRead(45) * (0.095);
    TT448_SMR_TUBE8_OUT = modbusTCPClient5.holdingRegisterRead(46) * (0.095);
    TT449_SMR_TUBE9_OUT = modbusTCPClient5.holdingRegisterRead(47) * (0.095);

    txGUI[0] = TT142_SR_FUEL;
    txGUI[1] = TT301_HX406_STEAM_OUT;
    txGUI[2] = TT303_HX504_STEAM_OUT;
    txGUI[3] = TT306_EJECTOR_STEAM_IN;
    txGUI[4] = TT313_HX402_STEAM_OUT;
    txGUI[5] = TT319_HX402_STEAM_SYSTEM;
    txGUI[6] = TT407_STEAM_REFORMER_OUT_LREF;
    txGUI[7] = TT408_HTS_IN_LREF;
    txGUI[8] = TT410_HTS_OUT_LREF;
    txGUI[9] = TT411_FPZ_OUT_LREF;
    txGUI[10] = TT430_SMR_TUBES_INLET;
    txGUI[11] = TT511_SILICON_CARBIDE_OUT;
    txGUI[12] = TT512_SILICON_CARBIDE_OUT;
    txGUI[13] = TT513_HX504_IN;
    txGUI[14] = TT514_HX504_OUT;
    txGUI[15] = TT441_SMR_TUBE1_OUT;
    txGUI[16] = TT442_SMR_TUBE2_OUT;
    txGUI[17] = TT443_SMR_TUBE3_OUT;
    txGUI[18] = TT444_SMR_TUBE4_OUT;
    txGUI[19] = TT445_SMR_TUBE5_OUT;
    txGUI[20] = TT446_SMR_TUBE6_OUT;
    txGUI[21] = TT447_SMR_TUBE7_OUT;
    txGUI[22] = TT448_SMR_TUBE8_OUT;
    txGUI[23] = TT449_SMR_TUBE9_OUT;
    txGUI[24] = BLOWER_SPEED_FEEDBACK;
    txGUI[25] = RO_PUMP_FEEDBACK;
    txGUI[27] = FT132_NG_FEED_FLOW ;
    txGUI[28] = PT318_HX406_OUTPUT_PRESSURE;
    txGUI[29] = PT213_RO_PRESSURE;
    txGUI[30] = PT420_STEAM_EJECTOR_PRESSURE;
    txGUI[31] = PT304_TWV308_INPUT_PRESSURE;
    txGUI[34] = FCV134_BURNER_FUEL_FLOW_FB;


    //analog outputs
    modbusTCPClient1.holdingRegisterWrite(40, OFF); //write channel 0 (BLWRSpeed)
    modbusTCPClient1.holdingRegisterWrite(41, OFF); //write channel 1 (WP_Speed)
    modbusTCPClient1.holdingRegisterWrite(42, OFF); //write channel 2 (FCV134)
    modbusTCPClient1.holdingRegisterWrite(43, OFF); //write channel 3 (FCV205)
    modbusTCPClient2.holdingRegisterWrite(40, OFF); //write channel 0 (FCV141)

    //digital outputs
    modbusTCPClient7.coilWrite(5, OFF); //XV801
    modbusTCPClient7.coilWrite(9, OFF); //BLWR_EN ON bc opposite
    modbusTCPClient7.coilWrite(10, OFF); //WP_EN ON bc opposite
    modbusTCPClient7.coilWrite(6, OFF); //TWV308
    modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(7, OFF); //TWV901
    modbusTCPClient7.coilWrite(2, OFF); //xv909

    FCV134_BURNER_FUEL_FLOW_FB = map( modbusTCPClient6.inputRegisterRead(7), 100, 65535, 0, 100); // read Channel 7 FCV134
    if (!(FCV134_BURNER_FUEL_FLOW_FB < 1 )) {
      ERROR = 2;
      FSM_STATE = INITIALIZE;
      SENSOR_INTEGRITY_CHECK = false;
    }
    else {
      SENSOR_INTEGRITY_CHECK = true;
    }

    modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed) off

    //get that dang dynamic pressure switch off!!
    BLOWER_SPEED_FEEDBACK = map(modbusTCPClient6.holdingRegisterRead(0), 0, 65535, 0, 60); //10volts/10000counts
    while (BLOWER_SPEED_FEEDBACK > 1) {
      modbusTCPClient1.holdingRegisterWrite(40, 0); //write channel 0 (BLWRSpeed) off
      BLOWER_SPEED_FEEDBACK = map(modbusTCPClient6.holdingRegisterRead(0), 0, 65535, 0, 60 ); //10volts/20000counts
      delay(1);
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
    if (FSM_STATE > BURNER_RAMP && BMM_PROOF_OF_FLAME == false && SHUTDOWN_FLAG == false) {
      FSM_STATE = INITIALIZE;
      ERROR = 10;
    }
    BMM_ALARM_STATUS = bitRead(OCI_OUTPUT_STATUS_WORD, 1);
    if (FSM_STATE > BMM_IGNITION && BMM_ALARM_STATUS == true && SHUTDOWN_FLAG == false) {
      FSM_STATE = INITIALIZE;
      ERROR = 10;
    }
    //OCI_TO_BMM_COM=bitRead(OCI_OUTPUT_STATUS_WORD,2);
    //  if(FSM_STATE>6 && OCI_TO_BMM_COM==true){FSM_STATE=0;ERROR=10;}
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
  txGUI[33] = DUN_PSL;
  //Serial.write('_'); Serial.write('!'); Serial.print(F("dun.psl: ")); Serial.println(DUN_PSL);
  Serial.write('_'); Serial.write(myChars[33]); Serial.println(txGUI[33]);
  //  if (DUN_PSL) {
  //    ERROR = 36;
  //  }
  DUN_PSH = bitRead(CURRENT_DI_STATUS_WORD, 3);
  txGUI[32] = DUN_PSH;
  Serial.write('_'); Serial.write(myChars[32]); Serial.println(txGUI[32]);
  // if (!DUN_PSH) {
  //  ERROR = 37;
  // }
  DUN_ZSL = bitRead(CURRENT_DI_STATUS_WORD, 5);
  txGUI[35] = DUN_ZSL;
  Serial.write('_'); Serial.write(myChars[35]); Serial.println(txGUI[35]);
  // if ( DUN_ZSL && FSM_STATE > BMM_IGNITION) {
  //  ERROR = 38;
  //  }

  //estop


  if (bitRead(CURRENT_DI_STATUS_WORD, 0)) { //estop
    ESTOP_FLAG == false;
  }
  else {
    ESTOP_FLAG = true; GRN_BTN_FLAG = false; AMB_BTN_FLAG = false; ERROR = 1; FSM_STATE = INITIALIZE;

    modbusTCPClient7.coilWrite(0, OFF); //green pilot OFF
    modbusTCPClient7.coilWrite(1, OFF); //AMB pilot OFF
    modbusTCPClient7.coilWrite(2, OFF); //CH4 digital output xv909
    modbusTCPClient7.coilWrite(3, OFF); //XV501
    modbusTCPClient7.coilWrite(4, OFF); //XV1100
    modbusTCPClient7.coilWrite(5, OFF); //XV801
    modbusTCPClient7.coilWrite(6, OFF); //TWV308
    modbusTCPClient7.coilWrite(7, OFF); //TWV901
    modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
    modbusTCPClient7.coilWrite(9, OFF); //blwr_en
    modbusTCPClient7.coilWrite(10, OFF); //wp_en
  }

  DI_STATUS_CHANGE = CURRENT_DI_STATUS_WORD ^ LAST_DI_STATUS_WORD; //xor to check for a change in inputs
  //Serial.print("CURRENT_DI_STATUS_WORD: "); Serial.println(CURRENT_DI_STATUS_WORD);
  //Serial.print("DI_STATUS_CHANGE: "); Serial.println(DI_STATUS_CHANGE);
  LAST_DI_STATUS_WORD = CURRENT_DI_STATUS_WORD;


  if (DI_STATUS_CHANGE ) { //check for a change in inputs


    if (ESTOP_FLAG == false) {

      //amb button
      if (bitRead(DI_STATUS_CHANGE, 2)) {
        if (bitRead(CURRENT_DI_STATUS_WORD, 2) && FSM_STATE == STABILIZE_MODE) {

          if (AMB_BTN_FLAG) { //twv901 switch reformate to VENT
            AMB_BTN_FLAG = false;
            modbusTCPClient7.coilWrite(7, 0);
          }
          else {
            modbusTCPClient7.coilWrite(7, 1); //twv901 switch reformate to PSA
            modbusTCPClient7.coilWrite(1, 1); //amb pilot light ON
            AMB_BTN_FLAG = true;
          }
        }
      }


      //grn btn
      if (bitRead(DI_STATUS_CHANGE, 1)) {
        if (bitRead(CURRENT_DI_STATUS_WORD, 1) ) {
          if (GRN_BTN_FLAG) {
            GRN_BTN_FLAG = false;
            FSM_STATE = INITIALIZE;

            modbusTCPClient7.coilWrite(5, OFF); //XV801
            modbusTCPClient7.coilWrite(6, OFF); //TWV308
            modbusTCPClient7.coilWrite(4, OFF); //XV1100
            modbusTCPClient7.coilWrite(3, OFF); //XV501
            modbusTCPClient7.coilWrite(8, OFF); //BMM_CR2
            modbusTCPClient7.coilWrite(7, OFF); //TWV901
            modbusTCPClient7.coilWrite(2, ON); //CH4 digital output xv909

            modbusTCPClient1.holdingRegisterWrite(40, BLOWER_PURGE_SPEED); //write channel 0 (BLWRSpeed)
            modbusTCPClient1.holdingRegisterWrite(42, FCV205_AT_35_PERCENT); //write channel 2 (FCV205)//made steam at 1000//last 3000//~about 35 percent
            modbusTCPClient1.holdingRegisterWrite(41, RO_PUMP_AT_10_GRAMS_PER_SEC); //write channel 1 (WP_Speed) made steam at 1*2000//last 3*2000

            blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
            fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
            twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
            twv901Override = false; xv909Override = false;
            Serial.write('~'); Serial.write('~');
          }

          else {
            modbusTCPClient7.coilWrite(0, 1); GRN_BTN_FLAG = true ; //grn pilot
          }
        }
      }
    }
  }
}

void error_Checker() {
  //may be redundant but for safety
  //shut gas down if an issue

  if (ERROR) {
    // Serial.println("ERROR CHECKER TRIGGERED!");
    Serial.write('_'); Serial.write('?'); Serial.print(F("LAST.ERROR: ")); Serial.println(ERROR);
    blwrOverride = false; wpOverride = false; fcv134Override = false; fcv205Override = false;
    fcv141Override = false; xv801Override = false; blwrEnOverride = false; wpEnOverride = false;
    twv308Override = false; xv1100Override = false; xv501Override = false; bmmCr2Override = false;
    twv901Override = false; xv909Override = false;

    cmdPercentFcv134 = 0;
    cmdPercentFcv141 = 0;
    cmdXv801 = OFF ;
    cmdTwv308 = OFF;
    cmdXv1100 = OFF;
    cmdXv501 = OFF;
    cmdBmmCr2 = OFF;
    cmdTwv901 = OFF ;
    cmdXv909 = ON;

    //GRN_BTN_FLAG = false;
    AMB_BTN_FLAG = false; ESTOP_FLAG = true;

    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134); //write channel 2 (FCV134)
    // modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134); //write channel 2 (FCV134)

    if (!modbusTCPClient2.connected()) {
      modbusTCPClient2.begin(serverIOEX2, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(40, cmdPercentFcv141); //write channel 4 (FCV141)

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
    modbusTCPClient7.coilWrite(10, OFF); //WP_EN..
    modbusTCPClient7.coilWrite(9, OFF); //blwr_EN..
    // off xv801


  }
  else {
    ESTOP_FLAG = false;
  }


}

void steamPressureLow() {

}

void monitor_SR_Tube_Temps() {
  // if (TT441_SMR_TUBE1_OUT > 650 || TT442_SMR_TUBE2_OUT > 650 || TT443_SMR_TUBE3_OUT > 650
  //     || TT444_SMR_TUBE4_OUT > 650 || TT445_SMR_TUBE5_OUT > 650 || TT446_SMR_TUBE6_OUT > 650
  //     || TT447_SMR_TUBE7_OUT > 650 || TT448_SMR_TUBE8_OUT > 650 || TT449_SMR_TUBE9_OUT > 650) {
  //suplemental air


  // modbusTCPClient7.coilWrite(3, 1); //XV501

  // }
  if (fcv134Override) {
    if (!modbusTCPClient1.connected()) {
      modbusTCPClient1.begin(serverIOEX1, 502);
    }
    modbusTCPClient1.holdingRegisterWrite(42, cmdPercentFcv134 );
  }

  else {

    averageSRTemp = (TT441_SMR_TUBE1_OUT + TT442_SMR_TUBE2_OUT + TT443_SMR_TUBE3_OUT +
                     TT444_SMR_TUBE4_OUT + TT445_SMR_TUBE5_OUT + TT446_SMR_TUBE6_OUT +
                     TT447_SMR_TUBE7_OUT + TT448_SMR_TUBE8_OUT + TT449_SMR_TUBE9_OUT ) / 9;

    SrTubesOffset = monitorSRTubesPID.calc_reverse(650, averageSRTemp);
    modbusTCPClient1.holdingRegisterWrite(42, SrTubesOffset); //write channel 2 (FCV134)

  }

}

void blinkGRN() {

  if (CURRENT_MILLIS - PREVIOUS_MILLIS_5 >= 10 && GRN_BTN_FLAG == false && ESTOP_FLAG == false) {
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
  if (CURRENT_MILLIS - PREVIOUS_MILLIS_6 >= 10 && AMB_BTN_FLAG == false && ESTOP_FLAG == false) {
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

void DB_RX() {
  /*regRX = telGetValue(TEL_ADDR, REGRX);
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

    }*/

}

void DB_TX() {
  /*
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
    telWriteValue(TEL_ADDR, OCIOT, OCI_OUTPUT_STATUS_WORD);*/

}

void DB_INIT() {
  /*
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
  */
}

void readAI() {
  //function for reading
  //motor speed
  //fcv134fb ft132fb

  BLOWER_SPEED_FEEDBACK = map(modbusTCPClient6.inputRegisterRead(0), 0, 65535, 0, 60); //10volts/10000counts
  FCV134_BURNER_FUEL_FLOW_FB = map(modbusTCPClient6.inputRegisterRead(7), 100, 65535, 0, 100);
  RO_PUMP_FEEDBACK = map(modbusTCPClient6.inputRegisterRead(1), 0, 65535, 0, 60);
  FT132_NG_FEED_FLOW = map(modbusTCPClient6.inputRegisterRead(2), 0, 65535, 0, 3.5);
  //FT132_ADJUSTED_MEASURE = (FT132_NG_FEED_FLOW * FT132_PIPE_DIA_CONV * FT132_COUNTS_TO_G_PER_SEC) - FT132_4MA_OFFSET;

  txGUI[24] = BLOWER_SPEED_FEEDBACK;
  Serial.write('_'); Serial.write(myChars[24]); Serial.println(txGUI[24]);
  txGUI[25] = RO_PUMP_FEEDBACK;
  Serial.write('_'); Serial.write(myChars[25]); Serial.println(txGUI[25]);
  txGUI[27] = FT132_NG_FEED_FLOW ;
  Serial.write('_'); Serial.write(myChars[27]); Serial.println(txGUI[27]);
  txGUI[34] = FCV134_BURNER_FUEL_FLOW_FB;
  Serial.write('_'); Serial.write(myChars[34]); Serial.println(txGUI[34]);

}

/*void GUI() {

  for (int i = 0; i <= 35; i++) {
    // if (txGUI[i] != oldtxGUI[i] ) {
    //print new value to dashboard
    Serial.write('_');
    Serial.write(myChars[i]);
    Serial.println(txGUI[i]);
    oldtxGUI[i] = txGUI[i];
    //  }
  }
  }*/

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void serialEvent() {
  while (Serial.available()) {
    dataChar = (char)Serial.read();

    //System Shutdown
    if (dataChar == '<') {
      SHUTDOWN_FLAG = true;
      FSM_STATE = SHUTDOWN_MODE;
      PREVIOUS_MILLIS_8 = millis();

    }

    //for setting overrides
    if (dataChar == '=') {
      dataChar = (char)Serial.read();

      if (dataChar == 'A') {
        blwrOverride = false;
      }

      if (dataChar == 'B') {
        wpOverride = false;
      }

      if (dataChar == 'C') {
        fcv134Override = false;
      }

      if (dataChar == 'D') {//fcv205 percent
        fcv205Override = false;
      }

      if (dataChar == 'E') {//fcv141 percent
        fcv141Override = false;
      }
/*
      if (dataChar == 'F') {//xv801
        xv801Override = false;
      }

      if (dataChar == 'G') {//blwr_en
        blwrEnOverride = false;
      }

      if (dataChar == 'H') {//wp_en
        wpEnOverride = false;
      }

      if (dataChar == 'I') {//twv308
        twv308Override = false;
      }

      if (dataChar == 'J') {//xv1100
        xv1100Override = false;
      }

      if (dataChar == 'K') {
        xv501Override = false;
      }

      if (dataChar == 'L') {//bmm_cr2
        bmmCr2Override = false;
      }

      if (dataChar == 'M') {//twv901
        twv901Override = false;
      }

      if (dataChar == 'N') {//xv909
        xv909Override = false;
      }
*/
    }

    //for setting overrides
    if (dataChar == '*') {
      dataChar = (char)Serial.read();
      if (dataChar == 'A') {
        blwrOverride = true;
      }

      if (dataChar == 'B') {
        wpOverride = true;
      }

      if (dataChar == 'C') {
        fcv134Override = true;
      }

      if (dataChar == 'D') {//fcv205 percent
        fcv205Override = true;
      }

      if (dataChar == 'E') {//fcv141 percent
        fcv141Override = true;
      }
/*
      if (dataChar == 'F') {//xv801
        xv801Override = true;
      }

      if (dataChar == 'G') {//blwr_en
        blwrEnOverride = true;
      }

      if (dataChar == 'H') {//wp_en
        wpEnOverride = true;
      }

      if (dataChar == 'I') {//twv308
        twv308Override = true;
      }

      if (dataChar == 'J') {//xv1100
        xv1100Override = true;
      }

      if (dataChar == 'K') {
        xv501Override = true;
      }

      if (dataChar == 'L') {//bmm_cr2
        bmmCr2Override = true;
      }

      if (dataChar == 'M') {//twv901
        twv901Override = true;
      }

      if (dataChar == 'N') {//xv909
        xv909Override = true;
      }
*/
    }

    //on-the-fly controls
    if (dataChar == '_') {//blowr508
      dataChar = Serial.read();
      if (dataChar == 'A') {
        cmdSpeedBlower = map(Serial.read(), 0, 60, 0, 10000);
      }

      if (dataChar == 'B') {
        cmdSpeedWp = map(Serial.read(), 0, 60, 0, 10000);
      }

      if (dataChar == 'C') {
        cmdPercentFcv134 = map(Serial.read(), 20, 100, 2000, 10000);
      }

      if (dataChar == 'D') {//fcv205 percent
        cmdPercentFcv205 = map(Serial.read(), 0, 100, 0, 10000);
      }

      if (dataChar == 'E') {//fcv141 percent
        cmdPercentFcv141 = map(Serial.read(), 0, 100, 0, 10000);
      }
/*
      if (dataChar == 'F') {//xv801
        cmdXv801 = Serial.read();
      }

      if (dataChar == 'G') {//blwr_en
        cmdBlwrEn = Serial.read();
      }

      if (dataChar == 'H') {//wp_en
        cmdWpEn = Serial.read();
      }

      if (dataChar == 'I') {//twv308
        cmdTwv308 = Serial.read();
      }

      if (dataChar == 'J') {//xv1100
        cmdXv1100 = Serial.read();
      }

      if (dataChar == 'K') {
        cmdXv501 = Serial.read();
      }

      if (dataChar == 'L') {//bmm_cr2
        cmdBmmCr2 = Serial.read();
      }

      if (dataChar == 'M') {//twv901
        cmdTwv901 = Serial.read();
      }

      if (dataChar == 'N') {//xv909
        cmdXv909 = Serial.read();
      }

      */
    }

    //On-The_Fly configurables
    if (dataChar == '>') {
      dataChar = Serial.read();
      // Serial.write('_'); Serial.write('!'); Serial.print(F("000000000000 ")); Serial.println(BURNER_TEMP_RAMP_END);


      if (dataChar == 'A') {
        BURNER_TEMP_RAMP_END = map(Serial.read(), 0, 255, 0, 1100);
        Serial.write('#'); Serial.write(myChars[0]); Serial.println(BURNER_TEMP_RAMP_END);
      }
      if (dataChar == 'B') {
        BURNER_TEMP_CROSSOVER = map(Serial.read(), 0, 255, 0, 1100);
        Serial.write('#'); Serial.write(myChars[1]); Serial.println(BURNER_TEMP_CROSSOVER);
      }
      if (dataChar == 'C') {
        PT304_SP_STEAMGEN = map(Serial.read(), 0, 255, 0, 240);
        Serial.write('#'); Serial.write(myChars[2]); Serial.println(PT304_SP_STEAMGEN);
      }
      if (dataChar == 'D') {
        SUPERHEAT_TIMER = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[3]); Serial.println(SUPERHEAT_TIMER);
      }
      if (dataChar == 'E') {
        BMM_OFF_TIMER = map(Serial.read(), 0, 255, 0, 40000);
        Serial.write('#'); Serial.write(myChars[4]); Serial.println(BMM_OFF_TIMER);
      }
      if (dataChar == 'F') {
        BMM_START_TIMER = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[5]); Serial.println(BMM_START_TIMER);
      }
      if (dataChar == 'G') {
        BMM_PURGE_TIMER = map(Serial.read(), 0, 255, 0, 40000);
        Serial.write('#'); Serial.write(myChars[6]); Serial.println(BMM_PURGE_TIMER);
      }
      if (dataChar == 'H') {
        BMM_IGNITION_TIMER = map(Serial.read(), 0, 255, 0, 60000);
        Serial.write('#'); Serial.write(myChars[7]); Serial.println(BMM_IGNITION_TIMER);

      }
      if (dataChar == 'I') {
        BURNER_REACH_END_TIMER = map(Serial.read(), 0, 255, 0, 4000000);
        Serial.write('#'); Serial.write(myChars[8]); Serial.println(BURNER_REACH_END_TIMER);

      }
      if (dataChar == 'J') {
        STEAM_GENERATION_TIMER = map(Serial.read(), 0, 255, 0, 2000000);
        Serial.write('#'); Serial.write(myChars[9]); Serial.println(STEAM_GENERATION_TIMER);

      }
      if (dataChar == 'K') {
        STEAM_AT_170PSI_TIMER = map(Serial.read(), 0, 255, 0, 20000);
        Serial.write('#'); Serial.write(myChars[10]); Serial.println(STEAM_AT_170PSI_TIMER);

      }
      if (dataChar == 'L') {
        OPEN_SR_FUEL_TIMER = map(Serial.read(), 0, 255, 0, 2000000);
        Serial.write('#'); Serial.write(myChars[11]); Serial.println(OPEN_SR_FUEL_TIMER);

      }
      if (dataChar == 'M') {
        SHUTDOWN_TIMER = map(Serial.read(), 0, 255, 0, 2000000);
        Serial.write('#'); Serial.write(myChars[12]); Serial.println(SHUTDOWN_TIMER);

      }
      if (dataChar == 'N') {
        BLOWER_PURGE_SPEED = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[13]); Serial.println(BLOWER_PURGE_SPEED);

      }
      if (dataChar == 'O') {
        BLOWER_TOP_SPEED = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[14]); Serial.println(BLOWER_TOP_SPEED);

      }
      if (dataChar == 'P') {
        RO_PUMP_AT_10_GRAMS_PER_SEC = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[15]); Serial.println(RO_PUMP_AT_10_GRAMS_PER_SEC);

      }
      if (dataChar == 'Q') {
        RO_PUMP_TOP_SPEED = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[16]); Serial.println(RO_PUMP_TOP_SPEED);

      }
      if (dataChar == 'R') {
        FCV205_AT_50_PERCENT = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[17]); Serial.println(FCV205_AT_50_PERCENT);

      }
      if (dataChar == 'S') {
        FCV205_AT_35_PERCENT = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[18]); Serial.println(FCV205_AT_35_PERCENT);

      }
      if (dataChar == 'T') {
        FCV134_BURNER_FUEL_FLOW_IGNITION = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[19]); Serial.println(FCV134_BURNER_FUEL_FLOW_IGNITION);

      }
      if (dataChar == 'U') {
        FCV134_BURNER_FUEL_FLOW_RAMP_END = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[20]); Serial.println(FCV134_BURNER_FUEL_FLOW_RAMP_END);

      }
      if (dataChar == 'V') {
        FCV134_BURNER_FUEL_FLOW_RAMP_BEGIN = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[21]); Serial.println(FCV134_BURNER_FUEL_FLOW_RAMP_BEGIN);

      }
      if (dataChar == 'W') {
        FCV141_SR_FUEL_START_PERCENT = map(Serial.read(), 0, 255, 0, 10000);
        Serial.write('#'); Serial.write(myChars[21]); Serial.println(FCV141_SR_FUEL_START_PERCENT);

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
