import processing.serial.*;
import controlP5.*;
import java.util.*;

ControlP5 cp5;
Chart myChart;
CheckBox checkbox;

String[] myControllers = {"tC142(C)", "tC301(C)", "tC303(C)", "tC306(C)", "tC313(C)", "tC319(C)", "tC407(C)", "tC408(C)", "tC410(C)", "tC411(C)", "tC430(C)", "tC511(C)",
  "tC512(C)", "tC513(C)", "tC514(C)", "tC441(C)", "tC442(C)", "tC443(C)", "tC444(C)", "tC445(C)", "tC446(C)", "tC447(C)", "tC448(C)", "tC449(C)",
  "BL508(Hz)", "PMP204(Hz)", "ERROR", "FT132(units)", "PT318(PSI)", "PT213(PSI)", "PT420(PSI)", "PT304(PSI)", "DUN_PSH(t/f)", "DUN_PSL(t/f)", "FCV134(%)", "DUN_ZSL(t/f)",
  "BLWR_508(Hz)", "WP_204(Hz)", "FCV_134(%)", "FCV_205(%)", "FCV_141(%)", "XV801", "BLWR_EN", "WP_EN", "TWV308", "XV1100", "XV501", "BMM_CR2", "TWV901", "XV909",
  "FSM_STATE", "SERIAL_PORT","LAST_ERROR"
};

String[] myConfigs = {"tt511.sp.ramp", "tt511.sp.c/o", "pt304.sp",
  "S/H_TMR", "bmmOffTmr", "bmmStrtTmr", "bmmPrgTmr", "bmmIgnTmr", "bmmRampTmr",
  "brnReachEndTmr", "steamGenTmr", "steamPressSpTmr", "openSrFlTmr", "shtdnTmr",
  "blwrPrgSpd", "blwrTopSpd", "wpSpd10gps", "wpTopSpd", "fcv205Max", "fcv205Min",
  "fcv134Ign", "fcv134RampEnd", "fcv134RampBegin", "fcv141Begin"};

String[] myOverrides = {"O/R_BLWR", "O/R_WP", "O/R_FCV134", "O/R_FCV205", "O/R_FCV141",
  "O/R_XV801", "O/R_BL_EN", "O/R_WP_EN", "O/R_TWV308", "O/R_XV1100", "O/R_XV501", "O/R_BMM_CR2",
  "O/R_TWV901", "O/R_XV909"
};

char[] myChars = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
  'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
  'Y', 'Z'};

Serial myPort;

String dataString;
String dataString2 = "0";

int nl = 10;
int j;

float AVG_SR_TUBE_TEMP=0;
float[] SR_TUBE_TEMP= new float[9];


int intx = 29, inty = 22; //for spacing numberbox controllers
//int intx1 = 0, inty1 = 300; //for spacing switch controllers

int intx3 = 36, inty3 = 592;


boolean dataHeader = false;

boolean blwrOverride = false, wpOverride = false, fcv134Override = false, fcv205Override = false;
boolean fcv141Override = false, xv801Override = false, blwrEnOverride = false, wpEnOverride = false;
boolean twv308Override = false, xv1100Override = false, xv501Override = false, bmmCr2Override = false;
boolean twv901Override = false, xv909Override = false;

boolean[] boolOverrides = {blwrOverride, wpOverride, fcv134Override, fcv205Override, fcv141Override,
  xv801Override, blwrEnOverride, wpEnOverride, twv308Override, xv1100Override, xv501Override, bmmCr2Override,
  twv901Override, xv909Override};

void setup() {

 // size(3850, 2400);//windows xps computer
  size(1366,768);//lenovo computer
  smooth();
  noStroke();
  PFont font = createFont("arial", 8);

  cp5 = new ControlP5(this);
  cp5.setColorForeground(0xff00aaff);//when selected color rrggbb
  cp5.setColorBackground(0xff003366);
  cp5.setFont(font);
  cp5.setColorActive(0xff00aaff);

  String portName = Serial.list()[0];//array of serial ports connected changed from 0-1 because on port 9 not 8
  myPort = new Serial(this, portName, 9600);

  cp5.addBang("SHUTDOWN")
    .setPosition(1207, 16)
    .setColorForeground(0xffffd800)
    .setColorActive(0xffff0000)
    .setSize(126, 32);

  cp5.addScrollableList("CONFIG")
    .setPosition(36, 512)
    .setHeight(64)
    .setWidth(180)
    .setBarHeight(20)
    .setItemHeight(20)
    .addItems(myConfigs)
    //.setLabelVisible(false)
    // .setType(ScrollableList.LIST) // currently supported DROPDOWN and LIST
    ;

  cp5.addTextfield("VALUE")
    .setPosition(252, 512)
    .setSize(72, 20)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 0, 0))
    ;

  checkbox = cp5.addCheckBox("checkBox")
    .setPosition(612, 80)
    .setSize(13, 13)
    .setItemsPerRow(7)
    .setSpacingRow(32)
    .setSpacingColumn(40)
    .setSpacingRow(6)
    .addItem("TC.511", 0)
    .addItem("TC.512", 16)
    .addItem("SRT", 32)
    .addItem("PT.318", 48)
    .addItem("PT.213", 64)
    .addItem("PT.420", 80)
    .addItem("PT.304", 96)
    .addItem("TC.301", 0)
    .addItem("TC.303", 16)
    .addItem("TC.306", 32)
    .addItem("TC.410", 48)
    .addItem("TC.411", 64)
    .addItem("BL.SP", 80)
    .addItem("WP.SP", 96)
    ;

  myChart = cp5.addChart("dataflow")
    .setPosition(432, 128)
    .setSize(864, 265)
    .setRange(0, 1000)
    .setView(Chart.LINE)
    .setStrokeWeight(.2)
    .setColorBackground(color(0, 0, 0))
    ;

  //knobs
  int intx2 = 0, inty2 = 96; //for spacing knob controlers
  for (int i = 36; i < 41; i++) {
    if (i >= 38) {
      cp5.addKnob(myControllers[i])
        .setRange(0, 100)
        .setPosition(432 + intx2, 352 + inty2)
        .setRadius(43)
        .setColorForeground(0xffffd800)
        .setColorActive(0xffffd800)
        .setNumberOfTickMarks(100)
        .setTickMarkLength(6)
        .snapToTickMarks(true)
        .setDragDirection(Knob.HORIZONTAL)
        .setValue(0)
        ;
    } else {
      cp5.addKnob(myControllers[i])
        .setRange(0, 60)
        .setPosition(432 + intx2, 352 + inty2)
        .setRadius(43)
        .setColorForeground(0xffffd800)
        .setColorActive(0xffffd800)
        .setNumberOfTickMarks(60)
        .setTickMarkLength(6)
        .snapToTickMarks(true)
        .setDragDirection(Knob.HORIZONTAL)
        .setValue(0)
        ;
    }

    intx2 += 144;
  }

  //to display configurables current assigned values
  for (int i = 0; i<=23; i++) {
    cp5.addNumberbox(myConfigs[i])
      .setPosition(intx3, inty3)
      .setDecimalPrecision(0)
      .setSize(90, 20)
      ;
    intx3+=108;
    if (i==5 || i==11 || i==17 || i==23 ) {
      inty3+=38;
      intx3=36;
    }
  }

  //adding NumberBoxes for dispalying feedback numbers
  for (int i = 0; i <= 35; i++) {
    cp5.addNumberbox(myControllers[i])
      .setPosition(intx, inty)
      .setDecimalPrecision(0)
      //.setSize(210,32)//for home use
      .setSize(90, 20)
      ;
    //for home use
    inty += 40;
    if (i == 11 || i == 23 || i == 35) {
      intx += 126;
      inty = 22;
    }
  }

  intx=540;
  inty=22;
  //toggles

  int intx4 = 1152, inty4 = 448;//used for override toggles

  for (int i=0; i<=13; i++) {
    cp5.addToggle(myOverrides[i])
      .setPosition(intx4, inty4)
      .setColorForeground(0xffffd800)
      .setColorActive(0xffffd800)
      .setSize(68, 20)
      .setValue(0);
    inty4+=38;
    if (i==6) {
      intx4+=108;
      inty4=448;
    }
  }

  int intx1 = 0, inty1 = 96; //for spacing switch controllers
  int Toggle_offset = 756;//START OF CHECKBOXES
  for (int i = 41; i <= 49; i++) {
    cp5.addToggle(myControllers[i])
      .setPosition(Toggle_offset + intx1, 480 + inty1)
      .setColorForeground(0xffffd800)
      .setColorActive(0xffffd800)
      .setSize(68, 20)
      .setValue(0);
    ;

    intx1 += 108;
    if (i == 43 || i==46) {
      inty1 += 48;
      intx1 = 0;
      //Toggle_offset = 100;
    }
  }

  int intx5=432,inty5=20;
  //textField
  for (int i=50; i<=52; i++) {
    cp5.addTextfield(myControllers[i])
      .setPosition(intx5, inty5)
      .setSize(180, 20)
      .setFont(font)
      .setFocus(true)
      .setColor(color(255, 0, 0))
      ;

    intx5+=216;
  }

  myChart.addDataSet("TT511");
  myChart.setData("TT511", new float[400]);

  myChart.addDataSet("TT512");
  myChart.setData("TT512", new float[400]);

  myChart.addDataSet("SR_TUBES");
  myChart.setData("SR_TUBES", new float[400]);

  myChart.addDataSet("PT318");
  myChart.setData("PT318", new float[400]);

  myChart.addDataSet("PT213");
  myChart.setData("PT213", new float[400]);

  myChart.addDataSet("PT420");
  myChart.setData("PT420", new float[400]);

  myChart.addDataSet("PT304");
  myChart.setData("PT304", new float[400]);

  myChart.addDataSet("TT301");
  myChart.setData("TT301", new float[400]);

  myChart.addDataSet("TT303");
  myChart.setData("TT303", new float[400]);

  myChart.addDataSet("TT306");
  myChart.setData("TT306", new float[400]);

  myChart.addDataSet("TT410");
  myChart.setData("TT410", new float[400]);

  myChart.addDataSet("TT411");
  myChart.setData("TT411", new float[400]);

  myChart.addDataSet("BL.SP");
  myChart.setData("BL.SP", new float[400]);

  myChart.addDataSet("WP_SP");
  myChart.setData("WP_SP", new float[400]);

  checkbox.getItem(0).setColorActive(color(255, 0, 255));//511
  checkbox.getItem(1).setColorActive(color(255, 0, 0));//512
  checkbox.getItem(2).setColorActive(color(0, 255, 0));//sr_tubes
  checkbox.getItem(3).setColorActive(color(255, 255, 0));//pt318
  checkbox.getItem(4).setColorActive(color(0, 255, 255));//pt213
  checkbox.getItem(5).setColorActive(color(255, 255, 255));//pt420
  checkbox.getItem(6).setColorActive(color(100, 100, 255));//pt304
  checkbox.getItem(7).setColorActive(color(255, 100, 100));//tt301
  checkbox.getItem(8).setColorActive(color(200, 255, 0));//tt303
  checkbox.getItem(9).setColorActive(color(100, 100, 100));//tt306
  checkbox.getItem(10).setColorActive(color(200, 200, 200));//tt410
  checkbox.getItem(11).setColorActive(color(150, 150, 250));//tt411
  checkbox.getItem(12).setColorActive(color(200, 50, 200));//blwr
  checkbox.getItem(13).setColorActive(color(100, 200, 100));//wp

  //set values at startup
  checkbox.getItem(12).setValue(1);//blwr
  checkbox.getItem(13).setValue(1);//wp
  checkbox.getItem(6).setValue(1);//pt304
  checkbox.getItem(4).setValue(1);//pt213
}

void draw() {

  background(150);

  while (myPort.available() > 0) {

    int header1 = myPort.read();
    int header2 = myPort.read();

    if (header1 == '-') {
      if (header2 == '+') {
        dataString = myPort.readStringUntil(nl);

        if (dataString!=null) {
          println(dataString);

          if (dataString.length() == 2) {
            char[] ch = new char[dataString.length()];
            for (int i = 0; i<dataString.length(); i++) {
              ch[i] = dataString.charAt(i);
            }

            if (ch[2] == '1') {

              cp5.getController(myControllers[49]).setValue(1);
            } else {
              cp5.getController(myControllers[49]).setValue(0);
            }
          }

          if (dataString.length() > 8) {
            char[] ch = new char[dataString.length()];
            for (int i = 0; i<dataString.length(); i++) {
              ch[i] = dataString.charAt(i);
            }

            if (ch[8]=='1') {//xv909
              cp5.getController(myControllers[49]).setValue(1);
            } else {
              cp5.getController(myControllers[49]).setValue(0);
            }

            if (ch[7]=='1') {//xv501
              cp5.getController(myControllers[46]).setValue(1);
            } else {
              cp5.getController(myControllers[46]).setValue(0);
            }

            if (ch[6]=='1') {//xv1100
              cp5.getController(myControllers[45]).setValue(1);
            } else {
              cp5.getController(myControllers[45]).setValue(0);
            }

            if (ch[5]=='1') {//xv801
              cp5.getController(myControllers[41]).setValue(1);
            } else {
              cp5.getController(myControllers[41]).setValue(0);
            }

            if (ch[4]=='1') {//xvtwv308
              cp5.getController(myControllers[44]).setValue(1);
            } else {
              cp5.getController(myControllers[44]).setValue(0);
            }

            if (ch[3]=='1') {//xvtwv901
              cp5.getController(myControllers[48]).setValue(1);
            } else {
              cp5.getController(myControllers[48]).setValue(0);
            }

            if (ch[2]=='1') {//bmm_cr2
              cp5.getController(myControllers[47]).setValue(1);
            } else {
              cp5.getController(myControllers[47]).setValue(0);
            }

            if (ch[1]=='1') {//blwr_en
              cp5.getController(myControllers[42]).setValue(1);
            } else {
              cp5.getController(myControllers[42]).setValue(0);
            }

            if (ch[0]=='1') {//wp_en
              cp5.getController(myControllers[43]).setValue(1);
            } else {
              cp5.getController(myControllers[43]).setValue(0);
            }
          }
        }
      }
    }

    if (header1=='~') {
      if (header2=='~') {
        for (int i = 0; i<=13; i++) {
          cp5.getController(myOverrides[i]).setValue(0);
        }
      }
    }

    if (header1 == '#') {
      if (header2 == 'A') {
        //tt511spramp
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[0]).setValue(float(dataString));
        }
      }
      if (header2 == 'B') {
        //tt511spc/o
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[1]).setValue(float(dataString));
        }
      }
      if (header2 == 'C') {
        //pt304sp
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[2]).setValue(float(dataString));
        }
      }

      if (header2 == 'D') {
        //superheattimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[3]).setValue(float(dataString));
        }
      }

      if (header2 == 'E') {
        //bmmofftimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[4]).setValue(float(dataString));
        }
      }

      if (header2 == 'F') {
        //bmmstarttimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[5]).setValue(float(dataString));
        }
      }

      if (header2 == 'G') {
        //bmmpurgetimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[6]).setValue(float(dataString));
        }
      }

      if (header2 == 'H') {
        //bmmignitiontimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[7]).setValue(float(dataString));
        }
      }

      if (header2 == 'I') {
        //burnerreachendtimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[8]).setValue(float(dataString));
        }
      }

      if (header2 == 'J') {
        //steamgentimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[9]).setValue(float(dataString));
        }
      }

      if (header2 == 'K') {
        //steampressuresptimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[10]).setValue(float(dataString));
        }
      }

      if (header2 == 'L') {
        //opensrfueltimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[11]).setValue(float(dataString));
        }
      }

      if (header2 == 'M') {
        //shutdowntimer
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[12]).setValue(float(dataString));
        }
      }

      if (header2 == 'N') {
        //blowerpurgespeed
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[13]).setValue(float(dataString));
        }
      }

      if (header2 == 'O') {
        //BLowertopspeed
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[14]).setValue(float(dataString));
        }
      }

      if (header2 == 'P') {
        //wpspeed10gps
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[15]).setValue(float(dataString));
        }
      }

      if (header2 == 'Q') {
        //wptopspeed
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[16]).setValue(float(dataString));
        }
      }

      if (header2 == 'R') {
        //fcv205max
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[17]).setValue(float(dataString));
        }
      }

      if (header2 == 'S') {
        //fcv205min
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[18]).setValue(float(dataString));
        }
      }

      if (header2 == 'T') {
        //fcv134ignition
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[19]).setValue(float(dataString));
        }
      }

      if (header2 == 'U') {
        //fcv134RampEnd
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[20]).setValue(float(dataString));
        }
      }

      if (header2 == 'V') {
        //fcv134RampBegin
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[21]).setValue(float(dataString));
        }
      }

      if (header2 == 'W') {
        //fcv141Begin
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myConfigs[22]).setValue(float(dataString));
        }
      }
    }

    if (header1=='_') {
      if (header2 == 'A') {  //142
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[0]).setValue(float(dataString));
        }
      }
      if (header2 == 'B') {//301
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[1]).setValue(float(dataString));
          myChart.push("TT301", float(dataString));
        }
      }
      if (header2 == 'C') {//303
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[2]).setValue(float(dataString));
          myChart.push("TT303", float(dataString));
        }
      }
      if (header2 == 'D') {//306
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[3]).setValue(float(dataString));
          myChart.push("TT306", float(dataString));
        }
      }
      if (header2 == 'E') {//313
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[4]).setValue(float(dataString));
        }
      }
      if (header2 == 'F') {//319
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[5]).setValue(float(dataString));
        }
      }
      if (header2 == 'G') {//407
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[6]).setValue(float(dataString));
        }
      }

      if (header2 == 'H') {//408
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[7]).setValue(float(dataString));
        }
      }
      if (header2 == 'I') {//410
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[8]).setValue(float(dataString));
          myChart.push("TT410", float(dataString));
        }
      }
      if (header2 == 'J') {//411
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[9]).setValue(float(dataString));
          myChart.push("TT411", float(dataString));
        }
      }
      if (header2 == 'K') {//430
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[10]).setValue(float(dataString));
        }
      }
      if (header2 == 'L') {//511
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[11]).setValue(float(dataString));
          myChart.push("TT511", float(dataString));
        }
      }
      if (header2 == 'M') {//512
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[12]).setValue(float(dataString));
        }
      }
      if (header2 == 'N') {//513
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[13]).setValue(float(dataString));
          myChart.push("TT512", float(dataString));
        }
      }
      if (header2 == 'O') {//514
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[14]).setValue(float(dataString));
        }
      }
      if (header2 == 'P') {//441
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[0]=float(dataString);
          cp5.getController(myControllers[15]).setValue(float(dataString));
        }
      }
      if (header2 == 'Q') {//442
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[1]=float(dataString);
          cp5.getController(myControllers[16]).setValue(float(dataString));
        }
      }
      if (header2 == 'R') {//443
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[2]=float(dataString);
          cp5.getController(myControllers[17]).setValue(float(dataString));
        }
      }
      if (header2 == 'S') {//444
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[3]=float(dataString);
          cp5.getController(myControllers[18]).setValue(float(dataString));
        }
      }
      if (header2 == 'T') {//445
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[4]=float(dataString);
          cp5.getController(myControllers[19]).setValue(float(dataString));
        }
      }
      if (header2 == 'U') {//446
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[5]=float(dataString);
          cp5.getController(myControllers[20]).setValue(float(dataString));
        }
      }
      if (header2 == 'V') {//447
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[6]=float(dataString);
          cp5.getController(myControllers[21]).setValue(float(dataString));
        }
      }
      if (header2 == 'W') {//448
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[7]=float(dataString);
          cp5.getController(myControllers[22]).setValue(float(dataString));
        }
      }
      if (header2 == 'X') {//449
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          SR_TUBE_TEMP[8]=float(dataString);
          cp5.getController(myControllers[23]).setValue(float(dataString));
        }
      }
      if (header2 == 'Y') {//blwr_fb, blwr_508(Hz)
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[24]).setValue(float(dataString));
          if (!boolOverrides[0]) {
            cp5.getController(myControllers[36]).setValue(float(dataString));
          }
          myChart.push("BL.SP", float(dataString));
        }
      }
      if (header2 == 'Z') {//wp_fb
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[25]).setValue(float(dataString));
          if (!boolOverrides[1]) {
            cp5.getController(myControllers[37]).setValue(float(dataString));
          }
          myChart.push("WP_SP", float(dataString));
        }
      }
      if (header2 == 'a') {//error
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[26]).setValue(float(dataString));
        }
      }
      if (header2 == 'b') {//ft132
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[27]).setValue(float(dataString));
        }
      }
      if (header2 == 'c') {//pt318
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[28]).setValue(float(dataString));
          myChart.push("PT318", float(dataString));
        }
      }
      if (header2 == 'd') {//pt213
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[29]).setValue(float(dataString));
          myChart.push("PT213", float(dataString));
        }
      }
      if (header2 == 'e') {//pt420
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[30]).setValue(float(dataString));
          myChart.push("PT420", float(dataString));
        }
      }
      if (header2 == 'f') {//pt304
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[31]).setValue(float(dataString));
          myChart.push("PT304", float(dataString));
        }
      }
      if (header2 == 'g') {//dun_psh
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[32]).setValue(float(dataString));
        }
      }
      if (header2 == 'h') {//dun_psl
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[33]).setValue(float(dataString));
        }
      }
      if (header2 == 'i') {//fcv134
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[34]).setValue(float(dataString));
          if (!boolOverrides[2]) {
            cp5.getController(myControllers[38]).setValue(float(dataString));
          }
        }
      }
      if (header2 == 'j') {//dun_zsl
        dataString = myPort.readStringUntil(nl);
        if (dataString != null) {
          cp5.getController(myControllers[35]).setValue(float(dataString));
        }
      }

      if (header2 == '@') {
        dataString2 = myPort.readStringUntil(nl);
        if (dataString2 != null) {
          cp5.get(Textfield.class, myControllers[50]).setText(dataString2);
          myPort.clear();
        }
      }

      if (header2 == '!') {
        dataString2 = myPort.readStringUntil(nl);
        if (dataString2 != null) {
          cp5.get(Textfield.class, myControllers[51]).setText(dataString2);
          myPort.clear();
        }
      }

      if (header2 == '?') {
        dataString2 = myPort.readStringUntil(nl);
        if (dataString2 != null) {
          cp5.get(Textfield.class, myControllers[52]).setText(dataString2);
          myPort.clear();
        }
      }
    }//if header
  }//while serial

  for (int i=0; i<=8; i++) {
    AVG_SR_TUBE_TEMP+=SR_TUBE_TEMP[i];
  }
  AVG_SR_TUBE_TEMP/=9;


  myChart.push("SR_TUBES", AVG_SR_TUBE_TEMP);


  if (checkbox.getItem(0).getState()==true) {

    myChart.setColors("TT511", color(255, 0, 255));
  } else {
    myChart.setColors("TT511", color(0, 0, 0));
  }

  if (checkbox.getItem(1).getState()==true) {

    myChart.setColors("TT512", color(255, 0, 0));
  } else {

    myChart.setColors("TT512", color(0, 0, 0));
  }

  if (checkbox.getItem(2).getState()==true) {

    myChart.setColors("SR_TUBES", color(0, 255, 0));
  } else {

    myChart.setColors("SR_TUBES", color(0, 0, 0));
  }

  if (checkbox.getItem(3).getState()==true) {

    myChart.setColors("PT318", color(255, 255, 0));
  } else {

    myChart.setColors("PT318", color(0, 0, 0));
  }

  if (checkbox.getItem(4).getState()==true) {

    myChart.setColors("PT213", color(0, 255, 255));
  } else {

    myChart.setColors("PT213", color(0, 0, 0));
  }

  if (checkbox.getItem(5).getState()==true) {

    myChart.setColors("PT420", color(255, 255, 255));
  } else {

    myChart.setColors("PT420", color(0, 0, 0));
  }

  if (checkbox.getItem(6).getState()==true) {

    myChart.setColors("PT304", color(100, 100, 255));
  } else {

    myChart.setColors("PT304", color(0, 0, 0));
  }

  if (checkbox.getItem(7).getState()==true) {

    myChart.setColors("TT301", color(255, 100, 100));
  } else {

    myChart.setColors("TT301", color(0, 0, 0));
  }

  if (checkbox.getItem(8).getState()==true) {

    myChart.setColors("TT303", color(200, 255, 100));
  } else {

    myChart.setColors("TT303", color(0, 0, 0));
  }

  if (checkbox.getItem(9).getState()==true) {

    myChart.setColors("TT306", color(100, 100, 100));
  } else {

    myChart.setColors("TT306", color(0, 0, 0));
  }

  if (checkbox.getItem(10).getState()==true) {

    myChart.setColors("TT410", color(200, 200, 200));
  } else {

    myChart.setColors("TT410", color(0, 0, 0));
  }

  if (checkbox.getItem(11).getState()==true) {

    myChart.setColors("TT411", color(150, 150, 250));
  } else {

    myChart.setColors("TT411", color(0, 0, 0));
  }

  if (checkbox.getItem(12).getState()==true) {

    myChart.setColors("BL.SP", color(200, 50, 200));
  } else {

    myChart.setColors("BL.SP", color(0, 0, 0));
  }

  if (checkbox.getItem(13).getState()==true) {

    myChart.setColors("WP_SP", color(100, 200, 100));
  } else {

    myChart.setColors("WP_SP", color(0, 0, 0));
  }

  if (//pts only on
    checkbox.getItem(0).getState()==false && checkbox.getItem(1).getState()==false &&
    checkbox.getItem(2).getState()==false && checkbox.getItem(3).getState() &&
    checkbox.getItem(4).getState() && checkbox.getItem(5).getState() &&
    checkbox.getItem(6).getState() )
  {
    myChart.setRange(0, 250);
  }

  if ( //only blower and wp selected
    checkbox.getItem(0).getState()==false && checkbox.getItem(1).getState()==false &&
    checkbox.getItem(2).getState()==false && checkbox.getItem(3).getState()==false &&
    checkbox.getItem(4).getState()==false && checkbox.getItem(5).getState()==false &&
    checkbox.getItem(6).getState()==false && checkbox.getItem(7).getState()==false &&
    checkbox.getItem(8).getState()==false && checkbox.getItem(9).getState()==false &&
    checkbox.getItem(10).getState()==false && checkbox.getItem(11).getState()==false &&
    checkbox.getItem(12).getState() && checkbox.getItem(13).getState())
  {
    myChart.setRange(0, 60);
  }

  if (  //if heater is not selected and something else is
    checkbox.getItem(0).getState() ==false && checkbox.getItem(1).getState()==false &&
    checkbox.getItem(2).getState()==false || checkbox.getItem(3).getState() ||
    checkbox.getItem(4).getState() || checkbox.getItem(5).getState() ||
    checkbox.getItem(6).getState() || checkbox.getItem(7).getState() ||
    checkbox.getItem(8).getState() || checkbox.getItem(9).getState() ||
    checkbox.getItem(10).getState() || checkbox.getItem(11).getState() ||
    checkbox.getItem(12).getState() || checkbox.getItem(13).getState()) {

    myChart.setRange(0, 250);
  } else {
    myChart.setRange(0, 900);
  }
}//draw


void controlEvent(ControlEvent theEvent) {
  //println(theEvent);
  if (theEvent.isController()) {

    for (int i=0; i<=13; i++) {
      if (theEvent.getController().getName() == myOverrides[i]) {
        if (cp5.getController(myOverrides[i]).getValue()>=1) {
          boolOverrides[i] = true;
          myPort.write('*');
          myPort.write(myChars[i]);
        } else {
          boolOverrides[i] = false;
          myPort.write('=');
          myPort.write(myChars[i]);
        }
      }
    }

    for (int i=0; i<=13; i++) {
      if (theEvent.getController().getName()==myControllers[36+i]) {
        if (boolOverrides[i]) {
          myPort.write('_');
          myPort.write(myChars[i]);
          myPort.write(int(cp5.getController(myControllers[36+i]).getValue()));
          println(theEvent);
        }
      }
    }

    if (theEvent.getController().getName()=="SHUTDOWN") {
      myPort.write('<');
    }
  }

  if (theEvent.isAssignableFrom(Textfield.class)) {
    if (theEvent.getController().getName()=="VALUE") {


      float foo =cp5.getController("CONFIG").getValue();
      myPort.write('>');

      if (int(foo)==0) {//tt511spramp
        float val = map(float(theEvent.getStringValue()), 0, 1100, 0, 255);
        myPort.write(myChars[0]);
        myPort.write(int(val));
      }
      if (int(foo)==1) {//tt511spc/o
        float val = map(float(theEvent.getStringValue()), 0, 1100, 0, 255);
        myPort.write(myChars[1]);
        myPort.write(int(val));
      }

      if (int(foo)==2) {//pt304sp
        float val = map(float(theEvent.getStringValue()), 0, 240, 0, 255);
        myPort.write(myChars[2]);
        myPort.write(int(val));
      }

      if (int(foo)==3) {//superheat timer
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[3]);
        myPort.write(int(val));
      }

      if (int(foo)==4) {//bmmofftimer
        float val = map(float(theEvent.getStringValue()), 0, 40000, 0, 255);
        myPort.write(myChars[4]);
        myPort.write(int(val));
      }
      if (int(foo)==5) {//bmmstarttimer
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[5]);
        myPort.write(int(val));
      }
      if (int(foo)==6) {//bmmpurgetimer
        float val = map(float(theEvent.getStringValue()), 0, 40000, 0, 255);
        myPort.write(myChars[6]);
        myPort.write(int(val));
      }
      if (int(foo)==7) {//bmmignitiontimer
        float val = map(float(theEvent.getStringValue()), 0, 60000, 0, 255);
        myPort.write(myChars[7]);
        myPort.write(int(val));
      }

      if (int(foo)==8) {//burnerreachendtimer
        float val = map(float(theEvent.getStringValue()), 0, 60000, 0, 255);
        myPort.write(myChars[8]);
        myPort.write(int(val));
      }

      if (int(foo)==9) {//steamgentimer
        float val = map(float(theEvent.getStringValue()), 0, 2000000, 0, 255);
        myPort.write(myChars[9]);
        myPort.write(int(val));
      }

      if (int(foo)==10) {//steampressuresptimer
        float val = map(float(theEvent.getStringValue()), 0, 20000, 0, 255);
        myPort.write(myChars[10]);
        myPort.write(int(val));
      }

      if (int(foo)==11) {//opensrfueltimer
        float val = map(float(theEvent.getStringValue()), 0, 2000000, 0, 255);
        myPort.write(myChars[11]);
        myPort.write(int(val));
      }

      if (int(foo)==12) {//shutdowntimer
        float val = map(float(theEvent.getStringValue()), 0, 2000000, 0, 255);
        myPort.write(myChars[12]);
        myPort.write(int(val));
      }

      if (int(foo)==13) {//blowerpurgespeed
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[13]);
        myPort.write(int(val));
      }

      if (int(foo)==14) {//blowertopspeed
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[14]);
        myPort.write(int(val));
      }

      if (int(foo)==15) {//wpspeed10gps
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[15]);
        myPort.write(int(val));
      }

      if (int(foo)==16) {//wptopspeed
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[16]);
        myPort.write(int(val));
      }

      if (int(foo)==17) {//fcv205max
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[17]);
        myPort.write(int(val));
      }

      if (int(foo)==18) {//fcv205min
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[18]);
        myPort.write(int(val));
      }

      if (int(foo)==19) {//fcv134ignition
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[19]);
        myPort.write(int(val));
      }

      if (int(foo)==20) {//fcv134rampend
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[20]);
        myPort.write(int(val));
      }

      if (int(foo)==21) {//fcv134rampbegin
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[21]);
        myPort.write(int(val));
      }

      if (int(foo)==22) {//fcv141begin
        float val = map(float(theEvent.getStringValue()), 0, 10000, 0, 255);
        myPort.write(myChars[22]);
        myPort.write(int(val));
      }
    }//value
  }//assignable from textfield
}//controlEvent
