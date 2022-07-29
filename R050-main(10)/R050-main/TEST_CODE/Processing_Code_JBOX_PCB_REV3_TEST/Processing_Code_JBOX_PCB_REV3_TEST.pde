import processing.serial.*;
import controlP5.*;

ControlP5 cp5;
Chart myChart;

String[] myControllers={"tC142" ,"tC301","tC303","tC306","tC313","tC319","tC407","tC408","tC410","tC411","tC430","tC511",
              "tC512","tC513","tC514","tC441","tC442","tC443","tC444","tC445","tC446","tC447","tC448","tC449",
              "BL508","PMP204","SPARE","FT132","PT318","PT213","PT420","PT304","DUN_PSH","DUN_PSL","FCV134","DUN_ZSL",
              "BLWR_508","WP_204","FCV_134","FCV_205","FCV_141","XV801","BLWR_EN","WP_EN","TWV308","XV1100","XV501","BMM_CR2","TWV901","XV909"};

char[] myChars={'A','B','C','D','E','F','G','H','I','J','K','L','M','N'};

Serial myPort;

String dataString;
int nl=10;
int j;

int Toggle_offset=1800;
int intx=80,inty=70;//for spacing numberbox controllers
int intx1=0,inty1=0; //for spacing switch controllers
int intx2=0,inty2=0; //for spacing knob controlers

boolean dataHeader=false;

void setup(){

size(3850,2400);
smooth();
noStroke();
PFont font = createFont("arial",75);

cp5 = new ControlP5(this);
cp5.setColorForeground(0xff00aaff);//when selected color rrggbb
cp5.setColorBackground(0xff003366);
cp5.setFont(font);
cp5.setColorActive(0xff00aaff);

  String portName = Serial.list()[0];
  myPort = new Serial(this,portName,9600);

myChart = cp5.addChart("dataflow")
            .setPosition(1875, 94)
            .setSize(1875, 828)
            .setRange(0,20000)
            .setView(Chart.BAR_CENTERED)
            .setStrokeWeight(.2)
            ;

cp5.addBang("SAVE  AS\nDEFAULT")
  .setPosition(2500,2050)
  .setSize(350,100);

for(int i=36;i<41;i++){
 if(i==38){
  cp5.addKnob(myControllers[i])
  .setRange(2,10)
  .setPosition(1600+intx2,1100+inty2)
  .setRadius(150)
  .setNumberOfTickMarks(8)
  .setTickMarkLength(20)
  .snapToTickMarks(true)
  .setDragDirection(Knob.HORIZONTAL)
  .setValue(0)
  ;}
else{
cp5.addKnob(myControllers[i])
  .setRange(0,10)
  .setPosition(1600+intx2,1100+inty2)
  .setRadius(150)
  .setNumberOfTickMarks(10)
  .setTickMarkLength(20)
  .snapToTickMarks(true)
  .setDragDirection(Knob.HORIZONTAL)
  .setValue(0)
  ;}

  intx2+=470;

  }

 //adding NumberBoxes for dispalying numbers
for(int i=0;i<=35;i++){
cp5.addNumberbox(myControllers[i])
   .setPosition(intx,inty)
   //.setSize(210,32)//for home use
   .setSize(375,63)
   ;
   //for home use
   inty+=188;
   if(i==11||i==23||i==35){
     intx+=450;
     inty=70;
 }
}

   for(int i=41;i<=49;i++){
   cp5.addToggle(myControllers[i])
      .setPosition(Toggle_offset+intx1,1600+inty1)
      .setSize(188,63)
      .setValue(0);
      ;

   intx1+=500;
   if(i==44){inty1+=200;intx1=0;Toggle_offset=1550;}
}

myChart.addDataSet("incoming");
myChart.setData("incoming",new float[200]);

}

void draw(){

background(150);

 while(myPort.available()>0){
 if(!dataHeader){

  int header = myPort.read();

  if(header=='@'){dataHeader=true;j=0;}}

 if(dataHeader){
  dataString=myPort.readStringUntil(nl);

  if(dataString!=null){
   if(j<=49){
    cp5.getController(myControllers[j]).setValue(float(dataString));
    if(j<=36){myChart.push("incoming",float(dataString));}
    j++;}
   else if(j>49){myPort.clear(); dataHeader=false;}
  }
 }
}
}

void controlEvent(ControlEvent theEvent){
 if(theEvent.isController()){
  for(int i=36;i<=49;i++){
   if(theEvent.getController().getName()==myControllers[i]){
    int val=int(theEvent.getController().getValue());
    myPort.write('@');
    myPort.write(myChars[i-36]);
    myPort.write(val);
    myPort.write('#');
   }
  }
 }

  if(theEvent.getController().getName()=="SAVE  AS\nDEFAULT"){
    myPort.write('~');

  }
}
