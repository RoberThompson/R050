#ifndef R050_h
#define R050_h

class R050{
private:

public:
  void steamGenPID();
  void superheatTest();
  void connect_IO_Expanders();
  void readPTs();
  void readTCs();//used DB_TX in here in last case.
  void readOut();
  void integrityCheck();
  void readOCI();
  void readBtn();
  void error_Checker();
  void steamPressureLow();
  void monitor_SR_Tube_Temps();
  void blinkGRN();
  void blinkAMB();
  void DB_RX();
  void DB_TX();
  void DB_INIT();
};

#endif
