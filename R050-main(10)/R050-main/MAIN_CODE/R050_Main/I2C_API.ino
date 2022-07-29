//---------------------------------------------------------------------//
//--------------------------Data Sharing over I2C----------------------//
//---------------------------------------------------------------------//
float telGetValue(uint8_t telAddr, const char* dsKey) {
  uint8_t rb_CmdByte[1] = {0};
  char rb_keyBuffer[5] = {0};
  floatToBytes readVal;
  
  telReadValue(telAddr, dsKey);
  telReadBack(telAddr, rb_CmdByte, rb_keyBuffer, readVal.asBytes);

  if(rb_CmdByte[0] != 0xFF){
    return 0xFFFFFFFF;
  }
  if(!strstr(rb_keyBuffer, dsKey)){
    return 0xFFFFFFFF;
  }
  return readVal.asFloat;
}

bool telSetValue(uint8_t telAddr, const char* dsKey, float dsVal) {
  uint8_t rb_CmdByte[1] = {0};
  char rb_keyBuffer[5] = {0};
  floatToBytes readVal;
  
  telWriteValue(telAddr, dsKey, dsVal);
  telReadBack(telAddr, rb_CmdByte, rb_keyBuffer, readVal.asBytes);

  if(rb_CmdByte[0] != 0xFE){
    return false;
  }
  if(!strstr(rb_keyBuffer, dsKey)){
    return false;
  }
  if(readVal.asFloat != dsVal){
    return false;
  }
  
  return true;
}

void telWriteValue(uint8_t telAddr, const char* dsKey, float dsVal) {
  floatToBytes writeVal;
  writeVal.asFloat = dsVal;
  Wire.beginTransmission(telAddr);              //Address of Uno used for telemetry
  Wire.write(0xFE);                             //Indicates a WRITE operation
  Wire.write(dsKey);                            //Coorelates to a Storage key
  Wire.write(writeVal.asBytes, 4);
  Wire.endTransmission();                       //Send message
}

void telReadValue(uint8_t telAddr, const char* dsKey) {
  Wire.beginTransmission(telAddr);              //Address of Uno used for telemetry
  Wire.write(0xFF);                             //Indicates a WRITE operation
  Wire.write(dsKey);                            //Coorelates to a Storage key
  Wire.endTransmission();                       //Send message
}

float telReadBack(uint8_t telAddr, uint8_t* cmdByte, char* dsKey, char* dsVal) {

  Wire.requestFrom(telAddr, 10);
  
  cmdByte[0] = Wire.read();
  
  for(int i = 0; i < 5; ++i){
    dsKey[i] = Wire.read();
  }

  for(int i = 0; i < 4; ++i){
    dsVal[i] = Wire.read();
  }
}
//---------------------------------------------------------------------//
//---------------------------------------------------------------------//
//---------------------------------------------------------------------//
