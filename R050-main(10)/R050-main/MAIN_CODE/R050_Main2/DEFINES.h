#ifndef DEFINES_H
#define DEFINES_H

//For OCI417 module ON OLD ECU
//#define MAX485_DE 12
//#define MAX485_RE_NEG 13
//FOR NEW ECU W ESP32
#define MAX485_DE 12
#define MAX485_RE_NEG 12

//ECU jibber jabber
#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

//dashboard data
#define TEL_ADDR 0x03

//Values sent to dashboard readouts
#define REGRX "REGRX"       //Indicates telemetry module has new values

#define BL_FB "BL_FB"    //modbus Analog Input, feedback of blower
#define WP_FB "WP_FB"    //modbus Analog Input, water pump FB
#define BF_FB "BF_FB"    //modbus Analog Input, burner fuel feedback fcv134
#define SR_FB "SR_FB"    //modbus Analog Input, SR_FLOW fb

#define PT213 "PT213"    //modbus Analog Input, Water Pressure
#define PT318 "PT318"    //modbus Analog input hx406 output pressure
#define PT420 "PT420"    //modbus Analog input steam ejector pressure
#define PT304 "PT304"    //modbus Analog input twv308 input pressure

#define TT142 "TT142"    //modbus Analog Input, TT142_SR_FUEL
#define TT301 "TT301"    //modbus Analog Input, TT301_HX406_STEAM_OUT
#define TT303 "TT303"    //modbus Analog Input, TT303_HX504_STEAM_OUT
#define TT306 "TT306"    //modbus Analog Input, TT306_EJECTOR_STEAM_IN
#define TT313 "TT313"    //modbus Analog Input, TT313_HX402_STEAM_OUT
#define TT319 "TT319"    //modbus Analog Input, TT319_HX402_STEAM_SYSTEM
#define TT407 "TT407"    //modbus Analog Input, TT407_STEAM_REFORMER_OUT_LREF
#define TT408 "TT408"    //modbus Analog Input, TT408_HTS_IN_LREF
#define TT410 "TT410"    //modbus Analog Input, TT410_HTS_OUT_LREF
#define TT411 "TT411"    //modbus Analog Input, TT411_FPZ_OUT_LREF
#define TT430 "tt430"    //modbus Analog Input, TT430_SMR_TUBES_INLET
#define TT511 "TT511"    //modbus Analog Input, TT511_SILICON_CARBIDE_OUT
#define TT512 "TT512"    //modbus Analog Input, TT512_SILICON_CARBIDE_OUT
#define TT513 "TT513"    //modbus Analog Input, TT513_HX504_IN
#define TT514 "TT514"    //modbus Analog Input, TT514_HX504_OUT
#define TT441 "TT441"    //modbus Analog Input, TT441_SMR_TUBE1_OUT
#define TT442 "TT442"    //modbus Analog Input, TT442_SMR_TUBE2_OUT
#define TT443 "TT443"    //modbus Analog Input, TT443_SMR_TUBE3_OUT
#define TT444 "TT444"    //modbus Analog Input, TT444_SMR_TUBE4_OUT
#define TT445 "TT445"    //modbus Analog Input, TT445_SMR_TUBE5_OUT
#define TT446 "TT446"    //modbus Analog Input, TT446_SMR_TUBE6_OUT
#define TT447 "TT447"    //modbus Analog Input, TT447_SMR_TUBE7_OUT
#define TT448 "TT448"    //modbus Analog Input, TT448_SMR_TUBE8_OUT
#define TT449 "TT449"    //modbus Analog Input, TT449_SMR_TUBE9_OUT
#define OCIIN "OCIIN"    //modbus rs485, OCI_INPUT_STATUS_WORD
#define OCIOT "OCIOT"    //modbus rs485,OCI_OUTPUT_STATUS_WORD

//values sent from dashboard //configurables
#define SHTMR "SHTMR"    //configurable timer, SUPERHEAT_TIMER
#define BOTMR "BITMR"    //configurable timer, BMM_OFF_TIMER
#define BSTMR "BSTMR"    //configurable timer, BMM_START_TIMER
#define BPTMR "BPTMR"    //configurable timer, BMM_PURGE_TIMER
#define BCTMR "BCTMR"    //configurable timer, BMM_IGNITION_TIMER
#define BRTMR "BRTMR"    //configurable timer, BURNER_RAMP_TIMER
#define BETMR "BETMR"    //configurable timer, BURNER_REACH_END_TIMER
#define SGTMR "SGTMR"    //configurable timer, STEAM_GENERATION_TIMER
#define SPTMR "SPTMR"    //configurable timer, STEAM_AT_170PSI_TIMER
#define SRTMR "SRTMR"    //configurable timer, OPEN_SR_FUEL_TIMER
#define BLPSD "BLPSD"    //configurable value, BLOWER_PURGE_SPEED
#define BLISD "BLISD"    //configurable value, BLOWER_IGNITION_SPEED
#define BLEND "BLEND"    //configurable value, BLOWER_TOP_SPEED
#define BLTSD "BLTSD"    //configurable value, BLOWER_TOP_SPEED
#define WP10G "WP10G"    //configurable value, RO_PUMP_AT_10_GRAMS_PER_SEC
#define WPTSD "WPTSD"    //configurable value, RO_PUMP_TOP_SPEED
#define F205S "F205S"    //configurable value, FCV205_AT_35_PERCENT
#define F205E "F205E"    //configurable value, FCV205_AT_50_PERCENT
#define BFIGN "BFIGN"    //configurable value, FCV134_BURNER_FUEL_FLOW_IGNITION
#define BFRED "BFRED"    //configurable value, FCV134_BURNER_FUEL_FLOW_RAMP_END
#define SRFST "SRFST"    //configurable value, FCV141_SR_FUEL_START_PERCENT
#define FTDIA "FTDIA"    //configurable value, FT132_PIPE_DIA_CONV
#define FTGPS "FTGPS"    //configurable value, FT132_COUNTS_TO_G_PER_SEC
#define FT4MA "FT4MA"    //configurable value, FT132_4MA_OFFSET
#define BTRED "BTRED"    //configurable value, BURNER_TEMP_RAMP_END
#define BTCOV "BTCOV"    //configurable value, BURNER_TEMP_CROSSOVER
#define SRFCT "SRFCT"    //configurable value, SR_FUEL_CUT

#endif
