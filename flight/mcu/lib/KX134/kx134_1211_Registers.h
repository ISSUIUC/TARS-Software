#ifndef __KX134_Registers_H__
#define __KS134_Registers_H__

#define KX134_CS_PIN 27

/////////////////////////////////////////
// KX134-1121 Registers //
/////////////////////////////////////////
#define MAIN_ID 0x00
#define PART_ID 0x01
#define XADP_L 0x02
#define XADP_H 0x03
#define YADP_L 0x04
#define YADP_H 0x05
#define ZADP_L 0x06
#define ZADP_H 0x07
#define XOUT_L 0x08
#define XOUT_H 0x09
#define YOUT_L 0x0A
#define YOUT_H 0x0B
#define ZOUT_L 0x0C
#define ZOUT_H 0x0D
//?Konix Reserved 0E-11

#define COTR 0x12
#define WHO_AM_I 0x13
#define TSCP 0x14
#define TSPP 0x15
#define INS1 0x16
#define INS2 0x17
#define INS3 0x18
#define STATUS_REG 0x19
#define INT_REL 0x1A

#define CNTL1 0x1B
#define CNTL2 0x1C
#define CNTL3 0x1D
#define CNTL4 0x1E
#define CNTL5 0x1F
#define CNTL6 0x20
#define ODNCTL 0x21
#define INC1 0x22
#define INC2 0x23
#define INC3 0x24
#define INC4 0x25
#define INC5 0x26
#define INC6 0x27
//?Konix Reserved 0x28
#define TILT_TIMER 0x29
#define TDTRC 0x2A
#define TDTC 0x2B
#define TTH 0x2C
#define TTL 0x2D
#define FTD 0x2E
#define STD 0x2F
#define TLT 0x30
#define TWS 0x31
#define FFTH 0x32

#define FFC 0x33
#define FFCNTL 0X34
//?Konix Reserved 0x35
//?Konix Resrved 0x36
#define TILT_ANGLE_LL 0x37
#define TILT_ANGLE_HL 0x38
#define HYST_SET 0x39
#define LP_CNTL1 0x3A
#define LP_CNTL2 0x3B
//?Konix Reserved 0x3C-48
#define WUFTH 0x49
#define BTSWUFTH 0x4A
#define BTSTH 0x4B
#define BTFC 0x4B
#define WUFC 0x4D
//?Konix Reserved 0x4E-5C
#define SELF_TEST 0x5D
#define BUF_CNTL1 0x5E
#define BUF_CNTL2 0x5F
#define BUF_STATUS_1 0x60
#define BUF_STATUS_2 0x61
#define BUF_CLEAR 0x62
#define BUF_READ 0x63
// #define ADP_CNTL(1-19)  0x64-76  // something wrong in parens?
//?Konix Reserved 0x77-7F

#endif
