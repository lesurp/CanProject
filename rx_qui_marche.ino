//************************************************************
// normal mode, recs one byte, std messages, filter on
//
//************************************************************



//#include <MCP23X08.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <MCP2510.h>  // The CAN interface device
#include <Canutil.h>  // The CAN interface utilities

MCP2510  can_dev(9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);

// MCP2510 library defines some basic access function to the CAN device like write register, read register and so on.
// For example, directly writing to the CANINTE register of the MCP2510 is:
//  can_dev.write(CANINTE,0x03);
//
// Canutil library defines "higher level" functions which manipulate more than one register and/or argument,
// like acceptance mask programming for example:
// canutil.setAcceptanceFilter(0x2AB, 2000, 0, 0);

// MCP2510 can be used standalone, Canutil must be used in conjunction with MCP2510

LiquidCrystal lcd(15, 0, 14, 4, 5, 6, 7);  //  4 bits without R/W pin
//MCP23008 i2c_io(MCP23008_ADDR);         // Init MCP23008


uint8_t canstat = 0;
uint8_t opmode, txstatus;
volatile int isInt; // variable used within interrupt routine, it MUST be "volatile"
uint8_t tosend[8];
uint8_t recSize, recData[8];
uint16_t msgID;


void setup() {

  // i2c_io.Write(IOCON, 0x04);   // makes I2C interrupt pin open-drain to allow sharing INT0 pin with MCP2510 without conflict
  attachInterrupt(0, somethingReceived, FALLING);  // int received on pin 2 if JMP16 is in position A

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("simple RX");
  opmode = canutil.whichOpMode();
  lcd.print(" mode ");
  lcd.print(opmode, DEC);


  canutil.setOpMode(4); // sets configuration mode
  // IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
  // filters and acceptance masks can be modified

  canutil.waitOpMode(4);  // waits configuration mode


  canutil.flashRxbf();  //just for fun!


  can_dev.write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil.setClkoutMode(0, 0); // disables CLKOUT
  canutil.setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input


  // Bit timing section
  //  setting the bit timing registers with Fosc = 16MHz -> Tosc = 62,5ns
  // data transfer = 125kHz -> bit time = 8us, we choose arbitrarily 8us = 16 TQ  (8 TQ <= bit time <= 25 TQ)
  // time quanta TQ = 2(BRP + 1) Tosc, so BRP =3
  // sync_seg = 1 TQ, we choose prop_seg = 2 TQ
  // Phase_seg1 = 7TQ yields a sampling point at 10 TQ (60% of bit length, recommended value)
  // phase_seg2 = 6 TQ SJSW <=4 TQ, SJSW = 1 TQ chosen
  can_dev.write(CNF1, 0x03); // SJW = 1, BRP = 3
  can_dev.write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
  can_dev.write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

  // acceptance filters and mask
  // Acceptance mask: if mask bit is set to 0, the message ID corresponding bit is always accepted
  //                  if mask bit is set to 1, the message ID corresponding bit is accepted provided it matches
  //                  the corresponding filter bit
  //                  mask = FFF... checks all ID bits, mask = 000... doesn't perform any check

  // Settings for buffer RXB0
  //canutil.setRxOperatingMode(1, 1, 1);  // mask off  and rollover
  canutil.setRxOperatingMode(1, 1, 0);  // standard ID messages only  and rollover
  canutil.setAcceptanceFilter(0x101, 0, 0, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil.setAcceptanceFilter(0x100, 0, 0, 1); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 1
  canutil.setAcceptanceMask(0xFFFF, 0x00000000, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 0
  // in this case, only messages with ID equal to 0x2AB or 0x2AC will be accepted since mask is set to 0xFFF
  canutil.setRxOperatingMode(1, 1, 1);  // std  ID messages  rollover ignored
  canutil.setAcceptanceFilter(0x2AA, 0, 1, 2); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 2
  canutil.setAcceptanceFilter(0x2AA, 0, 1, 3); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 3
  canutil.setAcceptanceFilter(0x2AA, 0, 1, 4); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 4
  canutil.setAcceptanceFilter(0x2AA, 0, 1, 5);// 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 5
  canutil.setAcceptanceMask(0xFFF, 0xFFFFFFFF, 1); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 1

  canutil.setOpMode(0); // sets normal  mode
  opmode = canutil.whichOpMode();
  lcd.setCursor(15, 0);
  lcd.print(opmode, DEC);

  isInt = 0; // resets interrupt flag
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags
  delay(3000);

}



void loop() {
  //isInt = 0;


  do {
    delay(1);
  }
  while (isInt == 0);  // waits for a received message

  

  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  recSize = canutil.whichRxDataLength(0); // checks the number of bytes received in buffer 0 (max = 8)

  for (int i = 0; i < recSize; i++) { // gets the bytes
    recData[i] = canutil.receivedDataValue(0, i);
  }

msgID = canutil.whichStdID(0);
  lcd.setCursor(0, 1);
  lcd.print("ID= ");
  lcd.print(msgID, HEX);
  lcd.print(" data= ");
  lcd.print(recData[0], HEX);

  isInt = 0; // resets interrupt flag
  //isInt = 0;
  delay(500);

}


//************************************************
// routine attached to INT pin
//************************************************
void somethingReceived()
{
  isInt = 1;
}


//******************************************************************
//                     other routines
//******************************************************************
