#include <MCP23X08.h>
#include <Spi.h>
#include <MCP2510.h> 
#include <Canutil.h> 
#include <Wire.h>
#include <SPI.h>

/********* VALUES USED FOR THE STATE MACHINE ************/
#define I2C_INTERRUPT -1
#define UNDEFINED_STATE 0
#define SLAVE 1
#define MASTER 2
#define NORMAL_MODE 3
volatile int state = UNDEFINED_STATE;
volatile int oldState = UNDEFINED_STATE;

/************* TODO : WILL PROBABLY HAVE TO DEFINE SUBSTATE VARIABLE *****************/

/************** PIN USED  FOR THE INTERRUPTS *************/
#define I2C_INTERRUPT_PIN 1
#define SPI_CAN_INTERRUPT_PIN 0

/************ DEFINE THE NODE ID *********************/
#define OWN_ID 1

/**************** SETUP THE I2C/SPI/CAN OBJECTS ****************/
#define MCP23008_ADDR 0x00            // this is the default address     
#define MCP23S08_ADDR 0x00            // this is the default address    

MCP23008 i2c_io(MCP23008_ADDR);         // Init MCP23008
MCP23S08 spi_io(MCP23S08_ADDR, 10);        // Init MCP23S08
MCP2510  can_dev(9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);

/************* DECLARE VOLATILE VARIABLE FOR THE GPIOS RAEADS, AND FORWARD DECLARE FUNCTIONS ***************/
volatile uint8_t i2cGpiosValues; 
volatile uint8_t SpiGpiosValues; 
uint8_t txstatus;
volatile uint8_t opmode;
uint8_t message[8];
uint16_t msgID;

void spiCanInterrupt();
void i2cInterrupt();
void scanNetwork();

/********************** SETUP **********************/
void setup() {

  //  SETUP SPI INTERRUPTS
  spi_io.Write(IOCON, VAL23S08_IOCON);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  spi_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  spi_io.Write(INTCON, 0x0F); // activate interrupts on the inputs (buttons)
  spi_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  spi_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  
  //  SETUP I2C INTERRUPTS
  i2c_io.Write(IOCON, VAL23S08_IOCON);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  i2c_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  i2c_io.Write(GPIO,0xF0);
  i2c_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  i2c_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  i2c_io.Write(GPINTEN, 0x0F);  // activate interrupts on the inputs (buttons)

  // SETUP CANUTIL (COPY PASTA)
  canutil.flashRxbf();  //just for fun!
  can_dev.write(CANINTE,0x01);  //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil.setClkoutMode(0,0);  // disables CLKOUT
  canutil.setTxnrtsPinMode(0,0,0);  // all TXnRTS pins as all-purpose digital input

  canutil.setOpMode(4); // sets configuration mode
  // IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
  // filters and acceptance masks can be modified

  canutil.waitOpMode(4);  // waits configuration mode

  // Bit timing section
  //  setting the bit timing registers with Fosc = 16MHz -> Tosc = 62,5ns
  // data transfer = 125kHz -> bit time = 8us, we choose arbitrarily 8us = 16 TQ  (8 TQ <= bit time <= 25 TQ)
  // time quanta TQ = 2(BRP + 1) Tosc, so BRP =3
  // sync_seg = 1 TQ, we choose prop_seg = 2 TQ 
  // Phase_seg1 = 7TQ yields a sampling point at 10 TQ (60% of bit length, recommended value)
  // phase_seg2 = 6 TQ SJSW <=4 TQ, SJSW = 1 TQ chosen
  can_dev.write(CNF1,0x03);  // SJW = 1, BRP = 3
  can_dev.write(CNF2,0b10110001);  //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
  can_dev.write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5
  

  canutil.setOpMode(0); // sets normal mode 
  opmode = canutil.whichOpMode();

  canutil.setTxBufferDataLength(0,1,0);   // TX normal data, 1 byte long, with buffer 0

  Serial.begin(9600);  // used for debug purpose only
  Serial.write("leaving setup\n");
  
    /************** SETUP INTERRUPTS ****************/
  //attachInterrupt(I2C_INTERRUPT_PIN,i2cInterruptCallback,HIGH);
 // attachInterrupt(SPI_CAN_INTERRUPT_PIN,spiCanInterrupt,LOW);
  
}

/******************************* LOOP *********************/
void loop() {
  switch(state) {
    case I2C_INTERRUPT:
      i2cInterruptLogic();
      break;
    case UNDEFINED_STATE:
      Serial.write("undefined\n"); // ebug indicator
      break;
    case SLAVE:
      //TODO
      Serial.write("slave\n");  // debug indicator
      break;
      
    case MASTER:
      scanNetwork();
      Serial.write("master\n");  // debug indicator
      break;
      
    case NORMAL_MODE:
      //TODO
      Serial.write("normal mode\n");  // debug indicator
      break;
     default:
       Serial.write("huge problem !\n");
       break;
  }
  
  if(digitalRead(3) == LOW) {
    oldState = state;
    state = I2C_INTERRUPT;
  }
}

/************* INTERRUPT CALLBACK SPI/CAN ***************/
void spiCanInterrupt() {
}

/************* INTERRUPT "CALLBACK" I2C ***************/
void i2cInterruptLogic() {
  Serial.write("entering interrupt I2C logic\n");
  i2cGpiosValues = i2c_io.Read(GPIO);  // Reading the gpio actually clear the flag
  switch(oldState) {
    case UNDEFINED_STATE:
      if ( (i2cGpiosValues | 0b11110111) == 0b11110111 ) {  // test if SW9 has been pressed
        state = MASTER;
      } else {
        state = UNDEFINED_STATE;
      }
      break;
    case NORMAL_MODE:
      // do stuff related to normal MODE
      //which will clear the flag cuz gpio read
      state = NORMAL_MODE;
      break;
    case MASTER:
      state = MASTER;
    default:
     state = oldState;
      Serial.write("problem I2C interrupt\n");
      break;
  }
    Serial.write("leaving interrupt I2C logic\n");  
}

void scanNetwork() {
    for(unsigned int i= 1;i <= 47 ;i++) {
  uint16_t message_id = 0x100;
  canutil.setTxBufferID(message_id,0,0,0);  // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
  canutil.setTxBufferDataField(message, 0);   // fills TX buffer 
  canutil.messageTransmitRequest(0,1,3);  // requests transmission of buffer 0 with highest priority*/
    do {
    txstatus = 0;
    txstatus = canutil.isTxError(0);  // checks tx error
    txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
    txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
    txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
  } 
  while (txstatus != 0);
 delay(500);
  message[0]++;
    }
    message[0]=0;
/*
   message[0]++;
   canutil.setTxBufferID(i,0,0,0);
      canutil.setTxBufferDataField(message, 0);
   canutil.messageTransmitRequest(0,1,3);  // requests transmission of buffer 0 with highest priority
  */

 // waits for no error condition   }
  // void Canutil::setTxBufferDataLength(uint8_t rtr,  uint8_t length, uint8_t buffer)
  // void Canutil::setTxBufferID(unsigned int stdID, unsigned long extID, uint8_t extended, uint8_t buffer)
  // void Canutil::setTxBufferDataField(uint8_t data[8], uint8_t buffer)
}
