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
#define RECEIVED_WAKE_UP 4
#define RECEIVED_LIST 5
volatile int state = UNDEFINED_STATE;
volatile int oldState = UNDEFINED_STATE;

/************* TODO : WILL PROBABLY HAVE TO DEFINE SUBSTATE VARIABLE *****************/

/************** PIN USED  FOR THE INTERRUPTS *************/
#define I2C_INTERRUPT_PIN 1
#define SPI_CAN_INTERRUPT_PIN 0

/************ DEFINE THE NODE ID *********************/
#define OWN_ID 5

/**************** SETUP THE I2C/SPI/CAN OBJECTS ****************/
MCP23008 i2c_io(MCP23008_ADDR);         // Init MCP23008
MCP23S08 spi_io(MCP23S08_ADDR, 10);        // Init MCP23S08
MCP2510  can_dev (9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);

/************* DECLARE VOLATILE VARIABLE FOR THE GPIOS RAEADS, AND FORWARD DECLARE FUNCTIONS ***************/
volatile uint8_t i2cGpiosValues;
volatile uint8_t SpiGpiosValues;
uint8_t txstatus;
volatile uint8_t opmode;
volatile uint8_t idsList[8];
volatile uint16_t msgID;
volatile int recSize;
int storedIdsNumber = 1;
bool alreadyScanned = false;
unsigned long startTime;
volatile int dataReceived[8];

void spiCanInterrupt();
void i2cInterruptCallback();
void scanNetwork();
void i2cInterruptLogic();
bool handleWakeUp();
bool getIdsList();
/********************** SETUP **********************/
void setup() {

  for (int i = 0; i < 8; i++) {
    idsList[i] = 0;
  }

  //  SETUP SPI INTERRUPTS
  spi_io.Write(IOCON, VAL23S08_IOCON | 0b00000010);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  spi_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  spi_io.Write(INTCON, 0x0F); // activate interrupts on the inputs (buttons)
  spi_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  spi_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state

  //  SETUP I2C INTERRUPTS
  i2c_io.Write(IOCON, VAL23S08_IOCON | 0b00000010);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  i2c_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  i2c_io.Write(GPIO, 0xF0);
  i2c_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  i2c_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  i2c_io.Write(GPINTEN, 0x0F);  // activate interrupts on the inputs (buttons)

  // SETUP CANUTIL (COPY PASTA)
  canutil.flashRxbf();  //just for fun!
  can_dev.write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil.setClkoutMode(0, 0); // disables CLKOUT
  canutil.setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

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
  can_dev.write(CNF1, 0x03); // SJW = 1, BRP = 3
  can_dev.write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
  can_dev.write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

  // SETUP MASKS / FILTERS FOR CAN
  canutil.setRxOperatingMode(1, 1, 0);  // standard ID messages only  and rollover
  canutil.setAcceptanceFilter(0x102, 0, 0, 2); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil.setAcceptanceFilter(0x101, 0, 0, 1); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil.setAcceptanceFilter(0x100, 0, 0, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 1
  canutil.setAcceptanceMask(0xFFFF, 0x00000000, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 0

  canutil.setOpMode(0); // sets normal mode
  opmode = canutil.whichOpMode();

  canutil.setTxBufferDataLength(0, 1, 0); // TX normal data, 1 byte long, with buffer 0

  Serial.begin(9600);  // used for debug purpose only
  Serial.write("leaving setup\n");

  /************** SETUP INTERRUPTS ****************/
  attachInterrupt(I2C_INTERRUPT_PIN, i2cInterruptCallback, FALLING);
  attachInterrupt(SPI_CAN_INTERRUPT_PIN, spiCanInterrupt, FALLING);

}

/******************************* LOOP *********************/
void loop() {
  switch (state) {
    case I2C_INTERRUPT:
      i2cInterruptLogic();
      break;
    case UNDEFINED_STATE:
      if (state != oldState) Serial.write("undefined\n"); // ebug indicator
      break;
    case SLAVE:
      //TODO
      //if (state != oldState) Serial.write("slave\n"); // debug indicator
      break;

    case MASTER:
      if (state != oldState) Serial.write("master\n"); // debug indicator
      if (!alreadyScanned) {
        scanNetwork();
        alreadyScanned = true;
        startTime = millis();
      } else if ((millis() - startTime) > 2000) {
        oldState = MASTER;
        state = NORMAL_MODE;
      }
      break;

    case NORMAL_MODE:
      if (state != oldState)  Serial.write("normal mode\n"); // debug indicator
      for (int i = 0; i < 8; i++) {
        Serial.println(idsList[i]);
      }
      break;
    case RECEIVED_WAKE_UP:
      if (handleWakeUp()) {
        oldState = state;
        state = SLAVE;
      } else {
        int temp = state;
        state = oldState;
        oldState = temp;
      }
      if (state != oldState) Serial.write("received wkae up mesg\n"); // debug indicator
      break;
      
    case RECEIVED_LIST:
      Serial.println(msgID);
      if (getIdsList()) {
        oldState = state;
        state = NORMAL_MODE;
      } else { 
        oldState = state;
        state = SLAVE;        
      }
      break;
    default:
      if (state != oldState) Serial.write("huge problem !\n");
      break;
  }
}

/************* INTERRUPT CALLBACK SPI/CAN ***************/
void spiCanInterrupt() {

  switch (state) {
    case UNDEFINED_STATE:
      oldState = state;
      state = RECEIVED_WAKE_UP;
      break;

    case SLAVE:
      oldState = state;
      state = RECEIVED_LIST;
      break;

    case NORMAL_MODE:// TODO : ALL THE WEIRD SHIT
      break;

    case MASTER:
      if (msgID == 0x101) {
        idsList[storedIdsNumber++] = canutil.receivedDataValue(0, 0);
      }
      break;
  }
}

/************* INTERRUPT "CALLBACK" I2C ***************/
void i2cInterruptCallback() {
  oldState = state;
  state = I2C_INTERRUPT;
}

void i2cInterruptLogic() {
  Serial.write("entering interrupt I2C logic\n");
  i2cGpiosValues = i2c_io.Read(GPIO);  // Reading the gpio actually clear the flag
  switch (oldState) {
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
  uint8_t message[8];
  message[0] = 1;
  idsList[0] = OWN_ID;
  for (unsigned int i = 1; i <= 47 ; i++) {
    uint16_t message_id = 0x100;
    canutil.setTxBufferID(message_id, 0, 0, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
    canutil.setTxBufferDataField(message, 0);   // fills TX buffer
    Serial.write("writing with id n100 with message: ");
    Serial.println(message[0]);
    canutil.messageTransmitRequest(0, 1, 3); // requests transmission of buffer 0 with highest priority*/
    do {
      txstatus = 0;
      txstatus = canutil.isTxError(0);  // checks tx error
      txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
      txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
      txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
    }
    while (txstatus != 0);
    Serial.write("wrote with id n100 with message: ");
    Serial.println(message[0]);
    delay(500);
    message[0]++;
  }
  message[0] = 0;
}


bool handleWakeUp() {
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags
  recSize = canutil.whichRxDataLength(0); // checks the number of bytes received in buffer 0 (max = 8)
  for (int i = 0; i < recSize; i++) { // gets the bytes
    dataReceived[i] = canutil.receivedDataValue(0, i);
  }
  msgID = canutil.whichStdID(0);

  Serial.println(dataReceived[0]);
  Serial.println(OWN_ID);
  if (msgID == 0x100 && dataReceived[0] == OWN_ID) { // TODO ANSWER WITH ACKNOWLEDGE
    Serial.println("la valeur que l'on veut");
    Serial.println(dataReceived[0]);
    Serial.write("Received own ID !");
    uint8_t message[8];
    message[0] = OWN_ID;
    uint16_t message_id = 0x101;
    canutil.setTxBufferID(message_id, 0, 0, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
    canutil.setTxBufferDataField(message, 0);   // fills TX buffer
    canutil.messageTransmitRequest(0, 1, 2); // requests transmission of buffer 0 with highest priority*/
    return true;
  }
  return false;
}

bool getIdsList() {
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags
  recSize = canutil.whichRxDataLength(0); // checks the number of bytes received in buffer 0 (max = 8)
  for (int i = 0; i < recSize; i++) { // gets the bytes
    dataReceived[i] = canutil.receivedDataValue(0, i);
  }
  msgID = canutil.whichStdID(0);


  if (msgID == 0x102) {
    for (int i = 0; i < recSize; i++) { // gets the bytes
      idsList[i] = dataReceived[i];
      Serial.println("coucou c est moi");
    }
    return true;
  }
  return false;
}

