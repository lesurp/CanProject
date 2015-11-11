#include <MCP23X08.h>
#include <Spi.h>
#include <MCP2510.h> 
#include <Canutil.h> 
#include <Wire.h>
#include <SPI.h>

/********* VALUES USED FOR THE STATE MACHINE ************/
#define UNDEFINED_STATE 0
#define SLAVE 1
#define MASTER 2
#define NORMAL_MODE 3
volatile int state = UNDEFINED_STATE;

/************* TODO : WILL PROBABLY HAVE TO DEFINE SUBSTATE VARIABLE *****************/

/************** PIN USED  FOR THE INTERRUPTS *************/
#define I2C_INTERRUPT 1
#define SPI_CAN_INTERRUPT 0

/************ DEFINE THE NODE ID *********************/
#define OWN_ID 1

/**************** SETUP THE I2C/SPI/CAN OBJECTS ****************/
#define MCP23008_ADDR 0x00            // this is the default address     
#define MCP23S08_ADDR 0x00            // this is the default address    

MCP23008 i2c_io(MCP23008_ADDR);         // Init MCP23008
MCP23S08 spi_io(MCP23S08_ADDR, 10);        // Init MCP23S08
MCP2510  can_dev(9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);


void setup() {
  /************** SETUP INTERRUPTS ****************/
  attachInterrupt(I2C_INTERRUPT,i2cInterrupt,LOW);
  attachInterrupt(SPI_CAN_INTERRUPT,spiCanInterrupt,LOW);
  
  //  SETUP SPI INTERRUPTS
    // TODO: setup open drain int
  spi_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  spi_io.Write(INTCON, 0x0F); // activate interrupts on the inputs (buttons)
  spi_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  spi_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  
  //  SETUP I2C INTERRUPTS
      // TODO: setup open drain int
  i2c_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  i2c_io.Write(GPINTEN, 0x0F);  // activate interrupts on the inputs (buttons)
  i2c_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  i2c_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  
  Serial.begin(9600);  // used for debug purpose only
}

void loop() {
  switch(state) {
    case SLAVE:
      //TODO
      Serial.write("slave");  // debug indicator
      break;
      
    case MASTER:
      //TODO
      Serial.write("master");  // debug indicator
      break;
      
    case NORMAL_MODE:
      //TODO
      Serial.write("normal mode");  // debug indicator
      break;
  }
}

/************* INTERRUPT CALLBACK ***************/
void spiCanInterrupt() {
}

/************* INTERRUPT CALLBACK ***************/
void i2cInterrupt() {
  switch(state) {
    case UNDEFINED_STATE:
      uint8_t gpiosValues = i2c_io.Read(GPIO);  // Reading the gpio actually clear the flag
      if ( gpiosValues & 0b00001000 ) {  // TODO check if condition correct
        state = MASTER;
      }
      break;
    case NORMAL_MODE:
      // do stuff related to normal MODE
      //which will clear the flag cuz gpio read
      break;
    default:
        // TODO : clear interrupt flag
      break;
  }
  
  
}
