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

volatile uint8_t i2cGpiosValues; 
volatile uint8_t SpiGpiosValues; 

void spiCanInterrupt();
void i2cInterrupt();

void setup() {

  //  SETUP SPI INTERRUPTS
    // TODO: setup open drain int
  spi_io.Write(IOCON, VAL23S08_IOCON);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  spi_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  spi_io.Write(INTCON, 0x0F); // activate interrupts on the inputs (buttons)
  spi_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  spi_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  
  //  SETUP I2C INTERRUPTS
      // TODO: setup open drain int
  i2c_io.Write(IOCON, VAL23S08_IOCON);  // Sets defaults for MCP23S08, in particular puts interrupt output open-drain
  i2c_io.Write(IODIR, 0x0F);   // sets port direction for individual bits
  i2c_io.Write(GPIO,0xF0);
  i2c_io.Write(INTCON, 0x0F);  // interrupts are triggered when compared to DEFVAL values, not on pin-change
  i2c_io.Write(DEFVAL, 0x0F);  // DEFVAL  sets the default values for the pin (high state here). Interrupts occurs on a low state
  i2c_io.Write(GPINTEN, 0x0F);  // activate interrupts on the inputs (buttons)

  Serial.begin(9600);  // used for debug purpose only
  Serial.write("leaving setup");
  
    /************** SETUP INTERRUPTS ****************/
  //attachInterrupt(I2C_INTERRUPT_PIN,i2cInterruptCallback,HIGH);
  attachInterrupt(SPI_CAN_INTERRUPT_PIN,spiCanInterrupt,LOW);
  
}

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
      //TODO
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

/************* INTERRUPT CALLBACK ***************/
void spiCanInterrupt() {
}

/************* INTERRUPT CALLBACK ***************/
/*void i2cInterruptCallback() {
 oldState = state;
 state = I2C_INTERRUPT; 
}*/

void i2cInterruptLogic() {
  Serial.write("entering interrupt I2C logic\n");
  i2cGpiosValues = i2c_io.Read(GPIO);  // Reading the gpio actually clear the flag
  switch(oldState) {
    case UNDEFINED_STATE:
      if ( (i2cGpiosValues | 0b11110111) == 0b11110111 ) {  // TODO check if condition correct
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
      Serial.write("problem I2C interrupt");
      break;
  }
    Serial.write("leaving interrupt I2C logic\n");  
}
