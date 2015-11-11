#define UNDEFINED_STATE 0
#define SLAVE 1
#define MASTER 2
#define NORMAL_MODE 3
#define I2C_INTERRUPT 1
#define SPI_CAN_INTERRUPT 0

volatile int state = UNDEFINED_STATE;

void setup() {
  attachInterrupt(I2C_INTERRUPT,i2cInterrupt,LOW);
  attachInterrupt(SPI_CAN_INTERRUPT,spiCanInterrupt,LOW);
}

void loop() {
  switch(state) {
    case SLAVE:
      //TODO
      break;
    case MASTER:
      //TODO
      break;
    case NORMAL_MODE:
      //TODO
      break;
  }
}

void spiCanInterrupt() {
}

void i2cInterrupt() {
  switch(state) {
    case UNDEFINED_STATE:
    // read GPIO, if sw9 =>
      state = MASTER;
      //flag reset cuz of the GPIO read
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
