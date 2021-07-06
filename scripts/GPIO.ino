void setup() {
  // open the serial port for communication
  Serial.begin(115200);
  

}

// lists to keep track of pin state
bool writepins[20] = {0};
bool readpins[20] = {0};

void loop() {
  // buffer to receive incoming instructions
  byte received[41] = {0};
  char endchar = 0;
  Serial.readBytesUntil(endchar, received, 41);
  for (int i = 0; i < 40; i+= 2) {
    
    int pin = received[i];
    int funcpin = pin;
    char mode = received[i+1];
    if (pin != 0) {
      // analog pins must go by A# for Arduino functions
      if (pin > 13){
        funcpin = A0 + pin%14;
      }
      // case for pin writing
       if (mode == 'l' || mode == 'h') {
          // update pin state
          if (writepins[pin-1] != 1) {
            pinMode(funcpin, OUTPUT);
            writepins[pin-1] = 1;
            readpins[pin-1] = 0; 
          }
          switch (mode){
            case 'l':
              digitalWrite(funcpin, LOW);
              break;
            case 'h':
              digitalWrite(funcpin, HIGH);
              break;
            default:
              break;
          }
       }
       else if (mode == 'r') {
          // update pin state
          if (readpins[pin-1] != 1) {
            pinMode(funcpin, INPUT);
            writepins[pin-1] = 0;
            readpins[pin-1] = 1; 
          }
          int pinvalue = digitalRead(funcpin);
          Serial.println(pinvalue);
       }
       else {
        break;
       }
    }
    else {
      break;
    }
    
  }



  
}
