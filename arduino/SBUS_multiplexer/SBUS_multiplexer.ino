#define SBUS_BAUD 100000
#define SBUS_HEADER 0x0f
#define SBUS_FOOTER 0x00
#define SBUS_DF_LEN 25

uint16_t channel_data[16]; //[ch1 : ch16]
uint8_t digital_channels[2] = {0, 0}; //[ch17, ch18]
uint8_t received_data[23];
uint8_t current_byte, previous_byte, byte_idx;

bool manual_control = false;

void setup() {
  //configure UART
  Serial.begin(SBUS_BAUD, SERIAL_8E2);

  pinMode(3, INPUT);
  pinMode(4, OUTPUT);
}

uint8_t SBUS_receive() {
  while (Serial.available() > 0) {
    current_byte = Serial.read();

    if (byte_idx == 0) { //check for data frame start
      if (current_byte == SBUS_HEADER && previous_byte == SBUS_FOOTER) {
        byte_idx = 1;
      } else {
        byte_idx = 0;
      }
    } else { //parse received data
      if (byte_idx == SBUS_DF_LEN-1) { //check footer
        byte_idx = 0;
        // digitalWrite(4, 0);
        return current_byte == SBUS_FOOTER;
      } else {
        received_data[byte_idx-1] = current_byte;
        // digitalWrite(4, !digitalRead(4));
      }

      byte_idx++;
    }
    
    previous_byte = current_byte;
  }
}

uint8_t SBUS_read() {
  
  if (SBUS_receive()) {

    // channel_data[0]  = (uint16_t) ((received_data[0]    |received_data[1] <<8)                     & 0x07FF);
    // channel_data[1]  = (uint16_t) ((received_data[1]>>3 |received_data[2] <<5)                     & 0x07FF);
    // channel_data[2]  = (uint16_t) ((received_data[2]>>6 |received_data[3] <<2 |received_data[4]<<10)    & 0x07FF);
    // channel_data[3]  = (uint16_t) ((received_data[4]>>1 |received_data[5] <<7)                     & 0x07FF);
    channel_data[4]  = (uint16_t) ((received_data[5]>>4 |received_data[6] <<4)                     & 0x07FF);
    // channel_data[5]  = (uint16_t) ((received_data[6]>>7 |received_data[7] <<1 |received_data[8]<<9)     & 0x07FF);
    // channel_data[6]  = (uint16_t) ((received_data[8]>>2 |received_data[9] <<6)                     & 0x07FF);
    // channel_data[7]  = (uint16_t) ((received_data[9]>>5 |received_data[10]<<3)                     & 0x07FF);
    // channel_data[8]  = (uint16_t) ((received_data[11]   |received_data[12]<<8)                     & 0x07FF);
    // channel_data[9]  = (uint16_t) ((received_data[12]>>3|received_data[13]<<5)                     & 0x07FF);
    // channel_data[10] = (uint16_t) ((received_data[13]>>6|received_data[14]<<2 |received_data[15]<<10)   & 0x07FF);
    // channel_data[11] = (uint16_t) ((received_data[15]>>1|received_data[16]<<7)                     & 0x07FF);
    // channel_data[12] = (uint16_t) ((received_data[16]>>4|received_data[17]<<4)                     & 0x07FF);
    // channel_data[13] = (uint16_t) ((received_data[17]>>7|received_data[18]<<1 |received_data[19]<<9)    & 0x07FF);
    // channel_data[14] = (uint16_t) ((received_data[19]>>2|received_data[20]<<6)                     & 0x07FF);
    // channel_data[15] = (uint16_t) ((received_data[20]>>5|received_data[21]<<3)                     & 0x07FF);

    // digital_channels[0] = received_data[22] & 0x01;
    // digital_channels[1] = received_data[22] & 0x02;

    return true;
  }
  
  return false;
}

void loop() {

  if (SBUS_read()) {
    digitalWrite(4, channel_data[4] > 1500);
  }

}
