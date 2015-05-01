#include "mbed.h"
#include "MRF24J40.h"

MRF24J40 mrf(p11, p12, p13, p14, p21);

void rf_send(char *data, uint8_t len) {
  //We need to prepend the message with a valid ZigBee header
  uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
  uint8_t *send_buf = new uint8_t[len+8];

  for(uint8_t i = 0; i < len+8; i++) {
      //prepend the 8-byte header
      send_buf[i] = (i<8) ? header[i] : data[i-8];
  }
  //pc.printf("Sent: %s\r\n", send_buf+8);

  mrf.Send(send_buf, len+8);
  delete send_buf;
}

int rf_receive(char *data, uint8_t maxLength) {
  uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
  uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};

  if(len > 10) {
    //Remove the header and footer of the message
    for(uint8_t i = 0; i < len-2; i++) {
      if(i<8) {
        //Make sure our header is valid first
        if(data[i] != header[i])
          return 0;
      } else {
        data[i-8] = data[i];
      }
    }

  }
  return ((int)len)-10;
}
