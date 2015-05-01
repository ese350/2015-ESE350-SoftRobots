#include "mbed.h"
#include "soft.hpp"
#include "hcsr04.h"
#include "rtos.h"
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


Serial pc(USBTX, USBRX);

char rxbuf[10];
unsigned int ping = 0;
bool flee_mode = false;
Ticker flee_ticker;

void ticker_disarm() {
  flee_mode = false;
  flee_ticker.detach();
}

DigitalOut led(LED4);
void get_rf(void const *args) {
  while(1) {
    int res = rf_receive(rxbuf, 10);
    if(res > 0) {
      led = 1;
      wait_ms(25);
      led = 0;
      printf("recv %s\n\r", rxbuf);
    }
    else
      continue;
    sscanf(rxbuf, "%u", &ping);
    if(ping < 10) {
      flee_mode = true;
      flee_ticker.attach(&ticker_disarm, 5.0);
    }
  }
}

int main() {
  mrf.SetChannel(9);
  
  Thread rf_thread(get_rf);
  
  Soft::Control softCtl;
  
  // actuator configuration
  softCtl.ctl["FL"] = p7;
  //softCtl["FL"] += new Soft::Monitor::Pressure(p9);
  softCtl.ctl["FR"] = p8;
  //softCtl["FR"] += new Soft::Monitor::Pressure(p10);
  softCtl.ctl["BL"] = p5;
  //softCtl["BL"] += new Soft::Monitor::Pressure(p11);
  softCtl.ctl["BR"] = p6;
  //softCtl["BR"] += new Soft::Monitor::Pressure(p12);
  
  // walk cycle configuration (backward)
  softCtl.alias["bwd"] = Soft::KeyRange(50,65);
  softCtl[50] = new Soft::Control::Frame(.3);
  softCtl[50] += softCtl.ctl["FL"];
  softCtl[53] = new Soft::Control::Frame(.2);
  softCtl[53] += softCtl.ctl["FL"];
  softCtl[53] += softCtl.ctl["BR"];
  softCtl[54] = new Soft::Control::Frame(.4);
  softCtl[54] += softCtl.ctl["BR"];
  softCtl[57] = new Soft::Control::Frame(.2);
  softCtl[57] += softCtl.ctl["BR"];
  softCtl[57] += softCtl.ctl["FR"];
  softCtl[58] = new Soft::Control::Frame(.3);
  softCtl[58] += softCtl.ctl["FR"];
  softCtl[61] = new Soft::Control::Frame(.2);
  softCtl[61] += softCtl.ctl["FR"];
  softCtl[61] += softCtl.ctl["BL"];
  softCtl[62] = new Soft::Control::Frame(.4);
  softCtl[62] += softCtl.ctl["BL"];
  softCtl[65] = new Soft::Control::Frame(.2);
  softCtl[65] += softCtl.ctl["BL"];
  softCtl[65] += softCtl.ctl["FL"];
  
  softCtl.alias["fwd"] = Soft::KeyRange(0,15);
  softCtl[0] = new Soft::Control::Frame(.3);
  softCtl[0] += softCtl.ctl["BL"];
  softCtl[3] = new Soft::Control::Frame(.2);
  softCtl[3] += softCtl.ctl["BL"];
  softCtl[3] += softCtl.ctl["FR"];
  softCtl[4] = new Soft::Control::Frame(.4);
  softCtl[4] += softCtl.ctl["FR"];
  softCtl[7] = new Soft::Control::Frame(.2);
  softCtl[7] += softCtl.ctl["FR"];
  softCtl[7] += softCtl.ctl["BR"];
  softCtl[8] = new Soft::Control::Frame(.3);
  softCtl[8] += softCtl.ctl["BR"];
  softCtl[11] = new Soft::Control::Frame(.2);
  softCtl[11] += softCtl.ctl["BR"];
  softCtl[11] += softCtl.ctl["FL"];
  softCtl[12] = new Soft::Control::Frame(.4);
  softCtl[12] += softCtl.ctl["FL"];
  softCtl[15] = new Soft::Control::Frame(.2);
  softCtl[15] += softCtl.ctl["FL"];
  softCtl[15] += softCtl.ctl["BL"];


  softCtl.alias["turn"] = Soft::KeyRange(100,115);
  softCtl[100] = new Soft::Control::Frame(.3);
  softCtl[100] += softCtl.ctl["BL"];
  softCtl[103] = new Soft::Control::Frame(.2);
  softCtl[103] += softCtl.ctl["BL"];
  softCtl[103] += softCtl.ctl["FR"];
  softCtl[104] = new Soft::Control::Frame(.4);
  softCtl[104] += softCtl.ctl["FR"];
  softCtl[107] = new Soft::Control::Frame(.2);
  softCtl[107] += softCtl.ctl["FR"];
  softCtl[107] += softCtl.ctl["BR"];
  softCtl[108] = new Soft::Control::Frame(.3);
  softCtl[108] += softCtl.ctl["FL"];
  softCtl[111] = new Soft::Control::Frame(.2);
  softCtl[111] += softCtl.ctl["FL"];
  softCtl[111] += softCtl.ctl["BL"];
  softCtl[112] = new Soft::Control::Frame(.4);
  softCtl[112] += softCtl.ctl["BR"];
  softCtl[115] = new Soft::Control::Frame(.2);
  softCtl[115] += softCtl.ctl["BR"];
  softCtl[115] += softCtl.ctl["FL"];
  
  //softCtl[0] = new Soft::Control::Frame(1.0);
  //softCtl[0] += softCtl.ctl["FL"];
  while(1) {
    printf("ping %u\n\r", ping);
    if(flee_mode) {
      printf("flee\n\r");
      softCtl.run("bwd");
    }
    else
      softCtl.run("fwd");
  }
}
