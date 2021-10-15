
/*
    Connect the SPI Master device to the following pins on the esp8266:

    GPIO    NodeMCU   Name  |   Uno
  ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13

    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!

*/

#include "SPISlave.h"

void setup() {
  SPISlave.onData([](uint8_t * data, size_t len) {
    String message = String((char *)data);
    (void) len;

  if (message.equals("hi")) {
      char answer[33]={0};
      sprintf(answer, "hi");
      SPISlave.setData("hi");
    }
  });

  SPISlave.onDataSent([]() {
  });

  SPISlave.onStatus([](uint32_t data) {
  });

  SPISlave.onStatusSent([]() {
  });
  SPISlave.begin();
}

void loop() {}
