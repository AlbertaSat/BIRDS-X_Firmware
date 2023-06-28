#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// Mode 0
SPIClass spi(PA7, PA6, PA5);
SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0);

// SX1276
// NSS: D6 (SPI_CS)
// DIO0: D10 
// RESET: D9
// DIO1: D7
SX1276 radio = new Module(PB0, PB9, PA8, PB2, spi, spiSettings);

// DIO2: D8
AFSKClient audio(&radio, PB8);

// Setup AX25 and APRS clients
AX25Client ax25(&audio);
APRSClient aprs(&ax25);

void setup() {

  Serial.begin(9600);

  Serial.println("Starting setup...");
  Serial.println(millis());

  // Initalize radio
  int state=radio.beginFSK(915.0);

  Serial.println(millis());

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio initialized for FSK!");
  } else {
    Serial.print("Failed FSK init, code ");
    Serial.println(state);
    while(true);
  }

  state = ax25.begin("VA6MPL", 11, 48);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("AX25 initialized");
  } else {
    Serial.print("Failed AX.25 init, code ");
    Serial.println(state);
    while(true);
  }

  state = aprs.begin('S', true);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("APRS initialized");
  } else {
    Serial.print("Failed APRS init, code ");
    Serial.println(state);
    while(true);
  }

}

void loop() {
  Serial.print(F("Sending APRS packet with position DO33fl at millis: "));
  Serial.println(millis());

  int state = aprs.sendPosition("APL0BX", 0, "5328.7N", "11332.52W", "Test packet");

  Serial.print("Packet sent at millis: ");
  Serial.println(millis());

  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }


  // This code will give a bitstream and clock on the 2 GPIO pins used for AFSK (DIO1 and DIO2)
  // Unfortunately, this is difficult to recover AFSK from
  // https://github.com/jgromes/RadioLib/issues/569
  Serial.print(F("Enabling direct recieve"));
  state = radio.receiveDirect();
  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}