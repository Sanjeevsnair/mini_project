#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // CE, CSN

//mosi=23
//miso=19
//sck=18

const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("NRF24 not detected!");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  Serial.println("Transmitter ready");
}

void loop() {
  const char text[] = "hello";
  radio.write(&text, sizeof(text));
  Serial.println("Sent: hello");
  delay(1000);
}