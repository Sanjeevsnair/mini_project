#include <Arduino.h>

//74HC595 pin connections
//DS (14) -> GPIO 23
//SHCP (11) -> GPIO 18
//STCP (12) -> GPIO 5

//mc -> vcc
//oe -> gnd

#define DATA_PIN 23
#define CLOCK_PIN 18
#define LATCH_PIN 5

void displayDigit(byte data);

byte digits[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < 10; i++) {
    displayDigit(digits[i]);
    delay(1000);
  }
}

void displayDigit(byte data) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data);
  digitalWrite(LATCH_PIN, HIGH);
}