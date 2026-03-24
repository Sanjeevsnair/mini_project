#include <Arduino.h>

//74HC595 pin connections
//IC 1 pin 9 -> IC 2 pin 14 (DS)
//IC 1 pin 11 -> IC 2 pin 11
//IC 1 pin 12 -> IC 2 pin 12

#define DATA_PIN 23
#define CLOCK_PIN 18
#define LATCH_PIN 5
void displayNumber(int num);
void handleSerialInput();


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

int countdownSeconds = 20;
unsigned long lastTickMillis = 0;
unsigned long lastSerialActivityMillis = 0;
String serialBuffer;


void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Send a number to add seconds to the countdown.");
  displayNumber(countdownSeconds);
  lastTickMillis = millis();
}

void loop() {
  handleSerialInput();

  unsigned long currentMillis = millis();
  if (countdownSeconds > 0 && currentMillis - lastTickMillis >= 1000) {
    lastTickMillis += 1000;
    countdownSeconds--;
    displayNumber(countdownSeconds);
  }
}

void handleSerialInput() {
  bool receivedAnySerialData = false;

  while (Serial.available() > 0) {
    receivedAnySerialData = true;
    char incomingChar = Serial.read();
    lastSerialActivityMillis = millis();

    if (incomingChar == '\n' || incomingChar == '\r') {
      if (serialBuffer.length() > 0) {
        int secondsToAdd = serialBuffer.toInt();
        serialBuffer = "";

        if (secondsToAdd > 0) {
          countdownSeconds += secondsToAdd;
          if (countdownSeconds > 99) {
            countdownSeconds = 99;
          }
          displayNumber(countdownSeconds);
          Serial.print("Added seconds: ");
          Serial.println(secondsToAdd);
          Serial.print("Countdown: ");
          Serial.println(countdownSeconds);
        }
      }
    } else if (isDigit(incomingChar)) {
      serialBuffer += incomingChar;
    }
  }

  if (!receivedAnySerialData && serialBuffer.length() > 0 && millis() - lastSerialActivityMillis > 250) {
    int secondsToAdd = serialBuffer.toInt();
    serialBuffer = "";

    if (secondsToAdd > 0) {
      countdownSeconds += secondsToAdd;
      if (countdownSeconds > 99) {
        countdownSeconds = 99;
      }
      displayNumber(countdownSeconds);
      Serial.print("Added seconds: ");
      Serial.println(secondsToAdd);
      Serial.print("Countdown: ");
      Serial.println(countdownSeconds);
    }
  }
}

void displayNumber(int num) {
  if (num < 0) {
    num = 0;
  }
  if (num > 99) {
    num = 99;
  }

  int tens = num / 10;   // LEFT digit (IC2)
  int ones = num % 10;   // RIGHT digit (IC1)

  digitalWrite(LATCH_PIN, LOW);

  // 🔴 send LEFT digit first → goes to SECOND IC
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, digits[tens]);

  // 🔵 send RIGHT digit → stays in FIRST IC
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, digits[ones]);

  digitalWrite(LATCH_PIN, HIGH);
}