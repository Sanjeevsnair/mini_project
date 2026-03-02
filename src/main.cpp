#include <Arduino.h>

#define RED 16
#define YELLOW 17
#define GREEN 15

int vehicleCount = 0;
bool emergency = false;
bool wrongDirection = false;
bool failSafe = false;

static void printPhaseSeconds(const char* phaseName, int durationMs) {
  Serial.print(phaseName);
  Serial.print(" for ");
  Serial.print(durationMs / 1000.0f, 1);
  Serial.println(" s");
}

static void handleSerialInput() {
  while (Serial.available()) {
    char input = static_cast<char>(Serial.read());

    switch (input) {
      case 'v':
        vehicleCount++;
        Serial.println("Vehicle detected");
        break;

      case 'h':
        vehicleCount += 5;
        Serial.println("Heavy traffic detected");
        break;

      case 'e':
        emergency = true;
        Serial.println("Emergency detected!");
        break;

      case 'r':
        wrongDirection = true;
        Serial.println("Wrong direction detected!");
        break;

      case 's':
        Serial.print("Vehicles: ");
        Serial.println(vehicleCount);
        Serial.print("Emergency: ");
        Serial.println(emergency);
        Serial.print("Wrong direction: ");
        Serial.println(wrongDirection);
        Serial.print("Fail-safe: ");
        Serial.println(failSafe);
        break;

      case 'x':
        vehicleCount = 0;
        emergency = false;
        wrongDirection = false;
        failSafe = false;
        Serial.println("System Reset");
        break;

      case 'f':
        failSafe = true;
        break;
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  digitalWrite(RED, HIGH);

  Serial.println("=== SMART TRAFFIC SYSTEM v2 ===");
  Serial.println("Commands:");
  Serial.println("v = vehicle");
  Serial.println("h = heavy traffic");
  Serial.println("e = emergency");
  Serial.println("r = wrong direction");
  Serial.println("s = status");
  Serial.println("x = reset");
  Serial.println("f = fail-safe");
}

void setLights(uint8_t r, uint8_t y, uint8_t g) {
  digitalWrite(RED, r);
  digitalWrite(YELLOW, y);
  digitalWrite(GREEN, g);
}

static void safetyRedNow() {
  Serial.println("SAFETY RED ACTIVATED");
  printPhaseSeconds("SAFETY RED", 5000);
  wrongDirection = false;
  setLights(HIGH, LOW, LOW);

  unsigned long start = millis();
  while (millis() - start < 5000UL) {
    handleSerialInput();
    if (failSafe) {
      return;
    }
    delay(5);
  }
}

static void emergencyOverrideNow() {
  Serial.println("EMERGENCY OVERRIDE");
  emergency = false;

  setLights(LOW, LOW, HIGH);
  Serial.println("GREEN ON");
  printPhaseSeconds("GREEN", 15000);
  unsigned long start = millis();
  while (millis() - start < 15000UL) {
    handleSerialInput();
    if (wrongDirection) {
      safetyRedNow();
      return;
    }
    if (failSafe) {
      return;
    }
    delay(5);
  }

  setLights(LOW, HIGH, LOW);
  Serial.println("YELLOW ON");
  printPhaseSeconds("YELLOW", 2000);
  start = millis();
  while (millis() - start < 2000UL) {
    handleSerialInput();
    if (wrongDirection) {
      safetyRedNow();
      return;
    }
    if (failSafe) {
      return;
    }
    delay(5);
  }

  setLights(HIGH, LOW, LOW);
  Serial.println("RED ON");
  printPhaseSeconds("RED", 2000);
  start = millis();
  while (millis() - start < 2000UL) {
    handleSerialInput();
    if (wrongDirection) {
      safetyRedNow();
      return;
    }
    if (failSafe) {
      return;
    }
    delay(5);
  }
}

static bool waitWithMonitoring(int durationMs) {
  const unsigned long start = millis();
  while (millis() - start < static_cast<unsigned long>(durationMs)) {
    handleSerialInput();

    if (failSafe) {
      return false;
    }
    if (wrongDirection) {
      safetyRedNow();
      return false;
    }
    if (emergency) {
      emergencyOverrideNow();
      return false;
    }

    delay(5);
  }

  return true;
}

void normalCycle(int greenTime, int redTime) {

  // GREEN PHASE
  setLights(LOW, LOW, HIGH);
  Serial.println("GREEN ON");
  printPhaseSeconds("GREEN", greenTime);
  if (!waitWithMonitoring(greenTime)) return;

  // YELLOW PHASE (fixed safety time)
  setLights(LOW, HIGH, LOW);
  Serial.println("YELLOW ON");
  printPhaseSeconds("YELLOW", 2000);
  if (!waitWithMonitoring(2000)) return;

  // RED PHASE
  setLights(HIGH, LOW, LOW);
  Serial.println("RED ON");
  printPhaseSeconds("RED", redTime);
  (void)waitWithMonitoring(redTime);
}

void failSafeMode() {
  Serial.println("FAIL SAFE MODE ACTIVE");
  Serial.println("Blinking YELLOW: 1.0 s ON / 1.0 s OFF");
  while (failSafe) {
    setLights(LOW, HIGH, LOW);
    (void)waitWithMonitoring(1000);
    setLights(LOW, LOW, LOW);
    (void)waitWithMonitoring(1000);

    if (!failSafe) {
      Serial.println("Exiting Fail Safe");
      setLights(HIGH, LOW, LOW);
    }
  }
}

void loop() {

  handleSerialInput();

  if (failSafe) {
    failSafeMode();
    return;
  }

  if (wrongDirection) {
    safetyRedNow();
    vehicleCount = 0;
    return;
  }

  if (emergency) {
    emergencyOverrideNow();
    vehicleCount = 0;
    return;
  }

  // PRIORITY LOGIC

  int greenTime = 5000;
  int redTime = 5000;

  // Adaptive green based on traffic
  if (vehicleCount > 3) greenTime = 8000;
  if (vehicleCount > 6) greenTime = 12000;

  // Adaptive red balancing (if low traffic, reduce red)
  if (vehicleCount == 0) redTime = 3000;
  if (vehicleCount > 6) redTime = 7000;

  normalCycle(greenTime, redTime);

  vehicleCount = 0;
}