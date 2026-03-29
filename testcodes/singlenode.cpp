#include <SPI.h>
#include <MFRC522.h>

// RFID pins
// =======================================================
// sda = 15
// rst = 27
// sck = 2
// mosi = 0
// miso = 4

// traffic light pins
// =======================================================
// RED = 26
// YELLOW = 25
// GREEN = 33

// 74HC595 pin connections
// =======================================================
// IC 1 pin 9 -> IC 2 pin 14 (DS)
// IC 1 pin 11 -> IC 2 pin 11
// IC 1 pin 12 -> IC 2 pin 12

// DATA_PIN = 23
// CLOCK_PIN = 18
// LATCH_PIN = 5

// =======================================================
// RFID MODE
// =======================================================
#define SINGLE_RFID_SERIAL_SWITCH 1

bool readCardFromReader(MFRC522 &reader, String &uidOut)
{
  if (!reader.PICC_IsNewCardPresent()) return false;
  if (!reader.PICC_ReadCardSerial()) return false;

  String uid = "";
  for (byte i = 0; i < reader.uid.size; i++)
  {
    if (reader.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(reader.uid.uidByte[i], HEX);
  }

  uid.toUpperCase();

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();

  reader.PCD_AntennaOff();
  delay(10);
  reader.PCD_AntennaOn();

  uidOut = uid;
  return true;
}

enum RFIDRole { RFID_ROLE_A, RFID_ROLE_B, RFID_ROLE_C };
static RFIDRole activeRFIDRole = RFID_ROLE_A;

void handleSerialRFIDRoleSwitch()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;

    if (c == 'A' || c == 'a')
    {
      activeRFIDRole = RFID_ROLE_A;
      Serial.println("[RFID] Active role = A (Approaching)");
    }
    else if (c == 'B' || c == 'b')
    {
      activeRFIDRole = RFID_ROLE_B;
      Serial.println("[RFID] Active role = B (Middle/Opposite)");
    }
    else if (c == 'C' || c == 'c')
    {
      activeRFIDRole = RFID_ROLE_C;
      Serial.println("[RFID] Active role = C (Stop-line)");
    }
  }
}

MFRC522 &readerForRole(RFIDRole role);

void trafficCycle();
void adjustTiming();
void countdown(int t);
void detectOpposite();
void detectApproaching();
void detectStopLineViolation();
void printEChallan(const String &uid);
bool isEmergencyUID(const String &uid);
bool checkEmergencyFromRFIDA();
void forceAllRed();
void enterEmergencyMode(const String &uid);
void handleEmergencyMode();

// =======================================================
// RFID PINS
// =======================================================
#define SS_A 15
#define SS_B 17
#define SS_C 16
#define RST_PIN 27

MFRC522 rfidA(SS_A, RST_PIN);
MFRC522 rfidB(SS_B, RST_PIN);
MFRC522 rfidC(SS_C, RST_PIN);

MFRC522 &readerForRole(RFIDRole role)
{
#if SINGLE_RFID_SERIAL_SWITCH
  (void)role;
  return rfidA;
#else
  switch (role)
  {
    case RFID_ROLE_A: return rfidA;
    case RFID_ROLE_B: return rfidB;
    case RFID_ROLE_C: return rfidC;
  }
  return rfidA;
#endif
}

// =======================================================
// TRAFFIC LIGHT PINS
// =======================================================
#define RED 26
#define YELLOW 25
#define GREEN 33

// =======================================================
// 74HC595 PINS
// =======================================================
#define DATA_PIN 23
#define CLOCK_PIN 18
#define LATCH_PIN 5

// =======================================================
// 7 SEGMENT DIGIT MAP (COMMON CATHODE)
// bit order: DP G F E D C B A
// =======================================================
byte digitMap[10] = {
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

// blank display
byte blankCode = 0b00000000;

// =======================================================
// SYSTEM TIMING
// =======================================================
const int DEFAULT_GREEN_TIME = 30;
const int DEFAULT_RED_TIME = 60;
const int YELLOW_TIME = 5;

int greenTime = DEFAULT_GREEN_TIME;
int redTime = DEFAULT_RED_TIME;

int maxGreen = 45;
int minRed = 10;

int vehicleCount = 0;
bool redPhase = false;

// =======================================================
// EMERGENCY
// =======================================================
bool emergencyActive = false;
bool emergencyAbortCycle = false;
String emergencyUID = "";
unsigned long emergencyStartMs = 0;
const unsigned long EMERGENCY_TIMEOUT_MS = 20000;

static const char *EMERGENCY_UIDS[] = {
  "1C284C06",
  "EC6D4D06",
};
static const size_t EMERGENCY_UIDS_COUNT = sizeof(EMERGENCY_UIDS) / sizeof(EMERGENCY_UIDS[0]);

// =======================================================
// UID STORAGE
// =======================================================
const int MAX_UIDS = 40;
String ignoreUID[MAX_UIDS];
int ignoreCount = 0;
unsigned long ignoreUntil[MAX_UIDS];

const unsigned long OPPOSITE_BLOCK_MS = 30000;

String lastIgnorePrintUID = "";
unsigned long lastIgnorePrintMs = 0;

String seenAUID[MAX_UIDS];
int seenACount = 0;
String seenBUID[MAX_UIDS];
int seenBCount = 0;
String challanUID[MAX_UIDS];
int challanCount = 0;

const int MAX_QUEUE = 40;
String queueUID[MAX_QUEUE];
int queueCount = 0;

// =======================================================
// 7 SEGMENT FUNCTIONS
// =======================================================
void setupShiftRegister()
{
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
}

void sendToShiftRegisters(byte leftDigit, byte rightDigit)
{
  digitalWrite(LATCH_PIN, LOW);

  // FIRST shifted byte goes to far IC (LEFT digit)
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, leftDigit);

  // SECOND shifted byte goes to near IC (RIGHT digit)
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, rightDigit);

  digitalWrite(LATCH_PIN, HIGH);
}

void displayNumber(int num)
{
  if (num < 0) num = 0;
  if (num > 99) num = 99;

  int tens = num / 10;
  int ones = num % 10;

  byte leftCode;
  byte rightCode;

  // show leading zero always: 00, 01, 02 ...
  leftCode = digitMap[tens];
  rightCode = digitMap[ones];

  sendToShiftRegisters(leftCode, rightCode);
}

void displayBlank()
{
  sendToShiftRegisters(blankCode, blankCode);
}

// =======================================================
// HELPERS
// =======================================================
String readUID(MFRC522 &reader)
{
  String uid = "";

  for(byte i=0;i<reader.uid.size;i++)
  {
    if(reader.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(reader.uid.uidByte[i],HEX);
  }

  uid.toUpperCase();
  return uid;
}

bool uidExists(String list[], int size, String uid)
{
  for(int i=0;i<size;i++)
    if(list[i]==uid) return true;
  return false;
}

int uidIndex(String list[], int size, const String &uid)
{
  for(int i=0;i<size;i++)
    if(list[i]==uid) return i;
  return -1;
}

void removeIgnoreAt(int idx)
{
  for (int j = idx; j < ignoreCount - 1; j++)
  {
    ignoreUID[j] = ignoreUID[j + 1];
    ignoreUntil[j] = ignoreUntil[j + 1];
  }
  ignoreCount--;
}

void purgeExpiredOpposites()
{
  unsigned long now = millis();
  for (int i = 0; i < ignoreCount; )
  {
    if ((long)(now - ignoreUntil[i]) >= 0)
      removeIgnoreAt(i);
    else
      i++;
  }
}

bool isOppositeBlocked(const String &uid)
{
  purgeExpiredOpposites();
  return uidExists(ignoreUID, ignoreCount, uid);
}

void blockOppositeUID(const String &uid)
{
  purgeExpiredOpposites();
  int idx = uidIndex(ignoreUID, ignoreCount, uid);
  unsigned long until = millis() + OPPOSITE_BLOCK_MS;
  if (idx >= 0)
  {
    ignoreUntil[idx] = until;
    return;
  }
  if (ignoreCount < MAX_UIDS)
  {
    ignoreUID[ignoreCount] = uid;
    ignoreUntil[ignoreCount] = until;
    ignoreCount++;
  }
}

void addUID(String list[], int &size, String uid)
{
  if(size < MAX_UIDS)
    list[size++] = uid;
}

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);

  SPI.begin(2, 4, 0, SS_A);

  pinMode(SS_A, OUTPUT); digitalWrite(SS_A, HIGH);
#if !SINGLE_RFID_SERIAL_SWITCH
  pinMode(SS_B, OUTPUT); digitalWrite(SS_B, HIGH);
  pinMode(SS_C, OUTPUT); digitalWrite(SS_C, HIGH);
#endif

  rfidA.PCD_Init();
#if !SINGLE_RFID_SERIAL_SWITCH
  rfidB.PCD_Init();
  rfidC.PCD_Init();
#endif

  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  setupShiftRegister();
  displayNumber(0);

  Serial.println("Smart Traffic System Started");
#if SINGLE_RFID_SERIAL_SWITCH
  Serial.println("[RFID] Single-reader mode: type A/B/C to select role on SS=15");
#endif
}

// =======================================================
// EMERGENCY FUNCTIONS
// =======================================================
bool isEmergencyUID(const String &uid)
{
  for (size_t i = 0; i < EMERGENCY_UIDS_COUNT; i++)
  {
    if (uid.equalsIgnoreCase(EMERGENCY_UIDS[i]))
      return true;
  }
  return false;
}

void forceAllRed()
{
  digitalWrite(GREEN, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(RED, HIGH);
}

void enterEmergencyMode(const String &uid)
{
  emergencyActive = true;
  emergencyAbortCycle = true;
  emergencyUID = uid;
  emergencyStartMs = millis();
  forceAllRed();

  Serial.println("\n==============================");
  Serial.println("EMERGENCY VEHICLE PRIORITY");
  Serial.print("Emergency UID detected at RFID-A: ");
  Serial.println(uid);
  Serial.println("ACTION: All signals -> RED");
  Serial.println("==============================\n");
}

bool checkEmergencyFromRFIDA()
{
  handleSerialRFIDRoleSwitch();
#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_A)
    return false;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_A), uid))
    return false;

  if (isEmergencyUID(uid))
  {
    enterEmergencyMode(uid);
    return true;
  }
  return false;
}

void handleEmergencyMode()
{
  while (emergencyActive)
  {
    handleSerialRFIDRoleSwitch();
    forceAllRed();

    // display "00" during emergency hold
    displayNumber(0);

    {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_C)
#endif
      {
        String uid;
        if (readCardFromReader(readerForRole(RFID_ROLE_C), uid))
        {
          if (uid.equalsIgnoreCase(emergencyUID))
          {
            Serial.print("Emergency vehicle passed (RFID-C): ");
            Serial.println(uid);
            emergencyActive = false;
            break;
          }
        }
      }
    }

    if (millis() - emergencyStartMs > EMERGENCY_TIMEOUT_MS)
    {
      Serial.println("Emergency mode timeout; resuming normal cycle");
      emergencyActive = false;
      break;
    }

    delay(50);
  }
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop()
{
  trafficCycle();
}

// =======================================================
// TRAFFIC CYCLE
// =======================================================
void trafficCycle()
{
  emergencyAbortCycle = false;

  vehicleCount = 0;
  queueCount = 0;
  purgeExpiredOpposites();
  seenACount = 0;
  seenBCount = 0;
  challanCount = 0;

  greenTime = DEFAULT_GREEN_TIME;
  redTime = DEFAULT_RED_TIME;

  digitalWrite(GREEN, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(RED, HIGH);

  redPhase = true;

  Serial.println("RED PHASE - vehicle detection");

  int elapsed = 0;

  while(true)
  {
#if SINGLE_RFID_SERIAL_SWITCH
    handleSerialRFIDRoleSwitch();
#endif

#if !SINGLE_RFID_SERIAL_SWITCH
    if (checkEmergencyFromRFIDA())
    {
      handleEmergencyMode();
      return;
    }
#endif

    detectOpposite();
    delay(20);
    detectApproaching();
    delay(20);

    if (emergencyActive)
    {
      handleEmergencyMode();
      return;
    }

    detectStopLineViolation();
    delay(20);

    adjustTiming();

    int remaining = redTime - elapsed;
    if(remaining <= 0) break;

    Serial.print("RED Timer: ");
    Serial.println(remaining);

    displayNumber(remaining);

    delay(1000);
    elapsed++;
  }

  redPhase = false;

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);

  Serial.println("GREEN SIGNAL");
  countdown(greenTime);

  if (emergencyAbortCycle) return;

  digitalWrite(GREEN, LOW);
  digitalWrite(YELLOW, HIGH);

  Serial.println("YELLOW SIGNAL");
  countdown(YELLOW_TIME);

  if (emergencyAbortCycle) return;

  digitalWrite(YELLOW, LOW);
}

// =======================================================
// RFID DETECTION
// =======================================================
void detectOpposite()
{
  if (!redPhase) return;

#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_B)
    return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_B), uid))
    return;

  if (uidExists(seenAUID, seenACount, uid))
  {
    if(!uidExists(seenBUID, seenBCount, uid))
    {
      addUID(seenBUID, seenBCount, uid);
      Serial.print("Middle vehicle detected (RFID-B): ");
      Serial.println(uid);
    }
    return;
  }

  if(!uidExists(ignoreUID,ignoreCount,uid))
  {
    blockOppositeUID(uid);
    Serial.print("Opposite vehicle detected: ");
    Serial.println(uid);
  }
}

void detectApproaching()
{
#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_A)
    return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_A), uid))
    return;

  if (isEmergencyUID(uid))
  {
    enterEmergencyMode(uid);
    return;
  }

  if(isOppositeBlocked(uid))
  {
    unsigned long now = millis();
    if (uid != lastIgnorePrintUID || (now - lastIgnorePrintMs) > 1500)
    {
      Serial.print("Opposite vehicle ignored at RFID-A: ");
      Serial.println(uid);
      lastIgnorePrintUID = uid;
      lastIgnorePrintMs = now;
    }
    return;
  }

  if(!uidExists(seenAUID, seenACount, uid))
    addUID(seenAUID, seenACount, uid);

  if(!uidExists(queueUID,queueCount,uid))
  {
    addUID(queueUID,queueCount,uid);
    vehicleCount++;

    Serial.print("Valid vehicle detected: ");
    Serial.println(uid);

    Serial.print("Queue count: ");
    Serial.println(vehicleCount);
  }
}

void adjustTiming()
{
  int newGreen = DEFAULT_GREEN_TIME + (vehicleCount * 3);
  int newRed = DEFAULT_RED_TIME - (vehicleCount * 3);

  if(newGreen > maxGreen) newGreen = maxGreen;
  if(newRed < minRed) newRed = minRed;

  greenTime = newGreen;
  redTime = newRed;
}

void countdown(int t)
{
  for(int i=t;i>0;i--)
  {
    if (checkEmergencyFromRFIDA())
    {
      handleEmergencyMode();
      return;
    }

    Serial.print("Timer: ");
    Serial.println(i);

    displayNumber(i);

    delay(1000);
  }

  displayNumber(0);
}

void detectStopLineViolation()
{
  if (!redPhase) return;

#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_C)
    return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_C), uid))
    return;

  if (isOppositeBlocked(uid))
  {
    Serial.print("RFID-C detected (opposite/other-road): ");
    Serial.println(uid);
    return;
  }

  bool eligible = uidExists(seenAUID, seenACount, uid) && uidExists(seenBUID, seenBCount, uid);
  if (!eligible)
  {
    Serial.print("RFID-C detected (treated as other-side vehicle): ");
    Serial.println(uid);
    return;
  }

  if (uidExists(challanUID, challanCount, uid))
    return;

  addUID(challanUID, challanCount, uid);
  printEChallan(uid);
}

// =======================================================
// SAMPLE DATABASE
// =======================================================
struct VehicleRecord {
  const char *uid;
  const char *vehicleNo;
  const char *owner;
  const char *vehicle;
  const char *phone;
  int fine;
};

static const VehicleRecord VEHICLE_DB[] = {
  {"4615D306", "KL-09-AB-1234", "Arun Kumar", "Honda Activa", "+91-90000-11111", 500},
  {"3511D306", "KL-10-CD-5678", "Priya Sharma", "Hyundai i10", "+91-90000-22222", 1000},
  {"B7A3D506", "KL-11-EF-9012", "Suresh R", "TVS Apache", "+91-90000-33333", 500},
};

static const VehicleRecord* findVehicle(const String &uid)
{
  for (size_t i = 0; i < (sizeof(VEHICLE_DB) / sizeof(VEHICLE_DB[0])); i++)
  {
    if (uid.equalsIgnoreCase(VEHICLE_DB[i].uid))
      return &VEHICLE_DB[i];
  }
  return nullptr;
}

void printEChallan(const String &uid)
{
  Serial.println("\n==============================");
  Serial.println("TRAFFIC LAW VIOLATION DETECTED");
  Serial.println("Violation: Stop line crossed during RED");
  Serial.print("Vehicle RFID UID: ");
  Serial.println(uid);

  const VehicleRecord *rec = findVehicle(uid);
  if (rec)
  {
    Serial.println("Database match: YES");
    Serial.print("Vehicle No: "); Serial.println(rec->vehicleNo);
    Serial.print("Owner: "); Serial.println(rec->owner);
    Serial.print("Vehicle: "); Serial.println(rec->vehicle);
    Serial.print("Phone: "); Serial.println(rec->phone);
    Serial.print("Fine: Rs "); Serial.println(rec->fine);
  }
  else
  {
    Serial.println("Database match: NO (sample record not found)");
    Serial.println("Vehicle No: UNKNOWN");
    Serial.println("Owner: UNKNOWN");
    Serial.println("Fine: Rs 500 (default)");
  }

  Serial.println("E-CHALLAN: GENERATED (SAMPLE)");
  Serial.println("==============================\n");
}