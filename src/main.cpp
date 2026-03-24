#include <SPI.h>
#include <MFRC522.h>

void trafficCycle(); void adjustTiming(); void countdown(int t);
void detectOpposite();
void detectApproaching();
void detectStopLineViolation();
void printEChallan(const String &uid);
bool isEmergencyUID(const String &uid);
bool checkEmergencyFromRFIDA();
void forceAllRed();
void enterEmergencyMode(const String &uid);
void handleEmergencyMode();

#define SS_A 5
#define SS_B 17
#define SS_C 16
#define RST_PIN 22

MFRC522 rfidA(SS_A, RST_PIN);
MFRC522 rfidB(SS_B, RST_PIN);
MFRC522 rfidC(SS_C, RST_PIN);

#define RED 25
#define YELLOW 26
#define GREEN 27

const int DEFAULT_GREEN_TIME = 10;
const int DEFAULT_RED_TIME = 20;
const int YELLOW_TIME = 5;

int greenTime = DEFAULT_GREEN_TIME;
int redTime = DEFAULT_RED_TIME;

int maxGreen = 20;
int minRed = 3;

int vehicleCount = 0;
bool redPhase = false;

// Emergency vehicle priority
bool emergencyActive = false;
bool emergencyAbortCycle = false;
String emergencyUID = "";
unsigned long emergencyStartMs = 0;
const unsigned long EMERGENCY_TIMEOUT_MS = 20000;

// Add your emergency vehicle tag UIDs here (AMBULANCE / FIRE / POLICE).
// UIDs must match the value printed by Serial (uppercase hex string).
static const char *EMERGENCY_UIDS[] = {
  "1C284C06",
  "EC6D4D06",
};
static const size_t EMERGENCY_UIDS_COUNT = sizeof(EMERGENCY_UIDS) / sizeof(EMERGENCY_UIDS[0]);

const int MAX_UIDS = 40;
String ignoreUID[MAX_UIDS];
int ignoreCount = 0;
unsigned long ignoreUntil[MAX_UIDS];

const unsigned long OPPOSITE_BLOCK_MS = 30000; // how long an opposite UID stays blocked (ms)

String lastIgnorePrintUID = "";
unsigned long lastIgnorePrintMs = 0;

// Seen lists for e-challan eligibility (must be seen on BOTH A and B, then detected on C)
String seenAUID[MAX_UIDS];
int seenACount = 0;
String seenBUID[MAX_UIDS];
int seenBCount = 0;
String challanUID[MAX_UIDS];
int challanCount = 0;

const int MAX_QUEUE = 40;
String queueUID[MAX_QUEUE];
int queueCount = 0;

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

void removeUID(String list[], int &size, String uid)
{
  for(int i=0;i<size;i++)
  {
    if(list[i]==uid)
    {
      for(int j=i;j<size-1;j++)
        list[j]=list[j+1];
      size--;
      return;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  SPI.begin();

  // Ensure all MFRC522 SS pins are de-selected before init (important for multi-reader SPI)
  pinMode(SS_A, OUTPUT); digitalWrite(SS_A, HIGH);
  pinMode(SS_B, OUTPUT); digitalWrite(SS_B, HIGH);
  pinMode(SS_C, OUTPUT); digitalWrite(SS_C, HIGH);

  rfidA.PCD_Init();
  rfidB.PCD_Init();
  rfidC.PCD_Init();

  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);

  Serial.println("Smart Traffic System Started");
}

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
  if(!rfidA.PICC_IsNewCardPresent()) return false;
  if(!rfidA.PICC_ReadCardSerial()) return false;

  String uid = readUID(rfidA);

  rfidA.PICC_HaltA();
  rfidA.PCD_StopCrypto1();

  if (isEmergencyUID(uid))
  {
    enterEmergencyMode(uid);
    return true;
  }
  return false;
}

void handleEmergencyMode()
{
  // Keep all signals RED until emergency vehicle is seen at RFID-C (passed stop line), or timeout.
  while (emergencyActive)
  {
    forceAllRed();

    // Clear emergency once the same UID is detected at RFID-C.
    if(rfidC.PICC_IsNewCardPresent() && rfidC.PICC_ReadCardSerial())
    {
      String uid = readUID(rfidC);
      rfidC.PICC_HaltA();
      rfidC.PCD_StopCrypto1();

      if (uid.equalsIgnoreCase(emergencyUID))
      {
        Serial.print("Emergency vehicle passed (RFID-C): ");
        Serial.println(uid);
        emergencyActive = false;
        break;
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

void loop()
{
  trafficCycle();
}

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

  digitalWrite(GREEN,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(RED,HIGH);

  redPhase = true;

  Serial.println("RED PHASE - vehicle detection");

  int elapsed = 0;

  while(true)
  {
    // Emergency can arrive any time; check RFID-A first.
    if (checkEmergencyFromRFIDA())
    {
      handleEmergencyMode();
      return;
    }

    detectOpposite();
    detectApproaching();

    if (emergencyActive)
    {
      handleEmergencyMode();
      return;
    }

    detectStopLineViolation();

    adjustTiming();

    int remaining = redTime - elapsed;
    if(remaining <= 0) break;

    Serial.print("RED Timer: ");
    Serial.println(remaining);

    delay(1000);
    elapsed++;
  }

  redPhase = false;

  digitalWrite(RED,LOW);
  digitalWrite(GREEN,HIGH);

  Serial.println("GREEN SIGNAL");

  countdown(greenTime);

  if (emergencyAbortCycle)
    return;

  digitalWrite(GREEN,LOW);
  digitalWrite(YELLOW,HIGH);

  Serial.println("YELLOW SIGNAL");

  countdown(YELLOW_TIME);

  if (emergencyAbortCycle)
    return;

  digitalWrite(YELLOW,LOW);
}

void detectOpposite()
{
  if (!redPhase) return;

  if(!rfidB.PICC_IsNewCardPresent()) return;
  if(!rfidB.PICC_ReadCardSerial()) return;

  String uid = readUID(rfidB);

  rfidB.PICC_HaltA();
  rfidB.PCD_StopCrypto1();

  // If this UID was already seen on RFID-A, then RFID-B is the same-lane "middle" checkpoint,
  // not an opposite vehicle.
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
  if(!rfidA.PICC_IsNewCardPresent()) return;
  if(!rfidA.PICC_ReadCardSerial()) return;

  String uid = readUID(rfidA);

  rfidA.PICC_HaltA();
  rfidA.PCD_StopCrypto1();

  // Emergency vehicle priority triggers immediately.
  if (isEmergencyUID(uid))
  {
    enterEmergencyMode(uid);
    return;
  }

  // If RFID-B already classified this UID as opposite/other-road, do NOT count it as same-lane
  // in the current red phase. It can be counted again in a later cycle if it appears again.
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

  // Mark as seen at RFID-A (far)
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
  int newGreen = DEFAULT_GREEN_TIME + (vehicleCount * 2);
  int newRed = DEFAULT_RED_TIME - (vehicleCount * 2);

  if(newGreen > maxGreen) newGreen = maxGreen;
  if(newRed < minRed) newRed = minRed;

  greenTime = newGreen;
  redTime = newRed;
}

void countdown(int t)
{
  for(int i=t;i>0;i--)
  {
    // Emergency can arrive during GREEN/YELLOW too.
    if (checkEmergencyFromRFIDA())
    {
      handleEmergencyMode();
      return;
    }

    Serial.print("Timer: ");
    Serial.println(i);
    delay(1000);
  }
}

void detectStopLineViolation()
{
  if (!redPhase) return; // violation only during RED

  if(!rfidC.PICC_IsNewCardPresent()) return;
  if(!rfidC.PICC_ReadCardSerial()) return;

  String uid = readUID(rfidC);

  rfidC.PICC_HaltA();
  rfidC.PCD_StopCrypto1();

  // If this UID is currently classified as opposite/other-road, never generate challan for it.
  if (isOppositeBlocked(uid))
  {
    Serial.print("RFID-C detected (opposite/other-road): ");
    Serial.println(uid);
    return;
  }

  // Only generate e-challan if UID was seen on BOTH RFID-A and RFID-B
  bool eligible = uidExists(seenAUID, seenACount, uid) && uidExists(seenBUID, seenBCount, uid);
  if (!eligible)
  {
    Serial.print("RFID-C detected (treated as other-side vehicle): ");
    Serial.println(uid);
    return;
  }

  // Prevent repeated challans for the same UID during the same red phase/cycle
  if (uidExists(challanUID, challanCount, uid))
    return;

  addUID(challanUID, challanCount, uid);
  printEChallan(uid);
}

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