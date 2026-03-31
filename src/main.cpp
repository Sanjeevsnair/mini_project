#include <SPI.h>
#include <MFRC522.h>

// =======================================================
// NODE CONFIGURATION
// =======================================================
#define MY_NODE_ID 2 // CHANGE TO 1 OR 2

#define TXD2 17
#define RXD2 16

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
void runRedPhase();
void runGreenPhase();
void runYellowPhase();
void adjustTiming();
void detectOpposite();
void detectApproaching();
void detectStopLineViolation();
void printEChallan(const String &uid);
bool isEmergencyUID(const String &uid);
void forceAllRed();
void enterEmergencyMode(const String &uid);
void handleEmergencyMode();

// UART COMMUNICATION
void sendNodeData();
void receiveNodeData();
void processNodePacket(const String &data);
void printMyNodeStatus();
void printOtherNodeStatus();
String getCurrentLightState();
void setAllSignalsOff();

// DYNAMIC YELLOW ONLY
int calculateDynamicYellowTime();

// =======================================================
// RFID PINS
// =======================================================
#define SS_A 13
#define SS_B 14
#define SS_C 27
#define RST_PIN 19

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
#define RED 32
#define YELLOW 33
#define GREEN 15

// =======================================================
// 74HC595 PINS
// =======================================================
#define DATA_PIN 18
#define CLOCK_PIN 5
#define LATCH_PIN 4

// =======================================================
// 7 SEGMENT DIGIT MAP (COMMON CATHODE)
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

byte blankCode = 0b00000000;

// =======================================================
// SYSTEM TIMING
// =======================================================
const int DEFAULT_GREEN_TIME = 30;
const int BASE_YELLOW_TIME = 5;
const int DEFAULT_RED_TIME = DEFAULT_GREEN_TIME + BASE_YELLOW_TIME;
const int MIN_GREEN_TIME = 10;
const int MAX_DYNAMIC_YELLOW = 15;

int greenTime = DEFAULT_GREEN_TIME;
int redTime = DEFAULT_RED_TIME;
int dynamicYellowTime = BASE_YELLOW_TIME;

int maxGreen = 45;
int minRed = 15;

int vehicleCount = 0;
bool redPhase = false;
int currentRemainingTimer = 0;

// =======================================================
// EMERGENCY
// =======================================================
bool emergencyActive = false;
String emergencyUID = "";
unsigned long emergencyStartMs = 0;
const unsigned long EMERGENCY_TIMEOUT_MS = 60000; // Increased to 60 seconds
unsigned long emergencyCooldownMs = 0;
String lastEmergencyUID = "";

enum EmergencyTrackingState {
  TRACK_NONE = 0,
  ORIGIN_WAIT_B = 1,
  ORIGIN_WAIT_C = 2,
  ORIGIN_DONE = 3,
  REMOTE_WAIT_C = 4,
  REMOTE_WAIT_B = 5,
  REMOTE_WAIT_A = 6,
  EMERGENCY_CLEARED = 7
};

EmergencyTrackingState emergState = TRACK_NONE;

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
// OTHER NODE DATA
// =======================================================
int otherNodeId = 0;
int otherVehicleDensity = 0;
String otherLightState = "UNKNOWN";
bool otherEmergency = false;
int otherRemainingTimer = 0;
int otherHeartbeat = 0;
String otherEmergencyUID = "NONE";
int otherEmergState = 0;

bool node1Synced = false;
String uart2RxBuffer = "";

unsigned long lastSendTime = 0;
int myHeartbeat = 0;

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
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, rightDigit);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, leftDigit);
  digitalWrite(LATCH_PIN, HIGH);
}

void displayNumber(int num)
{
  if (num < 0) num = 0;
  if (num > 99) num = 99;

  int tens = num / 10;
  int ones = num % 10;

  sendToShiftRegisters(digitMap[tens], digitMap[ones]);
}

void displayBlank()
{
  sendToShiftRegisters(blankCode, blankCode);
}

// =======================================================
// HELPERS
// =======================================================
bool uidExists(String list[], int size, String uid)
{
  for (int i = 0; i < size; i++)
    if (list[i] == uid) return true;
  return false;
}

int uidIndex(String list[], int size, const String &uid)
{
  for (int i = 0; i < size; i++)
    if (list[i] == uid) return i;
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
  if (size < MAX_UIDS)
    list[size++] = uid;
}

String getCurrentLightState()
{
  if (digitalRead(RED) == HIGH) return "RED";
  if (digitalRead(GREEN) == HIGH) return "GREEN";
  if (digitalRead(YELLOW) == HIGH) return "YELLOW";
  return "OFF";
}

void setTrafficLights(bool redOn, bool yellowOn, bool greenOn)
{
  digitalWrite(RED, redOn ? HIGH : LOW);
  digitalWrite(YELLOW, yellowOn ? HIGH : LOW);
  digitalWrite(GREEN, greenOn ? HIGH : LOW);
}

void setGreenSignal()
{
  setTrafficLights(false, false, true);
  redPhase = false;
}

void setYellowSignal()
{
  setTrafficLights(false, true, false);
  redPhase = false;
}

void setRedSignal()
{
  setTrafficLights(true, false, false);
  redPhase = true;
}

void setAllSignalsOff()
{
  setTrafficLights(false, false, false);
  redPhase = false;
}

// =======================================================
// DYNAMIC YELLOW LOGIC
// =======================================================
int calculateDynamicYellowTime()
{
  return BASE_YELLOW_TIME;
}

// =======================================================
// UART COMMUNICATION
// =======================================================
void sendNodeData()
{
  myHeartbeat++;

  String eUid = emergencyUID == "" ? "NONE" : emergencyUID;

  String msg = String(MY_NODE_ID) + "," +
               String(vehicleCount) + "," +
               getCurrentLightState() + "," +
               String(emergencyActive ? 1 : 0) + "," +
               String(currentRemainingTimer) + "," +
               String(myHeartbeat) + "," +
               String(greenTime) + "," +
               String(redTime) + "," +
               eUid + "," +
               String((int)emergState);

  Serial2.println(msg);
}

void processNodePacket(const String &data)
{
  int p1 = data.indexOf(',');
  int p2 = data.indexOf(',', p1 + 1);
  int p3 = data.indexOf(',', p2 + 1);
  int p4 = data.indexOf(',', p3 + 1);
  int p5 = data.indexOf(',', p4 + 1);
  int p6 = data.indexOf(',', p5 + 1);
  int p7 = data.indexOf(',', p6 + 1);
  int p8 = data.indexOf(',', p7 + 1);
  int p9 = data.indexOf(',', p8 + 1);

  if (p1 == -1 || p2 == -1 || p3 == -1 || p4 == -1 || p5 == -1 || p6 == -1 || p7 == -1 || p8 == -1 || p9 == -1)
    return;

  otherNodeId = data.substring(0, p1).toInt();
  otherVehicleDensity = data.substring(p1 + 1, p2).toInt();
  otherLightState = data.substring(p2 + 1, p3);
  otherEmergency = data.substring(p3 + 1, p4).toInt();
  otherRemainingTimer = data.substring(p4 + 1, p5).toInt();
  otherHeartbeat = data.substring(p5 + 1, p6).toInt();

  int remoteGreenTime = data.substring(p6 + 1, p7).toInt();
  int remoteRedTime = data.substring(p7 + 1, p8).toInt();

  otherEmergencyUID = data.substring(p8 + 1, p9);
  otherEmergState = data.substring(p9 + 1).toInt();

  if (otherNodeId == 1)
    node1Synced = true;

  // Real-time synchronization: Accept shrunk times from the RED node
  if (otherLightState == "RED" && !emergencyActive && !otherEmergency)
  {
    greenTime = remoteGreenTime;
    redTime   = remoteRedTime;
  }

  if (otherEmergency)
  {
    if (!emergencyActive)
    {
      // Cooldown check for remote triggers too!
      if (otherEmergencyUID.equalsIgnoreCase(lastEmergencyUID) && (millis() - emergencyCooldownMs < 30000))
      {
          // Skip triggering emergency mode, ignore stale remote messages
      }
      else
      {
        emergencyActive = true;
        emergState = REMOTE_WAIT_C;
        emergencyUID = otherEmergencyUID;
        emergencyStartMs = millis(); // IMPORTANT: Reset timer for remote node
        forceAllRed();

        Serial.println("\n==================================================");
        Serial.println("!!! EMERGENCY INCOMING FROM OTHER NODE !!!");
        Serial.printf("Emergency Vehicle ID [%s] detected on Opposite Node.\n", emergencyUID.c_str());
        Serial.println("Tracking C -> B -> A for exit.");
        Serial.println("==================================================\n");
      }
    }
    else
    {
        // Update timeout while still receiving emergency signals
        emergencyStartMs = millis();
    }
  }
}

void receiveNodeData()
{
  if (Serial2.available())
  {
    while (Serial2.available())
    {
      char incoming = (char)Serial2.read();

      if (incoming == '\r') continue;

      if (incoming == '\n')
      {
        String data = uart2RxBuffer;
        uart2RxBuffer = "";
        data.trim();

        if (data.length() > 0)
          processNodePacket(data);

        continue;
      }

      if (uart2RxBuffer.length() < 120)
        uart2RxBuffer += incoming;
    }
  }
}

// =======================================================
// SETUP
// =======================================================
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // SPI.begin(SCK, MISO, MOSI, SS);
  SPI.begin(23, 21, 22, SS_A);

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

  setAllSignalsOff();
  delay(200);

  setupShiftRegister();
  displayNumber(0);

  Serial.println("\n======================================");
  Serial.println(" SMART TRAFFIC SYSTEM STARTED");
  Serial.println("======================================");
}

// =======================================================
// EMERGENCY FUNCTIONS
// =======================================================
bool isEmergencyUID(const String &uid)
{
  for (size_t i = 0; i < EMERGENCY_UIDS_COUNT; i++)
  {
    if (uid.equalsIgnoreCase(EMERGENCY_UIDS[i])) return true;
  }
  return false;
}

void forceAllRed() { setRedSignal(); }

void enterEmergencyMode(const String &uid)
{
  emergencyActive = true;
  emergencyUID = uid;
  emergState = ORIGIN_WAIT_B;
  emergencyStartMs = millis();
  forceAllRed();

  Serial.println("\n==================================================");
  Serial.printf("[EMERGENCY] Vehicle ID %s detected on our node!\n", uid.c_str());
  Serial.println("ALL LIGHTS TO RED. Tracking A -> B -> C.\n==================================================\n");
}

void handleEmergencyMode()
{
  while (emergencyActive || otherEmergency)
  {
    handleSerialRFIDRoleSwitch();
    forceAllRed();
    displayNumber(0);
    currentRemainingTimer = 0;

    sendNodeData();
    receiveNodeData();

    // Check completion condition
    if (emergState == EMERGENCY_CLEARED || otherEmergState == EMERGENCY_CLEARED) {
       Serial.println("\n*** EMERGENCY VEHICLE HAS LEFT THE JUNCTION ***");
       Serial.println("*** RESUMING PREVIOUS TRAFFIC STATE ***\n");
       
       // Record cooldown BEFORE clearing variables!
       lastEmergencyUID = emergencyUID;
       emergencyCooldownMs = millis();
       
       emergencyActive = false;
       otherEmergency = false;
       emergState = TRACK_NONE;
       emergencyUID = "";
       
       // Ignore stale events from UART 
       uart2RxBuffer = ""; 
       break;
    }

    // Time out protection
    if (millis() - emergencyStartMs > EMERGENCY_TIMEOUT_MS)
    {
       Serial.println("\n*** EMERGENCY TIMEOUT - RESUMING PREVIOUS STATE ***\n");
       
       lastEmergencyUID = emergencyUID;
       emergencyCooldownMs = millis();
       
       emergencyActive = false;
       otherEmergency = false;
       emergState = TRACK_NONE;
       emergencyUID = "";
       break;
    }

    String uid;
    
    // Tracking logic for Origin Node
    if (emergState == ORIGIN_WAIT_B) {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_B)
#endif
      {
         if (readCardFromReader(readerForRole(RFID_ROLE_B), uid) && uid.equalsIgnoreCase(emergencyUID)) {
             emergState = ORIGIN_WAIT_C;
             Serial.printf("[EMERGENCY] Origin Node: Vehicle %s successfully passed RFID B!\n", uid.c_str());
         }
      }
    }
    else if (emergState == ORIGIN_WAIT_C) {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_C)
#endif
      {
         if (readCardFromReader(readerForRole(RFID_ROLE_C), uid) && uid.equalsIgnoreCase(emergencyUID)) {
             emergState = ORIGIN_DONE;
             Serial.printf("[EMERGENCY] Origin Node: Vehicle %s successfully passed RFID C! Waiting for opposite node...\n", uid.c_str());
         }
      }
    }

    // Tracking logic for Remote Node
    if (emergState == REMOTE_WAIT_C) {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_C)
#endif
      {
         if (readCardFromReader(readerForRole(RFID_ROLE_C), uid) && uid.equalsIgnoreCase(emergencyUID)) {
             emergState = REMOTE_WAIT_B;
             Serial.printf("[EMERGENCY] Remote Node: Vehicle %s successfully entered at RFID C!\n", uid.c_str());
         }
      }
    }
    else if (emergState == REMOTE_WAIT_B) {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_B)
#endif
      {
         if (readCardFromReader(readerForRole(RFID_ROLE_B), uid) && uid.equalsIgnoreCase(emergencyUID)) {
             emergState = REMOTE_WAIT_A;
             Serial.printf("[EMERGENCY] Remote Node: Vehicle %s successfully passed RFID B!\n", uid.c_str());
         }
      }
    }
    else if (emergState == REMOTE_WAIT_A) {
#if SINGLE_RFID_SERIAL_SWITCH
      if (activeRFIDRole == RFID_ROLE_A)
#endif
      {
         if (readCardFromReader(readerForRole(RFID_ROLE_A), uid) && uid.equalsIgnoreCase(emergencyUID)) {
             emergState = EMERGENCY_CLEARED;
             Serial.printf("[EMERGENCY] Remote Node: Vehicle %s successfully passed RFID A! Clear!\n", uid.c_str());
         }
      }
    }

    delay(20);
  }
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop()
{
  receiveNodeData();

  if (MY_NODE_ID != 1 && !node1Synced)
  {
    setAllSignalsOff();
    displayBlank();
    delay(100);
    return;
  }

  // Handle emergency outside traffic cycle if needed, but normally phases handle it
  if (emergencyActive || otherEmergency)
  {
    handleEmergencyMode();
    return; // Returns to main loop and re-evaluates
  }

  trafficCycle();
}

// =======================================================
// TRAFFIC CYCLE PHASES (REAL-TIME MILLIS ARCHITECTURE)
// =======================================================
void runRedPhase()
{
  setRedSignal();
  Serial.println("\n[ RED PHASE STARTED ]");

  unsigned long phaseStartMs = millis();
  int lastDisplay = -1;

  while (true)
  {
    receiveNodeData();
#if SINGLE_RFID_SERIAL_SWITCH
    handleSerialRFIDRoleSwitch();
#endif

    detectOpposite();
    detectApproaching();
    detectStopLineViolation();

    int elapsedSec = (millis() - phaseStartMs) / 1000;
    int remaining = redTime - elapsedSec;

    if (emergencyActive || otherEmergency) 
    { 
      handleEmergencyMode(); 
      // Resume timer seamlessly
      phaseStartMs = millis() - ((redTime - remaining) * 1000);
      setRedSignal();
      lastDisplay = -1;
      continue;
    }

    if (remaining <= 0) break;

    // Instantly fires if 1 second passes OR if redTime shrinks due to a vehicle
    if (remaining != lastDisplay)
    {
      currentRemainingTimer = remaining;
      displayNumber(remaining);
      sendNodeData(); // Broadcast new jump instantly
      
      Serial.println("\n--------------------------------------------------");
      Serial.printf("[NODE %d] | STATE: RED   | Timer: %d / %d s\n", MY_NODE_ID, remaining, redTime);
      Serial.printf("OUR LANE | Cars: %d    | OTHER NODE: %s (Timer: %d, Cars: %d)\n", 
                    vehicleCount, otherLightState.c_str(), otherRemainingTimer, otherVehicleDensity);
      Serial.println("--------------------------------------------------");
      
      lastDisplay = remaining;
    }

    delay(20); // Smooth 50Hz scanning loop
  }

  currentRemainingTimer = 0;
  displayNumber(0);
}

void runGreenPhase()
{
  setGreenSignal();
  Serial.println("\n[ GREEN PHASE STARTED ]");

  unsigned long phaseStartMs = millis();
  int lastDisplay = -1;

  while (true)
  {
    receiveNodeData(); // Instantly catches shrinking times from the RED node
#if SINGLE_RFID_SERIAL_SWITCH
    handleSerialRFIDRoleSwitch();
#endif
    
    detectApproaching(); // Can detect emergency vehicles during Green

    int elapsedSec = (millis() - phaseStartMs) / 1000;
    int remaining = greenTime - elapsedSec; // Recalculates dynamically every 20ms

    if (emergencyActive || otherEmergency) 
    { 
      handleEmergencyMode(); 
      phaseStartMs = millis() - ((greenTime - remaining) * 1000);
      setGreenSignal();
      lastDisplay = -1;
      continue;
    }

    if (remaining <= 0) break;

    if (remaining != lastDisplay)
    {
      currentRemainingTimer = remaining;
      displayNumber(remaining);
      sendNodeData();
      
      Serial.println("\n--------------------------------------------------");
      Serial.printf("[NODE %d] | STATE: GREEN | Timer: %d / %d s\n", MY_NODE_ID, remaining, greenTime);
      Serial.printf("OUR LANE | Cars: %d    | OTHER NODE: %s (Timer: %d, Cars: %d)\n", 
                    vehicleCount, otherLightState.c_str(), otherRemainingTimer, otherVehicleDensity);
      Serial.println("--------------------------------------------------");
      
      lastDisplay = remaining;
    }

    delay(20);
  }

  currentRemainingTimer = 0;
  displayNumber(0);
}

void runYellowPhase()
{
  setYellowSignal();
  Serial.println("\n[ YELLOW PHASE STARTED ]");
  dynamicYellowTime = calculateDynamicYellowTime();

  unsigned long phaseStartMs = millis();
  int lastDisplay = -1;

  while (true)
  {
    receiveNodeData();
#if SINGLE_RFID_SERIAL_SWITCH
    handleSerialRFIDRoleSwitch();
#endif
    
    detectApproaching();

    int elapsedSec = (millis() - phaseStartMs) / 1000;
    int remaining = dynamicYellowTime - elapsedSec;

    if (emergencyActive || otherEmergency) 
    { 
      handleEmergencyMode(); 
      phaseStartMs = millis() - ((dynamicYellowTime - remaining) * 1000);
      setYellowSignal();
      lastDisplay = -1;
      continue;
    }

    if (remaining <= 0) break;

    if (remaining != lastDisplay)
    {
      currentRemainingTimer = remaining;
      displayNumber(remaining);
      sendNodeData();
      
      Serial.println("\n--------------------------------------------------");
      Serial.printf("[NODE %d] | STATE: YELLOW| Timer: %d / %d s\n", MY_NODE_ID, remaining, dynamicYellowTime);
      Serial.printf("OUR LANE | Cars: %d    | OTHER NODE: %s (Timer: %d, Cars: %d)\n", 
                    vehicleCount, otherLightState.c_str(), otherRemainingTimer, otherVehicleDensity);
      Serial.println("--------------------------------------------------");
      
      lastDisplay = remaining;
    }

    delay(20);
  }

  currentRemainingTimer = 0;
  displayNumber(0);
}

void trafficCycle()
{
  vehicleCount = 0;
  queueCount = 0;
  purgeExpiredOpposites();
  seenACount = 0;
  seenBCount = 0;
  challanCount = 0;

  greenTime = DEFAULT_GREEN_TIME;
  redTime = DEFAULT_RED_TIME;
  dynamicYellowTime = BASE_YELLOW_TIME;

  if (MY_NODE_ID == 1)
  {
    runRedPhase();    
    runGreenPhase();  
    runYellowPhase(); 
  }
  else 
  {
    runGreenPhase();  
    runYellowPhase(); 
    runRedPhase();    
  }
} // =======================================================
// RFID DETECTION & ADJUSTMENT
// =======================================================
void detectOpposite()
{
  if (!redPhase) return;
#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_B) return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_B), uid)) return;

  if (uidExists(seenAUID, seenACount, uid))
  {
    if (!uidExists(seenBUID, seenBCount, uid)) 
    {
      addUID(seenBUID, seenBCount, uid);
      Serial.printf("[RFID B] Vehicle %s VERIFIED as our lane vehicle (seen at A previously).\n", uid.c_str());
    }
    return;
  }

  if (!uidExists(ignoreUID, ignoreCount, uid)) 
  {
    blockOppositeUID(uid);
    Serial.printf("[RFID B] Vehicle %s detected FIRST on B. Opposite vehicle, SKIPPING.\n", uid.c_str());
  }
}

void detectApproaching()
{
#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_A) return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_A), uid)) return;

  if (isEmergencyUID(uid))
  {
    // Check if this emergency vehicle was just processed
    if (uid.equalsIgnoreCase(lastEmergencyUID) && (millis() - emergencyCooldownMs < 30000))
    {
       // Skip triggering again if within 30 seconds
       return;
    }

    enterEmergencyMode(uid);
    return;
  }

  // ONLY PROCESS NORMAL VEHICLES DURING RED PHASE!
  if (!redPhase) return;

  if (isOppositeBlocked(uid)) 
  {
    // Optionally print here, but might spam. Print once if needed.
    // Serial.printf("[RFID A] Vehicle %s ignored (Detected as opposite vehicle earlier)\n", uid.c_str());
    return;
  }

  if (!uidExists(seenAUID, seenACount, uid)) 
  {
    addUID(seenAUID, seenACount, uid);
    Serial.printf("[RFID A] Vehicle %s detected FIRST in our lane.\n", uid.c_str());
  }

  if (!uidExists(queueUID, queueCount, uid))
  {
    addUID(queueUID, queueCount, uid);
    vehicleCount++;
    Serial.printf("[RFID A] Our Lane Density updated: %d\n", vehicleCount);
    adjustTiming(); // Shrinks the time
    sendNodeData(); // Instantly transmits to the other node
  }
}

void adjustTiming()
{
  if (!redPhase) return;

  int reduction = vehicleCount * 3;
  int newGreen = DEFAULT_GREEN_TIME - reduction;
  if (newGreen < MIN_GREEN_TIME) newGreen = MIN_GREEN_TIME;
  
  int newRed = newGreen + BASE_YELLOW_TIME;

  greenTime = newGreen;
  redTime   = newRed;
  
  // No need to manually update 7-segment here, the millis() loop will catch it instantly!
}

void detectStopLineViolation()
{
  if (!redPhase) return;
#if SINGLE_RFID_SERIAL_SWITCH
  if (activeRFIDRole != RFID_ROLE_C) return;
#endif

  String uid;
  if (!readCardFromReader(readerForRole(RFID_ROLE_C), uid)) return;
  
  if (isOppositeBlocked(uid)) return;

  bool eligible = uidExists(seenAUID, seenACount, uid) && uidExists(seenBUID, seenBCount, uid);
  if (!eligible) 
  {
    if (!uidExists(ignoreUID, ignoreCount, uid)) 
    {
      blockOppositeUID(uid);
      Serial.printf("[RFID C] Vehicle %s detected without A/B sequence. Opposite vehicle, SKIPPING.\n", uid.c_str());
    }
    return;
  }

  if (uidExists(challanUID, challanCount, uid)) return;

  addUID(challanUID, challanCount, uid);
  Serial.printf("\n[!!!] TRAFFIC VIOLATION [!!!]\nVehicle %s crossed the STOP LINE during RED light!\n", uid.c_str());
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
};

static const VehicleRecord* findVehicle(const String &uid)
{
  for (size_t i = 0; i < (sizeof(VEHICLE_DB) / sizeof(VEHICLE_DB[0])); i++)
  {
    if (uid.equalsIgnoreCase(VEHICLE_DB[i].uid)) return &VEHICLE_DB[i];
  }
  return nullptr;
}

void printEChallan(const String &uid)
{
  Serial.println("\n--- CHALLAN GENERATED ---");
  Serial.println(uid);
}