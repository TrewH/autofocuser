/**
 * Bare-bones A4988 + AccelStepper firmware with soft limits and EEPROM zero.
 * Commands are ASCII lines over Serial (115200).
 *
 * Dependencies:
 *   - AccelStepper (Arduino Library Manager)
 *
 * Safety/Notes:
 *   - Soft limits are enforced in *user coordinates* (zeroed frame).
 *   - EEPROM writes are throttled (on motion complete and every SAVE_INTERVAL_MS).
 */

#include <AccelStepper.h>
#include <EEPROM.h>

// -------------------- Pin config --------------------
constexpr uint8_t STEP_PIN   = 2;
constexpr uint8_t DIR_PIN    = 3;
constexpr uint8_t ENABLE_PIN = 8;  // LOW = enabled on most A4988 carriers

constexpr int PIN_MS1 = 9;
constexpr int PIN_MS2 = 10;
constexpr int PIN_MS3 = 11;

// -------------------- Motion config defaults --------------------
constexpr float DEFAULT_MAX_SPEED   = 500.0f;  // steps/s
constexpr float DEFAULT_ACCEL       = 500.0f;   // steps/s^2

// -------------------- EEPROM layout --------------------
struct Persist{
  uint16_t magic;        // 0xBEEF
  int32_t  zero_offset;  // driver steps that correspond to user coord 0
  int32_t  min_limit;    // user coord min
  int32_t  max_limit;    // user coord max
  int32_t  last_user_pos;// last known user coord (for fast resume)
  uint8_t  checksum;     // simple xor checksum over payload bytes
};

Persist cfg;  // current persistent config/state


// Where to store Persist in EEPROM:
constexpr int EEPROM_ADDR = 0;
constexpr uint16_t MAGIC  = 0xBEEF;

// -------------------- Globals --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Position bookkeeping
// AccelStepper uses "driver steps" (its internal coordinate frame).
// We expose "user coords" to ROS such that user 0 is wherever cfg.zero_offset is.
inline int32_t userToDriver(int32_t userPos) {
  // driver = user + zero_offset
  return userPos + cfg.zero_offset;
}
inline int32_t driverToUser(long driverPos) {
  // user = driver - zero_offset
  return static_cast<int32_t>(driverPos) - cfg.zero_offset;
}

// EEPROM write throttling
constexpr unsigned long SAVE_INTERVAL_MS = 2000; // also saves on stop
unsigned long lastSaveMs = 0;
int32_t lastSavedUserPos = 0;

// Serial command buffer
String rxLine;

// -------------------- Helpers --------------------
uint8_t calcChecksum(const Persist& p) {
  const uint8_t* b = reinterpret_cast<const uint8_t*>(&p);
  size_t n = sizeof(Persist) - sizeof(p.checksum);
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum ^= b[i];
  return sum;
}

void loadPersist() {
  EEPROM.get(EEPROM_ADDR, cfg);
  if (cfg.magic != MAGIC || cfg.checksum != calcChecksum(cfg)) {
    // Defaults if EEPROM empty/corrupt
    cfg.magic        = MAGIC;
    cfg.zero_offset  = 0;
    cfg.min_limit    = -100000; // generous default bounds
    cfg.max_limit    =  100000;
    cfg.last_user_pos=  0;
    cfg.checksum     = calcChecksum(cfg);
    EEPROM.put(EEPROM_ADDR, cfg);
  }
}

void savePersist() {
  cfg.checksum = calcChecksum(cfg);
  EEPROM.put(EEPROM_ADDR, cfg);
}

void maybeAutoSave() {
  const unsigned long now = millis();
  const int32_t userPos = driverToUser(stepper.currentPosition());
  if ((now - lastSaveMs) >= SAVE_INTERVAL_MS && userPos != lastSavedUserPos && stepper.distanceToGo() == 0) {
    cfg.last_user_pos = userPos;
    savePersist();
    lastSavedUserPos = userPos;
    lastSaveMs = now;
  }
}

// Clamp a requested user coordinate to soft limits.
int32_t clampToLimits(int32_t userPos) {
  if (userPos < cfg.min_limit) return cfg.min_limit;
  if (userPos > cfg.max_limit) return cfg.max_limit;
  return userPos;
}

// -------------------- Motion --------------------
void setTargetUser(int32_t userTarget) {
  const int32_t clamped = clampToLimits(userTarget);
  stepper.moveTo(userToDriver(clamped));
}

void stopNow() {
  // Stop as fast as acceleration allows
  stepper.stop(); // non-blocking; target set such that it decelerates to stop
}

void hardStop() {
  // Immediate stop (no decel); not generally recommended
  stepper.setCurrentPosition(stepper.currentPosition());
}

// -------------------- Serial Command Handling --------------------
// Commands are lines of ASCII: COMMAND [args...]
// Return lines start with "OK" or "ERR", plus a brief message or value.
// Keep this tiny so ROS can be the brains.

void printStatus() {
  const long drvCur = stepper.currentPosition();
  const long drvTar = stepper.targetPosition();
  const int32_t usrCur = driverToUser(drvCur);
  const int32_t usrTar = driverToUser(drvTar);
  Serial.print(F("OK STATUS cur="));
  Serial.print(usrCur);
  Serial.print(F(" target="));
  Serial.print(usrTar);
  Serial.print(F(" dist="));
  Serial.print(stepper.distanceToGo());
  Serial.print(F(" min="));
  Serial.print(cfg.min_limit);
  Serial.print(F(" max="));
  Serial.print(cfg.max_limit);
  Serial.print(F(" zero_offset="));
  Serial.println(cfg.zero_offset);
}

void handleCommand(const String& line) {
  // Tokenize by spaces
  // NOTE: For robustness you could implement a small state machine; this is fine for short lines.
  int firstSpace = line.indexOf(' ');
  String cmd = (firstSpace < 0) ? line : line.substring(0, firstSpace);
  String args = (firstSpace < 0) ? ""   : line.substring(firstSpace + 1);

  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println(F("OK PONG"));
  }
  else if (cmd == "ENABLE") {
    digitalWrite(ENABLE_PIN, LOW); // enable driver
    Serial.println(F("OK ENABLED"));
  }
  else if (cmd == "DISABLE") {
    digitalWrite(ENABLE_PIN, HIGH); // disable driver
    Serial.println(F("OK DISABLED"));
  }
  else if (cmd == "SETSPEED") {
    float v = args.toFloat();
    if (v <= 0) { Serial.println(F("ERR speed>0")); return; }
    stepper.setMaxSpeed(v);
    Serial.println(F("OK"));
  }
  else if (cmd == "SETACCEL") {
    float a = args.toFloat();
    if (a <= 0) { Serial.println(F("ERR accel>0")); return; }
    stepper.setAcceleration(a);
    Serial.println(F("OK"));
  }
  else if (cmd == "GOTO") {
    // GOTO <user_steps>
    char *endp = nullptr;
    long val = strtol(args.c_str(), &endp, 10);
    setTargetUser((int32_t)val);
    Serial.println(F("OK MOVING"));
  }
  else if (cmd == "GOTOR") {
    // GOTOR <delta_steps> (relative, in user coords)
    char *endp = nullptr;
    long delta = strtol(args.c_str(), &endp, 10);
    int32_t currentUser = driverToUser(stepper.currentPosition());
    setTargetUser(currentUser + (int32_t)delta);
    Serial.println(F("OK MOVING"));
  }
  else if (cmd == "WHERE") {
    int32_t pos = driverToUser(stepper.currentPosition());
    Serial.print(F("OK "));
    Serial.println(pos);
  }
  else if (cmd == "AT") {
    long dist = stepper.distanceToGo();
    Serial.print(F("OK "));
    Serial.println(dist);
  }
  else if (cmd == "STOP") {    // soft stop (decelerate)
    stopNow();
    Serial.println(F("OK STOPPING"));
  }
  else if (cmd == "HARDSTOP") {// hard stop (no decel)
    hardStop();
    Serial.println(F("OK HARDSTOP"));
  }
  else if (cmd == "SETZERO") {
    // SETZERO [user_pos]; default 0
    // Redefine user=0 at current driver position or at specified user position.
    // Implementation: set zero_offset so that driverToUser(current)=desired_user
    long desired = 0;
    if (args.length() > 0) {
      desired = strtol(args.c_str(), nullptr, 10);
    }
    long drv = stepper.currentPosition();
    cfg.zero_offset = (int32_t)(drv - desired);
    cfg.last_user_pos = driverToUser(drv);
    savePersist();
    lastSavedUserPos = cfg.last_user_pos;
    Serial.println(F("OK ZERO_SET"));
  }
  else if (cmd == "SETLIMITS") {
    // SETLIMITS <min> <max>   (user coords)
    int sp = args.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR two args")); return; }
    long mn = strtol(args.substring(0, sp).c_str(), nullptr, 10);
    long mx = strtol(args.substring(sp + 1).c_str(), nullptr, 10);
    if (mn >= mx) { Serial.println(F("ERR min<max")); return; }
    cfg.min_limit = (int32_t)mn;
    cfg.max_limit = (int32_t)mx;
    savePersist();
    Serial.println(F("OK LIMITS_SET"));
  }
  else if (cmd == "SAVE") {
    cfg.last_user_pos = driverToUser(stepper.currentPosition());
    savePersist();
    lastSavedUserPos = cfg.last_user_pos;
    Serial.println(F("OK SAVED"));
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "HELP") {
    Serial.println(F("OK CMDS: PING, ENABLE, DISABLE, SETSPEED <s>, SETACCEL <a>, "
                     "GOTO <pos>, GOTOR <d>, WHERE, AT, STOP, HARDSTOP, "
                     "SETZERO [pos], SETLIMITS <min> <max>, SAVE, STATUS, HELP"));
  }
  else {
    Serial.println(F("ERR unknown"));
  }
}

// -------------------- Arduino lifecycle --------------------
void setup() {
  // Serial
  Serial.begin(115200);
  while (!Serial) { /* wait on native USB */ }

  // Pins
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // start disabled

  // Optional microstep control (1/16)
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);
  digitalWrite(PIN_MS1, HIGH);
  digitalWrite(PIN_MS2, HIGH);
  digitalWrite(PIN_MS3, HIGH);

  // Stepper driver polarity
  stepper.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper.setAcceleration(DEFAULT_ACCEL);

  // Restore persisted config
  loadPersist();

  // Rebase current position to the last saved user position so ROS can resume smoothly.
  // Set AccelStepper's driver frame accordingly:
  // driver = user + zero_offset
  long driverResume = userToDriver(cfg.last_user_pos);
  stepper.setCurrentPosition(driverResume);

  lastSavedUserPos = cfg.last_user_pos;
  lastSaveMs = millis();

  Serial.println(F("OK READY"));
  printStatus();
}

void loop() {
  // Non-blocking stepper update
  stepper.run();

  // Periodic autosave when stopped and position changed
  maybeAutoSave();

  // Read line-based serial protocol
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxLine.trim();
      if (rxLine.length() > 0) {
        handleCommand(rxLine);
      }
      rxLine = "";
    } else {
      // small guard
      if (rxLine.length() < 64) rxLine += c;
    }
  }
}
