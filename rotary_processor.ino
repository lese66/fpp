/*
  PURPOSE   Drive a film/paper processor for multiple JOBO tanks (2500-2800 series)

  SOURCES
            Resistor  https://www.hackster.io/mdraber/using-rotary-encoders-with-arduino-interrupts-db3699#code
            LCD       https://dronebotworkshop.com/lcd-displays-arduino/
                      https://docs.arduino.cc/learn/electronics/lcd-displays
                      I2C Scanner: Nick Gammon
            Motors    https://dronebotworkshop.com/dc-motor-drivers/#Large_Motor_Drivers
            H-Bridge  IBT-2 (BTS7960) https://dronebotworkshop.com
            Keypad    https://dronebotworkshop.com/keypads-arduino/
                      https://forum.arduino.cc/t/using-a-4-by4-keypad-to-get-an-integer/195484/9
                      https://forum.arduino.cc/t/solved-keypad-number-input-and-store/57566/25
                      https://forum.arduino.cc/t/setting-values-in-program-using-keypad/577913/15
                      https://forum.arduino.cc/t/setting-values-in-program-using-keypad/577913/5
                      ** https://forum.arduino.cc/t/converting-keypad-entry-string-to-integer/1037087
            States    https://forum.arduino.cc/t/control-speed-and-timing-for-dc-motor-using-keypad-input-and-shown-on-lcd-disply/385810/11
                      by sterretje
            Timer     https://forum.arduino.cc/t/millis-vs-timestamp/885015

  Copyright (c) 2024 Luis Samaniego
  Licensed under the GNU License. See LICENSE file in the project root for details.
  AUTHOR    Luis Samaniego
  HISTORY   2024/01   v3.0  Original code
            2025/12   v4.0  Bugs and improvements: Beep 5s before end, Menu persistence
                            Potentiometer controls motor while running,
                            Development time stored in EEPROM (1..9)
                            Blacklight
            2025/12   v5.x  Temperature control with 3 waterproof DS18B20 sensors.
                            Implemantation in x phases
                      v5.1.0  Add TEMP page + profiles + simulated temps + CAL framework
                      v5.1.1  Fix HT drift (remove += integration bug)
                      v5.1.2  TEMP: profile jump #..#, A/B paging, HT stable (no integrator)

  VERSION   5.1.2
*/

// ---------------------
// Include the libraries
// ---------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>



// ---------------------
// PARAMETERS
// ---------------------

#define DEBUG_MOTOR 0    // set to 1 to enable motor debug prints
#define SIM_TEMPS 0      // 1 = simulated temps, 0 = keep placeholders (-99.9) for now
#define USE_DS18B20 1    // 0 = proxies, 1 = real DS18B20 (when libs + wiring ready)
#define ONE_WIRE_BUS A1  // use analog pin A1 as digital pin for DS18B20 data
#define DS18B20_SCAN 0   // only used when USE_DS18B20==1; prints ROM codes to Serial

#if USE_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

const float HT_GAMMA = 0.20f;  // tune later
const float HT_MIN_C = 20.0f;
const float HT_MAX_C = 60.0f;

// ---------------------
// EEPROM PERSISTENCE
// ---------------------

// ---- Development steps (1..9) ----
struct DevStep {
  uint8_t minutes;
  uint8_t seconds;
  uint8_t defined;  // 0 = not defined, 1 = defined
};

struct Settings {
  uint16_t version;
  uint16_t tMinutes;
  uint16_t tSeconds;
  DevStep devSteps[10];  // use indices 1..9; index 0 unused
  uint8_t profileIndex;  // stored currentProfile (0..NPROFILES-1)

  // Sensor/calibration offsets in 0.1C
  int16_t offHtr_10;     // H-Tronic offset (0.1C)
  int16_t offBath_10;    // Bath sensor offset (0.1C)
  int16_t offTank_10;    // Tank sensor offset (0.1C)
  int16_t offBottle_10;  // Bottle sensor offset (0.1C)
};

struct DevProfile {
  uint8_t id;           // 1..9 (used by "# + digit" selection)
  const char *process;  // "C-41", "RA4", ...
  int setC_tenths;      // target temperature (0.1C)

  uint16_t tank;    // e.g. 1521
  uint16_t vol_ml;  // e.g. 270

  int dT_pour_10;       // pour cooling (0.1C), e.g. 5 => 0.5C
  uint16_t tmin_pre_s;  // minimum preheat time in seconds
  int dT_boost_10;      // optional boost (0.1C), usually 0
};

const DevProfile profiles[] = {
  // Film processing
  { 1, "BW ", 200, 2521, 330, 0, 60, 0 },
  { 2, "C41", 380, 2521, 330, 5, 300, 0 },
  { 3, "E6 ", 380, 2521, 330, 5, 300, 0 },
  { 4, "BW ", 200, 2551, 730, 0, 60, 0 },
  { 5, "C41", 380, 2551, 730, 5, 300, 0 },
  { 6, "E6 ", 380, 2551, 730, 5, 300, 0 },
  { 7, "BW ", 200, 2561, 1000, 0, 60, 0 },
  { 8, "C41", 380, 2561, 1000, 5, 300, 0 },
  { 9, "E6 ", 380, 2561, 1000, 5, 300, 0 },
  // Paper processing
  { 10, "BW ", 200, 2821, 40, 3, 45, 0 },
  { 11, "RA4", 350, 2821, 40, 3, 45, 0 },
  { 12, "BW ", 200, 2831, 100, 3, 60, 0 },
  { 13, "RA4", 350, 2831, 100, 3, 60, 0 },
  { 14, "BW ", 200, 2841, 120, 3, 60, 0 },
  { 15, "RA4", 350, 2841, 120, 3, 60, 0 },
  { 16, "BW ", 200, 2851, 200, 3, 60, 0 },
  { 17, "RA4", 350, 2851, 200, 3, 60, 0 }
};

const uint8_t NPROFILES = sizeof(profiles) / sizeof(profiles[0]);

uint8_t currentProfile = 1;  // default BW

const uint16_t SETTINGS_VERSION = 4;  // restarted
Settings settings;

// ---------------------
// DISPLAY PAGES (v5)
// ---------------------
enum DisplayPage {
  PAGE_DEV = 0,  // classic v4 screen(s)
  PAGE_TEMP = 1  // new stub temperature screen
};

DisplayPage currentPage = PAGE_DEV;


// States of the application
enum AppState : byte {
  ST_DISPLAY_MAINMENU,  // 0
  ST_WAIT,              // 1
  ST_SETTIMING_M,       // 2
  ST_SETTIMING_S,       // 3
  ST_STARTMOTOR,        // 4
  ST_PREHEAT,           // 5
  ST_IDLE,              // 6
  ST_CAL                // 7
};

// Motor control states
enum MotorCommand : byte {
  MOTOR_CONTINUE,   // 0
  MOTOR_START,      // 1
  MOTOR_FORCESTOP,  // 2
  MOTOR_CW,         // 3
  MOTOR_CCW         // 4
};

enum RunMode { MODE_IDLE,
               MODE_DEV,
               MODE_PREHEAT };
RunMode runMode = MODE_IDLE;

// LCD
const byte ROWS = 4;  //  Constants for row and column sizes
const byte COLS = 4;
const int I2C_addr = 0x27;  //  Define I2C Address - found with scanner

// Keypad
char hexaKeys[ROWS][COLS] = {  // Array to represent keys on keypad
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

// Tank
const int deltaMax = 15;   // max adjustement of the angular velocity of the tank [RPM]
const int wTankHigh = 86;  // max angular velocity of the tank = max motor velocity [RPM]
const int wCorFactor = 0;  // correction factor based on measuments
const int nFullRev = 3;    // number of full revolutions of the tank, any direction

// Motor speed
const int wMotorMax = 255;             // max motor speed index (in Arduino). Motor max 84 RPM
unsigned long accelCycleTime = 300UL;  // [0,accelCycleTime] ms.              Acceleration time, 10% total time [s]

// ---------------------
// GLOBAL VARIABLES
// ---------------------

// Potentiometer (rotational speed control)
int potValue;  // analog pot reading

// Temperature sensors
float tempBathC = -99.9f;
float tempTankC = -99.9f;
float tempBottleC = -99.9f;

unsigned long nextTempUpdateMs = 0;
const unsigned long TEMP_UI_REFRESH_MS = 1000UL;  // redraw TEMP once per second
const unsigned long TEMP_UPDATE_INTERVAL_MS = 1000UL;
unsigned long nextTempUiRefreshMs = 0;


#if USE_DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ADD/KEEP these three lines here:
DeviceAddress addrBath = { 0x28, 0x17, 0xB6, 0x54, 0x00, 0x00, 0x00, 0xC5 };  // paste ROM from scan
DeviceAddress addrTank = { 0 };                                               // paste ROM from scan
DeviceAddress addrBottle = { 0 };                                             // paste ROM from scan

// Optional: per-sensor offsets (C) for calibration later
float tempOffsetBathC = 0.0f;
float tempOffsetTankC = 0.0f;
float tempOffsetBottleC = 0.0f;
#endif

// Tank
int delta;     // Change in rotor angular speed [RPM]
int wTankLow;  // min angular speed of the tank [RPM]
int wTank;     // adjusted angular speed of the tank [RPM]
float wMean;   // average angular speed per cycle [RPM]
int wTankInt;  // (75) default Jobo recommendation in [RPM]

// Motor
unsigned long theTiming;       // total time to rotate in [s]
unsigned long theCycleTiming;  // total time to rotate in a CW or CCW cycle [ms]                              e.g. 3000UL
unsigned long breakCycleTime;  // [breakCycleTime,theCycleTiming] ms. Deacceleration time, 10% total time [s] e.g. 2700UL

int wMotor;                       // motor speed index in the range [0,255] = f(theSpeed)
int wMotorMin;                    // min motor speed index (~ 50% of max). Motor min 50 RPM
unsigned long motorRunStartTime;  // the 'time' that the motor started running

// Development control (defaults, will be overwritten by EEPROM if valid)
unsigned long tMinutes = 5;
unsigned long tSeconds = 0;

// States of the application
AppState currentState = ST_DISPLAY_MAINMENU;       // app state
MotorCommand currentStateMotor = MOTOR_FORCESTOP;  // motor initially stopped

// Buzzer state (for 5 s pre-end warning without tone()/Timer2)
bool beepActive = false;
unsigned long beepEndTime = 0;

bool stepStoreMode = false;  // true after '#' until a digit or cancel

// Backlight state & key-combo for darkroom mode
bool backlightOn = true;           // tracks whether backlight is on
bool awaitOffCombo = false;        // true after '*' when waiting for '#'
unsigned long comboStartTime = 0;  // when '*' was pressed (for timeout)

// After run finishes, we wait a few seconds and then redraw the menu
bool showMenuAfterStop = false;
unsigned long stopTimeMs = 0;

// For Preheat phase
unsigned long preheatStartTimeMs = 0;

// PREHEAT PWM (direct pot control)
int pwmPreheat = 150;             // default
const int PWM_PREHEAT_MIN = 100;  // adjust to your motor (must not stall)
const int PWM_PREHEAT_MAX = 255;

bool preheatReady = false;
bool preheatReadyBeepDone = false;
//const unsigned long PREHEAT_READY_MS = 5UL * 60UL * 1000UL;  // 5 min
const unsigned long PREHEAT_READY_MS = 10UL * 1000UL;  // test only
bool devPreEndBeepDone = false;                        // one-shot 5 s warning during DEV

// ---- derived (computed) temperatures (Kodak Hg scale after offsets) ----
float tempMeanC = -99.9f;    // mean(Bh,Tk,Bo) after offsets
float tempDevC = -99.9f;     // Tdev* estimate (not directly observed)
float tempHtrSetC = -99.9f;  // suggested H-Tronic dial setting (C)
bool tempDiagMode = false;   // TEMP-only diagnostic screen toggle

// ---- tuning parameters (Phase 1) ----
const float GAMMA_HTR = 0.05f;  // HT_suggested integrator gain [C/C per 1s update]

// tau [s] = f(volume, rpm)
const float TAU_MIN_S = 10.0f;
const float TAU_MAX_S = 600.0f;
const float TAU_BASE_S = 30.0f;
const float TAU_VOL_S_PER_100ML = 15.0f;  // +15s per 100 mL
const float TAU_RPM_S_AT80 = 30.0f;       // +30s * (80/rpm)
bool devJustStarted = false;

// TEMP profile jump mode (used also to suppress 1 Hz TEMP redraw)
static bool profSelectMode = false;  // now means "#..#" jump mode
static char profBuf[3] = { 0 };      // up to 2 digits + '\0'
static uint8_t profIdx = 0;


// ---------------------
// CALIBRATION (offsets in 0.1C)
// ---------------------
enum CalPage : uint8_t {
  CAL_HTR = 0,
  CAL_BATH = 1,
  CAL_TANK = 2,
  CAL_BOTTLE = 3
};

CalPage calPage = CAL_HTR;

// live offsets (0.1C)
int16_t offHtr_10 = 0;
int16_t offBath_10 = 0;
int16_t offTank_10 = 0;
int16_t offBottle_10 = 0;


// ---------------------
// DEFINE PINS
// ---------------------

// Potentiomenter
const int POT_pin = A0;     // analog potentiometer
const int BUZZER_PIN = A3;  // choose a free digital pin

// Motor (both PWM)
const int RPWM = 10;
const int LPWM = 11;


// LCD
const int en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;

// Keypad
byte rowPins[ROWS] = { 9, 8, 7, 6 };  //connect to the row pinouts of the keypad
byte colPins[COLS] = { 5, 4, 3, 2 };  //connect to the column pinouts of the keypad

// ---------------------
// CREATE OBJECTS
// ---------------------

Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(I2C_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

// helper functions for EEPROM
void factoryResetSettings() {
  settings.version = SETTINGS_VERSION;

  settings.tMinutes = 5;
  settings.tSeconds = 0;

  for (int i = 0; i < 10; ++i) {
    settings.devSteps[i].minutes = 0;
    settings.devSteps[i].seconds = 0;
    settings.devSteps[i].defined = 0;
  }

  settings.offHtr_10 = 0;
  settings.offBath_10 = 0;
  settings.offTank_10 = 0;
  settings.offBottle_10 = 0;

  settings.profileIndex = 1;  // default profile (C41)
  EEPROM.put(0, settings);
}

void loadSettings() {
  EEPROM.get(0, settings);

  // During development: any mismatch => clean slate
  if (settings.version != SETTINGS_VERSION) {
    factoryResetSettings();
    EEPROM.get(0, settings);  // reload what we just wrote (optional, but clean)
  }

  // Load persisted timing
  tMinutes = settings.tMinutes;
  tSeconds = settings.tSeconds;

  // Load persisted temp correction offsets
  offHtr_10 = settings.offHtr_10;
  offBath_10 = settings.offBath_10;
  offTank_10 = settings.offTank_10;
  offBottle_10 = settings.offBottle_10;

  // Load persisted profile safely
  currentProfile = (settings.profileIndex < NPROFILES) ? settings.profileIndex : 1;
}

void saveSettings() {
  settings.version = SETTINGS_VERSION;
  settings.tMinutes = (uint16_t)tMinutes;
  settings.tSeconds = (uint16_t)tSeconds;
  settings.profileIndex = currentProfile;
  settings.offHtr_10 = offHtr_10;
  settings.offBath_10 = offBath_10;
  settings.offTank_10 = offTank_10;
  settings.offBottle_10 = offBottle_10;
  EEPROM.put(0, settings);
}

// -----------------------------------------------------------------
// FUNCTION PROTOTYPES
// -----------------------------------------------------------------

char getKeyWithEcho();
char *getKeypadText();
void displayTitleDEV();
void displayMenu();
void displayTimingMenuM();
void displayTimingMenuS();
void displayCounterMenu();
void renderPageTEMP();
void renderPageDEV();
void renderCurrentPage();
void setTankSpeed();
void runMotor(MotorCommand command);

// v5 / preheat helpers
void startPreheatMotor();
void runPreheatMotor();
void stopPreheatMotor();
void startBeep(unsigned long durationMs);
bool isPreheatReady();
void updateTempProxies();
//void updateTemps1Hz();
int rpmFromPwm(int pwm);

void renderCalPage();
static int16_t *calPtrForPage(CalPage p);
static const char *calNameForPage(CalPage p);

void updateTemps();  // chooses DS18B20 or proxies
#if USE_DS18B20
void updateTempsDS18B20();  // real reads (only compiled when enabled)
static bool isAddrSet(const DeviceAddress a);
#endif

#if USE_DS18B20 && DS18B20_SCAN
void scanDs18b20Bus();
static void printDeviceAddress(const DeviceAddress addr);
#endif

static bool computeHtrKnob10(int16_t *outHT10);
void updateDerivedTemps();
static float computeTauS(uint16_t vol_ml, int rpm);
void updateHtrSuggestion1Hz();
bool updateTemps1Hz();

static bool isValidTempC(float tC);
static int16_t roundC_to_10(float tC);

void setup() {

  // -------------------
  // INITIALIZE
  // -------------------
  Wire.begin();
  Wire.setClock(100000);  // Force 100 kHz

  // Serial port
  Serial.begin(57600);

  // LCD
  lcd.backlight();
  lcd.begin(20, 4);
  Serial.println(F("20x4 display"));
  backlightOn = true;

  // show boot splash for a few seconds (only at boot)
  displayBootScreen();
  delay(2000);  // "few seconds"

  // PINS
  pinMode(POT_pin, INPUT);  // set potentiometer input
  pinMode(RPWM, OUTPUT);    // set motor connections as outputs
  pinMode(LPWM, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);    // Buzzer
  digitalWrite(BUZZER_PIN, LOW);  // OFF at start (active-high: HIGH=ON)

  // Initial motor state set to stop
  analogWrite(RPWM, 0);  // Stop = 0
  analogWrite(LPWM, 0);

  // ---- load last used timing from EEPROM ----
  loadSettings();

  // Initialize Variables
  theTiming = tMinutes * 60UL + tSeconds;  // Default prewarming time in [s]
  delta = 0;
  wTankInt = wTankHigh - deltaMax;
  wTankLow = wTankInt - deltaMax;
  wTank = wTankInt;  // start at nominal RPM
  wMotorMin = wMotorMax * wTankLow / wTankHigh;

  // initialize cycle times here
  theCycleTiming = (nFullRev * 60000UL) / wTank + accelCycleTime;
  breakCycleTime = theCycleTiming - accelCycleTime;
  wMean = (float)(wTank * (theCycleTiming - accelCycleTime) / theCycleTiming);

  randomSeed(analogRead(A2));  // A2 floating is OK as seed
#if USE_DS18B20
  ds18b20.begin();
  ds18b20.setWaitForConversion(false);  // non-blocking conversions
#endif
#if USE_DS18B20 && DS18B20_SCAN
  scanDs18b20Bus();
#endif
}

void loop() {

  // text from keypad
  char *text;

  // -------------------------
  // 1 Hz service block (v5.1)
  // -------------------------

  if (updateTemps1Hz()) {
    updateDerivedTemps();      // uses fresh sensor temps -> updates tempDevC
    updateHtrSuggestion1Hz();  // uses tempDevC
  }


  // Adjust tank speed (with potentiometer)
  setTankSpeed();

  // PREHEAT speed from pot (direct control)
  int pot = analogRead(POT_pin);
  pwmPreheat = map(pot, 0, 1023, PWM_PREHEAT_MIN, PWM_PREHEAT_MAX);
  pwmPreheat = constrain(pwmPreheat, PWM_PREHEAT_MIN, PWM_PREHEAT_MAX);


  // let the motor do what it was doing
  if (runMode == MODE_PREHEAT) {
    runPreheatMotor();
  } else {
    runMotor(MOTOR_CONTINUE);
  }

  serviceDevPreEndBeep();  // beep 5s before end (any display)

  preheatReady = isPreheatReady();

  // PREHEAT: one-shot READY beep
  if (runMode == MODE_PREHEAT && preheatReady && !preheatReadyBeepDone) {
    startBeep(200UL);
    preheatReadyBeepDone = true;
    if (currentPage == PAGE_TEMP) {
      currentState = ST_DISPLAY_MAINMENU;
    }
  }

  // refresh display while running (ONLY in ST_WAIT)
  // so page switches (ST_DISPLAY_MAINMENU) get one clean draw first
  if (currentState == ST_WAIT) {

    // TEMP page: refresh periodically even when IDLE
    if (currentPage == PAGE_TEMP) {
      unsigned long now = millis();

      // DO NOT overwrite the jump prompt while waiting for the closing '#'
      if (!profSelectMode) {
        if (now >= nextTempUiRefreshMs) {
          nextTempUiRefreshMs = now + TEMP_UI_REFRESH_MS;
          renderCurrentPage();
        }
      }
    }

    // DEV page: keep the existing fast refresh only while developing
    else if (runMode == MODE_DEV) {
      renderCurrentPage();  // DEV countdown needs frequent updates
    }
  }

  // After a timed run finishes, show the main menu again after ~3 s
  if (showMenuAfterStop && motorRunStartTime == 0) {
    unsigned long now = millis();
    if (now - stopTimeMs >= 3000UL) {
      if (currentPage == PAGE_DEV) {
        displayMenu();
      } else {
        renderPageTEMP();
      }
      currentState = ST_WAIT;
      showMenuAfterStop = false;
    }
  }

  // --- Beep service ---
  if (beepActive && millis() >= beepEndTime) {
    digitalWrite(BUZZER_PIN, LOW);  // OFF
    beepActive = false;
  }

  //Serial.println(currentState);

  switch (currentState) {
    case ST_DISPLAY_MAINMENU:
      // display main menu
      if (currentPage == PAGE_DEV) {
        displayMenu();
      } else {
        renderPageTEMP();
      }
      // switch to wait state
      currentState = ST_WAIT;
      break;

    case ST_WAIT:
      {
        unsigned long now = millis();

        // -------- persistent combo/profile-mode flags --------
        static bool awaitCalCombo = false;
        static unsigned long calComboStart = 0;
        static bool pendingAProfile = false;

        // ----------------------------------------------------
        // (A) Handle "A alone" timeout WITHOUT requiring a key
        // ----------------------------------------------------
        if (awaitCalCombo && pendingAProfile && (now - calComboStart > 800UL)) {
          awaitCalCombo = false;
          pendingAProfile = false;

          // A-alone => cycle profile
          currentProfile = (currentProfile + 1) % NPROFILES;
          saveSettings();
          nextTempUiRefreshMs = 0;
          currentState = ST_DISPLAY_MAINMENU;
          break;
        }

        // expire pending "* then #" combo if too old
        if (awaitOffCombo && (now - comboStartTime > 1000UL)) {
          awaitOffCombo = false;
        }

        // read raw key (NO echo here)
        char rawKey = keypad.getKey();
        if (rawKey == NO_KEY) {
          break;  // nothing pressed
        }

        // -------------------------------------------------
        // 1) BACKLIGHT CONTROL: "*" and "* then #"
        // -------------------------------------------------
        if (rawKey == '*') {
          if (!backlightOn) {
            lcd.backlight();
            backlightOn = true;
            awaitOffCombo = false;
          } else {
            awaitOffCombo = true;
            comboStartTime = now;
          }
          break;
        }

        if (rawKey == '#' && awaitOffCombo) {
          if (backlightOn) {
            lcd.noBacklight();
            backlightOn = false;
          }
          awaitOffCombo = false;
          break;
        }

        // if '#' arrives but no combo pending, let it fall through
        awaitOffCombo = false;

        // -------------------------------------------------
        // 1b) CAL COMBO (TEMP + IDLE): A then B within 800 ms
        // -------------------------------------------------
        if (currentPage == PAGE_TEMP && runMode == MODE_IDLE) {

          if (rawKey == 'A') {
            awaitCalCombo = true;
            pendingAProfile = true;
            calComboStart = now;
            break;
          }

          // A then B quickly => enter CAL (unchanged)
          if (rawKey == 'B' && awaitCalCombo && (now - calComboStart <= 800UL)) {
            awaitCalCombo = false;
            pendingAProfile = false;

            profSelectMode = false;  // cancel other temp modes
            calPage = CAL_HTR;       // start CAL at HTR page
            currentState = ST_CAL;
            break;
          }

          // NEW: B alone => profile backward
          if (rawKey == 'B') {
            awaitCalCombo = false;
            pendingAProfile = false;

            currentProfile = (currentProfile + NPROFILES - 1) % NPROFILES;
            saveSettings();
            nextTempUiRefreshMs = 0;
            currentState = ST_DISPLAY_MAINMENU;
            break;
          }

          // any other key cancels the combo
          awaitCalCombo = false;
          pendingAProfile = false;
        }

        // -------------------------------------------------
        // 2) STEP PROGRAMMING (DEV): '#' then digit 1..9
        // -------------------------------------------------
        if (currentPage == PAGE_DEV) {
          if (rawKey == '#') {
            stepStoreMode = true;
            lcd.setCursor(0, 3);
            lcd.print(F("Store step: 1...9   "));
            break;
          }

          if (stepStoreMode) {
            if (rawKey >= '1' && rawKey <= '9') {
              int idx = rawKey - '0';
              settings.devSteps[idx].minutes = (uint8_t)tMinutes;
              settings.devSteps[idx].seconds = (uint8_t)tSeconds;
              settings.devSteps[idx].defined = 1;
              stepStoreMode = false;
              saveSettings();
              displayMenu();
              break;
            } else {
              stepStoreMode = false;
              displayMenu();
              // fall through to interpret this key normally
            }
          }
        }

        // -------------------------------------------------
        // 2b) PROFILE SELECT (TEMP): "# <1-2 digits> #"
        //     Example: #15# jumps to profile with id==15
        // -------------------------------------------------
        if (currentPage == PAGE_TEMP) {

          // If we are already in jump mode, consume digits and closing '#'
          if (profSelectMode) {

            if (rawKey >= '0' && rawKey <= '9') {
              if (profIdx < 2) {
                profBuf[profIdx++] = rawKey;
                profBuf[profIdx] = '\0';

                lcd.setCursor(0, 3);
                lcd.print(F("Jump to P:#..#      "));  // 20
                lcd.setCursor(11, 3);                  // after "Jump P:"
                lcd.print(profBuf);
              }
              break;
            }

            if (rawKey == '#') {
              // apply jump
              int want = atoi(profBuf);  // 0..99 (we use 1..N)
              profSelectMode = false;
              profIdx = 0;
              profBuf[0] = '\0';

              bool found = false;
              for (uint8_t i = 0; i < NPROFILES; i++) {
                if (profiles[i].id == (uint8_t)want) {
                  currentProfile = i;  // index in profiles[]
                  found = true;
                  break;
                }
              }

              if (found) {
                saveSettings();
                nextTempUiRefreshMs = 0;
              }
              currentState = ST_DISPLAY_MAINMENU;
              break;
            }

            // any other key: cancel jump mode and continue processing this key normally
            profSelectMode = false;
            profIdx = 0;
            profBuf[0] = '\0';
            // no break here
          }

          // keep your diag toggle (but don't steal '0' while jumping)
          if (rawKey == '0') {
            tempDiagMode = !tempDiagMode;
            currentState = ST_DISPLAY_MAINMENU;
            break;
          }

          // start jump mode
          if (rawKey == '#') {
            profSelectMode = true;
            profIdx = 0;
            profBuf[0] = '\0';

            lcd.setCursor(0, 3);
            lcd.print(F("Jump to P:#..#      "));  // 20

            break;
          }
        }


        // -------------------------------------------------
        // 3) STEP RECALL (DEV): short press 1..9
        // -------------------------------------------------
        if (currentPage == PAGE_DEV) {
          if (rawKey >= '1' && rawKey <= '9') {
            int idx = rawKey - '0';
            if (settings.devSteps[idx].defined) {
              tMinutes = settings.devSteps[idx].minutes;
              tSeconds = settings.devSteps[idx].seconds;
              theTiming = tMinutes * 60UL + tSeconds;
              saveSettings();
              displayMenu();
            }
            break;
          }
        }

        // -------------------------------------------------
        // 4) NORMAL MENU KEYS A/B/C/D
        // -------------------------------------------------
        if (rawKey == 'A') {
          if (currentPage == PAGE_TEMP) {
            // NOTE: In TEMP, A is handled by CAL logic above (A then B, or A-alone timeout).
            // So here we ignore A on TEMP.
            break;
          } else {
            displayTimingMenuM();
            currentState = ST_SETTIMING_M;
            break;
          }

        } else if (rawKey == 'B') {
          if (currentPage == PAGE_DEV) {
            displayTimingMenuS();
            currentState = ST_SETTIMING_S;
          }
          break;

        } else if (rawKey == 'C') {

          if (runMode != MODE_IDLE) {
            currentState = ST_IDLE;
            if (currentPage == PAGE_TEMP) renderPageTEMP();
            else displayMenu();
          } else {
            if (currentPage == PAGE_DEV) {
              displayMenu();
              currentState = ST_STARTMOTOR;
            } else {
              currentState = ST_PREHEAT;
              renderPageTEMP();
            }
          }
          // cancel temp submodes when starting/stopping
          awaitCalCombo = false;
          pendingAProfile = false;
          profSelectMode = false;
          break;

        } else if (rawKey == 'D') {

          // If idle, D toggles page
          currentPage = (currentPage == PAGE_DEV) ? PAGE_TEMP : PAGE_DEV;

          stepStoreMode = false;
          awaitCalCombo = false;
          pendingAProfile = false;
          profSelectMode = false;

          currentState = ST_DISPLAY_MAINMENU;
          break;
        }

        // any other key is ignored in WAIT
        break;
      }
      break;  // IMPORTANT: do not fall-through to ST_SETTIMING_M

    case ST_SETTIMING_M:
      // get the text entered on the keypad
      text = getKeypadText();
      // if text complete
      if (text != NULL) {
        // if user did enter a timming
        if (text[0] != '\0') {
          //theTiming = atoi(text);
          tMinutes = atoi(text);
        }
        // actualize the total timing
        theTiming = tMinutes * 60UL + tSeconds;
        saveSettings();  // keep last time of turn off
        currentState = ST_DISPLAY_MAINMENU;
      }
      break;



    case ST_SETTIMING_S:
      // get the text entered on the keypad
      text = getKeypadText();
      // if text complete
      if (text != NULL) {
        // if user did enter a timming
        if (text[0] != '\0') {
          tSeconds = atoi(text);
        }
        // actualize the total timing
        theTiming = tMinutes * 60UL + tSeconds;
        saveSettings();  // <-- NEW
        currentState = ST_DISPLAY_MAINMENU;
      }
      break;

    case ST_STARTMOTOR:
      // start motor
      devPreEndBeepDone = false;
      runMotor(MOTOR_START);
      runMode = MODE_DEV;
      devJustStarted = true;
      currentState = ST_DISPLAY_MAINMENU;
      break;

    case ST_PREHEAT:
      startPreheatMotor();
      renderPageTEMP();        // show status immediately
      currentState = ST_WAIT;  // return to key handling
      break;

    case ST_CAL:
      {
        static bool calJustEntered = true;
        static unsigned long nextCalUiRefreshMs = 0;

        unsigned long now = millis();

        // refresh CAL view once per second (so you see live temperature changes)
        if (now >= nextCalUiRefreshMs) {
          nextCalUiRefreshMs = now + 1000UL;
          renderCalPage();
        }

        if (calJustEntered) {
          // force an immediate draw on entry
          nextCalUiRefreshMs = 0;
          calJustEntered = false;
        }

        char k = keypad.getKey();
        if (k == NO_KEY) break;

        // -------------------------------------------------
        // CAL UX (your spec):
        //   A => +0.1
        //   B => -0.1
        //   C => Save
        //   D => Return (discard)
        //   # => Next sensor page
        //   * => Previous sensor page
        // -------------------------------------------------
        if (k == '#') {
          calPage = (CalPage)((((uint8_t)calPage) + 1) % 4);
          nextCalUiRefreshMs = 0;
          renderCalPage();
          break;
        }
        if (k == '*') {
          calPage = (CalPage)((((uint8_t)calPage) + 3) % 4);
          nextCalUiRefreshMs = 0;
          renderCalPage();
          break;
        }

        int16_t *p = calPtrForPage(calPage);

        if (k == 'A') {
          if (*p < 999) *p += 1;  // +0.1C
          renderCalPage();
          break;
        }

        if (k == 'B') {
          if (*p > -999) *p -= 1;  // -0.1C
          renderCalPage();
          break;
        }

        if (k == 'C') {
          // Save offsets to EEPROM and exit
          saveSettings();
          calJustEntered = true;
          currentState = ST_DISPLAY_MAINMENU;
          break;
        }

        if (k == 'D') {
          // Exit without saving: restore globals from EEPROM-backed settings
          offHtr_10 = settings.offHtr_10;
          offBath_10 = settings.offBath_10;
          offTank_10 = settings.offTank_10;
          offBottle_10 = settings.offBottle_10;

          calJustEntered = true;
          currentState = ST_DISPLAY_MAINMENU;
          break;
        }

        break;
      }

    case ST_IDLE:
      // stop motor
      if (runMode == MODE_PREHEAT) {
        stopPreheatMotor();
      } else {
        runMotor(MOTOR_FORCESTOP);
        runMode = MODE_IDLE;
      }
      currentState = ST_WAIT;
      break;

  }  // end_of_switch
}  // end loop()

// -------------------------------------------------------------------------
// FUNCTIONS
// -------------------------------------------------------------------------

// -------------------------------------------------
// BOOT SCREEN (shown ONLY once at power-up/reset)
// -------------------------------------------------
#define FW_VERSION_STR "5.1.2"

void displayBootScreen() {
  lcd.clear();
  delay(3);

  lcd.setCursor(0, 0);
  lcd.print(F("  FPP-3 Processor   "));  // 20

  lcd.setCursor(0, 1);
  lcd.print(F("   Luis Samaniego   "));  // 20

  lcd.setCursor(0, 2);
  // lcd.print(F("Version " FW_VERSION_STR ));  // 20 if FW_VERSION_STR="5.1"
  lcd.print(F("                     "));

  lcd.setCursor(0, 3);
  // lcd.print(F("Booting...          "));  // 20
  lcd.print(F("Booting v. " FW_VERSION_STR " ..."));  // 20
}

/// RPM_est = (rpmA_u * PWM + rpmB_u) / 1e6
const long rpmA_u = 355135;    // 0.35513452 * 1,000,000
const long rpmB_u = -1513883;  // -1.51388278 * 1,000,000

int rpmFromPwm(int pwm) {
  long r_u = rpmA_u * (long)pwm + rpmB_u;  // micro-RPM
  if (r_u < 0) r_u = 0;
  return (int)((r_u + 500000L) / 1000000L);  // round to nearest RPM
}

void startBeep(unsigned long durationMs) {
  if (beepActive) return;          // don't overlap
  digitalWrite(BUZZER_PIN, HIGH);  // ON
  beepActive = true;
  beepEndTime = millis() + durationMs;
}

bool isPreheatReady() {

  // --- constants (tenths of a degree, ASCII/punch-card style) ---
  const int16_t BAND_IN_10 = 3;                 // +/-0.3C to ENTER the ready band
  const int16_t BAND_OUT_10 = 5;                // +/-0.5C to LEAVE the ready band (hysteresis)
  const unsigned long HOLD_MS = 30UL * 1000UL;  // must stay in-band this long

  static unsigned long inBandStartMs = 0;
  static bool readyLatched = false;
  static uint8_t lastProf = 255;

  // reset when not preheating
  if (runMode != MODE_PREHEAT || preheatStartTimeMs == 0) {
    inBandStartMs = 0;
    readyLatched = false;
    lastProf = currentProfile;
    return false;
  }

  // if profile changes while in PREHEAT, reset
  if (lastProf != currentProfile) {
    lastProf = currentProfile;
    inBandStartMs = 0;
    readyLatched = false;
  }

  // If bath temp invalid (no sensors), keep your old time fallback
  if (tempBathC < -50.0f || tempBathC > 100.0f) {
    return (millis() - preheatStartTimeMs >= PREHEAT_READY_MS);
  }

  // Compare in tenths to avoid float edge cases
  int16_t bath10 = (int16_t)(tempBathC * 10.0f + (tempBathC >= 0 ? 0.5f : -0.5f));
  int16_t set10 = (int16_t)profiles[currentProfile].setC_tenths;
  int16_t diff10 = bath10 - set10;
  if (diff10 < 0) diff10 = -diff10;

  unsigned long now = millis();

  if (!readyLatched) {
    // enter band
    if (diff10 <= BAND_IN_10) {
      if (inBandStartMs == 0) inBandStartMs = now;
      if (now - inBandStartMs >= HOLD_MS) readyLatched = true;
    } else {
      inBandStartMs = 0;
    }
  } else {
    // leave band (hysteresis)
    if (diff10 > BAND_OUT_10) {
      readyLatched = false;
      inBandStartMs = 0;
    }
  }

  return readyLatched;
}

static void fmtTemp4(float tC, char out5[5]) {
  // produces 4 chars + '\0'  e.g. "38.0" or "--.-"
  if (tC < -50.0f || tC > 100.0f) {
    out5[0] = '-';
    out5[1] = '-';
    out5[2] = '.';
    out5[3] = '-';
    out5[4] = '\0';
  } else {
    char b[6];
    dtostrf(tC, 4, 1, b);  // width 4, 1 decimal
    out5[0] = b[0];
    out5[1] = b[1];
    out5[2] = b[2];
    out5[3] = b[3];
    out5[4] = '\0';
  }
}

#if USE_DS18B20 && DS18B20_SCAN

static void printDeviceAddress(const DeviceAddress addr) {
  // prints: {0x28,0xFF,0x1A,0xB2,0x64,0x16,0x04,0x3C}
  Serial.print('{');
  for (uint8_t i = 0; i < 8; i++) {
    if (i) Serial.print(',');
    Serial.print("0x");
    if (addr[i] < 16) Serial.print('0');
    Serial.print(addr[i], HEX);
  }
  Serial.println('}');
}

void scanDs18b20Bus() {
  Serial.println(F("DS18B20 scan:"));
  uint8_t count = ds18b20.getDeviceCount();
  Serial.print(F("Devices found: "));
  Serial.println(count);

  for (uint8_t i = 0; i < count; i++) {
    DeviceAddress a;
    Serial.print(F("Index "));
    Serial.print(i);
    Serial.print(F(" ROM="));
    if (ds18b20.getAddress(a, i)) {
      printDeviceAddress(a);
    } else {
      Serial.println(F("{ADDRESS_READ_FAIL}"));
    }
  }
  Serial.println(F("Copy/paste ROMs into addrBath/addrTank/addrBottle."));
}
#endif

void updateTempProxies() {

#if USE_DS18B20
  // TODO Phase 2:
  // read DS18B20 sensors and assign:
  // tempBathC, tempTankC, tempBottleC
  // Example placeholders for now:
  // tempBathC   = ...;
  // tempTankC   = ...;
  // tempBottleC = ...;

#else  // USE_DS18B20 == 0  -> use proxies
#if SIM_TEMPS
  // Visible 0.1C steps (tenths)
  static int16_t bh10 = 0;    // Bath
  static int16_t tk10 = -4;   // Tank
  static int16_t bo10 = -12;  // Bottle

  static bool inited = false;
  static uint8_t lastProf = 255;

  // Temp of Bh, Tk, Bo should be taken from the profile where set is given.
  // => in SIM_TEMPS, we always simulate around the selected profile setpoint.
  int16_t target10 = (int16_t)profiles[currentProfile].setC_tenths;

  if (!inited) {
    bh10 = target10;
    tk10 = target10 - 4;
    bo10 = target10 - 12;
    lastProf = currentProfile;
    inited = true;
  }

  // snap to new profile setpoint when profile changes (even if IDLE)
  if (lastProf != currentProfile) {
    bh10 = target10;
    tk10 = target10 - 4;
    bo10 = target10 - 12;
    lastProf = currentProfile;
  }

  // move Bath toward target by 0.1C per second, plus tiny noise
  if (bh10 < target10) bh10 += 1;
  else if (bh10 > target10) bh10 -= 1;

  bh10 += (int16_t)random(-1, 2);  // -0.1..+0.1C

  // Tank/Bottle follow Bath with fixed offsets (+ tiny noise)
  tk10 = bh10 - 4 + (int16_t)random(-1, 2);   // ~0.4C below
  bo10 = bh10 - 12 + (int16_t)random(-1, 2);  // ~1.2C below

  // broad clamps (tenths)
  if (bh10 < 150) bh10 = 150;
  if (bh10 > 450) bh10 = 450;
  if (tk10 < 150) tk10 = 150;
  if (tk10 > 450) tk10 = 450;
  if (bo10 < 150) bo10 = 150;
  if (bo10 > 450) bo10 = 450;

  tempBathC = bh10 / 10.0f;
  tempTankC = tk10 / 10.0f;
  tempBottleC = bo10 / 10.0f;

#else
  tempBathC = -99.9f;
  tempTankC = -99.9f;
  tempBottleC = -99.9f;
#endif
#endif  // USE_DS18B20

  // -------------------------------------------------
  // APPLY OFFSETS ONCE (works for proxies and DS later)
  // Use the LIVE globals (offBath_10, ...) not settings.*
  // -------------------------------------------------
  if (tempBathC > -50.0f && tempBathC < 100.0f) {
    tempBathC += offBath_10 / 10.0f;
  }
  if (tempTankC > -50.0f && tempTankC < 100.0f) {
    tempTankC += offTank_10 / 10.0f;
  }
  if (tempBottleC > -50.0f && tempBottleC < 100.0f) {
    tempBottleC += offBottle_10 / 10.0f;
  }
}


#if USE_DS18B20
void updateTempsDS18B20() {
  // With setWaitForConversion(false), we do a 2-step cycle:
  // call 1: request conversion
  // call 2 (~1s later): read results (conversion is done)
  static bool requested = false;

  if (!requested) {
    ds18b20.requestTemperatures();
    requested = true;
    return;
  }
  requested = false;

  float tBath = isAddrSet(addrBath) ? ds18b20.getTempC(addrBath) : DEVICE_DISCONNECTED_C;
  float tTank = isAddrSet(addrTank) ? ds18b20.getTempC(addrTank) : DEVICE_DISCONNECTED_C;
  float tBot = isAddrSet(addrBottle) ? ds18b20.getTempC(addrBottle) : DEVICE_DISCONNECTED_C;

  // DallasTemperature uses DEVICE_DISCONNECTED_C (-127) when missing
  tempBathC = (tBath <= -100.0f) ? -99.9f : (tBath + tempOffsetBathC);
  tempTankC = (tTank <= -100.0f) ? -99.9f : (tTank + tempOffsetTankC);
  tempBottleC = (tBot <= -100.0f) ? -99.9f : (tBot + tempOffsetBottleC);

  // APPLY OFFSETS (same meaning as proxies; offsets in 0.1C)
  if (tempBathC > -50.0f && tempBathC < 100.0f) tempBathC += offBath_10 / 10.0f;
  if (tempTankC > -50.0f && tempTankC < 100.0f) tempTankC += offTank_10 / 10.0f;
  if (tempBottleC > -50.0f && tempBottleC < 100.0f) tempBottleC += offBottle_10 / 10.0f;
}
#endif

void serviceDevPreEndBeep() {

  // reset when not in DEV
  if (runMode != MODE_DEV || motorRunStartTime == 0) {
    devPreEndBeepDone = false;
    return;
  }

  unsigned long now = millis();
  unsigned long totalMs = theTiming * 1000UL;

  if (totalMs <= 5000UL) return;  // no 5s warning for very short runs
  unsigned long elapsedMs = now - motorRunStartTime;
  if (elapsedMs >= totalMs) return;

  unsigned long remainingMs = totalMs - elapsedMs;

  if (!devPreEndBeepDone && !beepActive && remainingMs <= 5000UL) {
    startBeep(200UL);
    devPreEndBeepDone = true;
  }
}
// -------------------------------------------------
// Tdev update functions
// -------------------------------------------------
static float computeTauS(uint16_t vol_ml, int rpm) {

  float tau = TAU_BASE_S;

  // volume term
  tau += TAU_VOL_S_PER_100ML * ((float)vol_ml / 100.0f);

  // rpm term (avoid divide-by-zero)
  if (rpm < 5) rpm = 5;
  tau += TAU_RPM_S_AT80 * (80.0f / (float)rpm);

  if (tau < TAU_MIN_S) tau = TAU_MIN_S;
  if (tau > TAU_MAX_S) tau = TAU_MAX_S;
  return tau;
}
// -------------------------------------------------
// PDE solver:
// First-order physical “lag” model (closer to reality)
// -------------------------------------------------

void updateDerivedTemps() {

  // ---- mean of valid corrected sensors ----
  float sum = 0.0f;
  uint8_t n = 0;
  if (isValidTempC(tempBathC)) {
    sum += tempBathC;
    n++;
  }
  if (isValidTempC(tempTankC)) {
    sum += tempTankC;
    n++;
  }
  if (isValidTempC(tempBottleC)) {
    sum += tempBottleC;
    n++;
  }
  tempMeanC = (n > 0) ? (sum / (float)n) : -99.9f;

  // ---- choose driver temperature for the lag model ----
  float Tdrive = -99.9f;

  // PREHEAT (empty tank): Tank sensor is closest/immediate contact
  if (runMode == MODE_PREHEAT) {
    if (isValidTempC(tempTankC)) Tdrive = tempTankC;
    else if (isValidTempC(tempMeanC)) Tdrive = tempMeanC;
    else if (isValidTempC(tempBathC)) Tdrive = tempBathC;
  }
  // DEV/IDLE: fall back to mean (if available)
  else {
    if (isValidTempC(tempMeanC)) Tdrive = tempMeanC;
    else if (isValidTempC(tempTankC)) Tdrive = tempTankC;
    else if (isValidTempC(tempBathC)) Tdrive = tempBathC;
    else if (isValidTempC(tempBottleC)) Tdrive = tempBottleC;
  }

  // ---- compute rpm + volume for tau ----
  int rpmNow = 0;
  uint16_t vol_ml = 0;

  if (runMode == MODE_PREHEAT) {
    rpmNow = rpmFromPwm(pwmPreheat);
    vol_ml = 0;  // empty tank assumption in PREHEAT
  } else {
    rpmNow = wTank;  // DEV motor target
    vol_ml = profiles[currentProfile].vol_ml;
  }

  // ---- DEV pour event: one-shot IC for Tdev* (Kodak-ref tenths) ----
  // (Set devJustStarted=true when DEV starts.)
  if (devJustStarted && runMode == MODE_DEV && motorRunStartTime > 0) {

    // Prefer Bo (developer bottle), already Kodak-ref corrected by offsets
    if (isValidTempC(tempBottleC)) {
      int16_t Bo10 = roundC_to_10(tempBottleC);
      int16_t Tdev0_10 = Bo10 - (int16_t)profiles[currentProfile].dT_pour_10;
      tempDevC = Tdev0_10 / 10.0f;  // IC
    } else {
      // fallback if Bo missing
      tempDevC = isValidTempC(Tdrive) ? Tdrive : -99.9f;
    }

    devJustStarted = false;  // one-shot
  }

  // ---- 1st-order lag: Tdev* ----
  if (isValidTempC(Tdrive)) {

    if (!isValidTempC(tempDevC)) {
      tempDevC = Tdrive;  // initialize state
    }

    float tauS = computeTauS(vol_ml, rpmNow);

    // dt is exactly your TEMP_UPDATE_INTERVAL_MS (1 Hz)
    float alpha = (float)TEMP_UPDATE_INTERVAL_MS / (tauS * 1000.0f);
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    tempDevC = tempDevC + alpha * (Tdrive - tempDevC);

  } else {
    tempDevC = -99.9f;
  }
}
static void fmtSigned10_to_5(int16_t v10, char out6[6]) {
  // "+0.3" or "-1.2" width 4 incl sign and decimal
  int16_t a = v10;
  char sign = '+';
  if (a < 0) {
    sign = '-';
    a = -a;
  }
  int16_t i = a / 10;
  int16_t d = a % 10;
  // result 5 incl '\0' => "+12.3" max if you keep small offsets
  snprintf(out6, 6, "%c%2d.%1d", sign, (int)i, (int)d);
}

static int16_t *calPtrForPage(CalPage p) {
  switch (p) {
    case CAL_HTR: return &offHtr_10;
    case CAL_BATH: return &offBath_10;
    case CAL_TANK: return &offTank_10;
    case CAL_BOTTLE: return &offBottle_10;
  }
  return &offBath_10;
}

static const char *calNameForPage(CalPage p) {
  switch (p) {
    case CAL_HTR: return "HTR";
    case CAL_BATH: return "Bh ";
    case CAL_TANK: return "Tk ";
    case CAL_BOTTLE: return "Bo ";
  }
  return "???";
}

void renderCalPage() {
  // lcd.clear();
  // delay(3);

  char row0[21], row1[21], row2[21], row3[21];

  // Row0 (20 chars)
  lcd.setCursor(0, 0);
  if (calPage == CAL_HTR) {
    lcd.print(F("CAL H-Tronic setting"));  // 20 chars
  } else {
    snprintf(row0, sizeof(row0), "CAL %-3s offset       ", calNameForPage(calPage));
    lcd.print(row0);
  }

  // Row1: show profile reference (setpoint) + current reading (measured/proxy)
  // This makes Bh/Tk/Bo explicitly tied to the current profile setpoint (like HTR).
  int t10 = profiles[currentProfile].setC_tenths;
  int ti = t10 / 10;
  int td = abs(t10 % 10);

  char bMeas[5];
  if (calPage == CAL_BATH) {
    fmtTemp4(tempBathC, bMeas);
  } else if (calPage == CAL_TANK) {
    fmtTemp4(tempTankC, bMeas);
  } else if (calPage == CAL_BOTTLE) {
    fmtTemp4(tempBottleC, bMeas);
  } else {
    // CAL_HTR: show computed H-Tronic knob temperature (HT)
    int16_t ht10;
    if (computeHtrKnob10(&ht10)) {
      float htC = ht10 / 10.0f;
      fmtTemp4(htC, bMeas);
    } else {
      bMeas[0] = '-';
      bMeas[1] = '-';
      bMeas[2] = '.';
      bMeas[3] = '-';
      bMeas[4] = '\0';
    }
  }

  // EXACT 20 chars:
  // "Ref:%2d.%1dC " = 11
  // "<TAG>:%4sC"    = 8  (TAG is 3 incl ':', e.g. "Bh:" + 4 + "C")
  // +1 space        = 1  => 20
  const char *tag = "HT ";
  if (calPage == CAL_BATH) tag = "Bh ";
  else if (calPage == CAL_TANK) tag = "Tk ";
  else if (calPage == CAL_BOTTLE) tag = "Bo ";

  // Build "Bh:" etc
  char tag3[4];
  tag3[0] = tag[0];
  tag3[1] = tag[1];
  tag3[2] = ':';
  tag3[3] = '\0';

  snprintf(row1, sizeof(row1), "Ref:%2d.%1dC %s%4sC ",
           ti, td, tag3, bMeas);
  lcd.setCursor(0, 1);
  lcd.print(row1);

  // Row2: offset value
  char offb[6];
  fmtSigned10_to_5(*calPtrForPage(calPage), offb);
  // EXACT 20 chars: "Off:%5sC            " (5+1+5+1+8=20)
  snprintf(row2, sizeof(row2), "Off:%5sC            ", offb);
  lcd.setCursor(0, 2);
  lcd.print(row2);

  // Row3: controls (your spec + page nav)
  // EXACT 20 chars:
  // "A:+ B:- #N C:Sv D:Ex" = 20
  snprintf(row3, sizeof(row3), "A:+ B:- #N C:Sv D:Ex");
  lcd.setCursor(0, 3);
  lcd.print(row3);
}

void updateTemps() {
#if USE_DS18B20
  updateTempsDS18B20();
#else
  updateTempProxies();
#endif
}

#if USE_DS18B20
static bool isAddrSet(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++)
    if (a[i] != 0) return true;
  return false;
}
#endif


bool updateTemps1Hz() {
  unsigned long now = millis();
  if (now < nextTempUpdateMs) return false;
  nextTempUpdateMs = now + TEMP_UPDATE_INTERVAL_MS;

  updateTemps();  // DS18B20 or proxies
  return true;
}

static bool isValidTempC(float tC) {
  return (tC > -50.0f && tC < 100.0f);
}

static int16_t roundC_to_10(float tC) {
  return (int16_t)(tC * 10.0f + (tC >= 0 ? 0.5f : -0.5f));
}

// HT10 = SET10 + (Bh10 - Bo10) + offHtr_10
// where Bh/Bo are already on Kodak Hg scale (offsets applied in updateTemps*)
static bool computeHtrKnob10(int16_t *outHT10) {

  // Need Bath + Bottle to compute gradient
  if (!isValidTempC(tempBathC) || !isValidTempC(tempBottleC)) {
    return false;
  }

  int16_t bath10 = roundC_to_10(tempBathC);
  int16_t bot10 = roundC_to_10(tempBottleC);
  int16_t set10 = (int16_t)profiles[currentProfile].setC_tenths;

  int32_t ht10 = (int32_t)set10 + (int32_t)(bath10 - bot10) + (int32_t)offHtr_10;

  // keep within reasonable display/edit range
  if (ht10 > 999) ht10 = 999;
  if (ht10 < -999) ht10 = -999;

  *outHT10 = (int16_t)ht10;
  return true;
}

// ---------------------
// TEMP PAGE (simple, always redraw 4 rows)
// RULE: every snprintf string below is EXACTLY 20 characters.
// ---------------------
void updateHtrSuggestion1Hz() {

  // ---- Phase 1 / SIM temps: DO NOT integrate HT ----
  // Compute dial suggestion directly from gradient rule:
  // HT10 = SET10 + (Bh10 - Bo10) + offHtr_10  (Bh/Bo already corrected by offsets)
  int16_t ht10;

  if (computeHtrKnob10(&ht10)) {
    tempHtrSetC = ht10 / 10.0f;
  } else {
    tempHtrSetC = profiles[currentProfile].setC_tenths / 10.0f;
  }

  // DEV: compensate expected pour cooling (your existing feature)
  if (runMode == MODE_DEV && motorRunStartTime > 0) {
    tempHtrSetC += profiles[currentProfile].dT_pour_10 / 10.0f;
  }

  // clamp
  if (tempHtrSetC < HT_MIN_C) tempHtrSetC = HT_MIN_C;
  if (tempHtrSetC > HT_MAX_C) tempHtrSetC = HT_MAX_C;
}

void renderPageTEMP() {

  char row0[21], row1[21], row2[21], row3[21];

  // ---------- Row 0 ----------
  int t10 = profiles[currentProfile].setC_tenths;
  int ti = t10 / 10;
  int td = abs(t10 % 10);

  if (!tempDiagMode) {
    snprintf(row0, sizeof(row0), "TEMP %-3s Set:%2d.%1dC  ",
             profiles[currentProfile].process, ti, td);  // 20
  } else {
    snprintf(row0, sizeof(row0), "DIAG %-3s Set:%2d.%1dC  ",
             profiles[currentProfile].process, ti, td);  // 20
  }
  lcd.setCursor(0, 0);
  lcd.print(row0);

  // ---------- Row 1 (status OR diag Bh/Tk) ----------
  if (!tempDiagMode) {
    if (runMode == MODE_PREHEAT && preheatStartTimeMs > 0) {
      unsigned long elapsed_s = (millis() - preheatStartTimeMs) / 1000UL;
      unsigned int mm = (unsigned int)(elapsed_s / 60UL);
      unsigned int ss = (unsigned int)(elapsed_s % 60UL);

      if (preheatReady) snprintf(row1, sizeof(row1), "PREHEAT READY %02u:%02u ", mm, ss);  // 20
      else snprintf(row1, sizeof(row1), "PREHEAT RUN   %02u:%02u ", mm, ss);               // 20

    } else if (runMode == MODE_DEV && motorRunStartTime > 0) {
      snprintf(row1, sizeof(row1), "DEV RUNNING         ");  // 20
    } else {
      //     snprintf(row1, sizeof(row1), "IDLE                ");  // 20
      snprintf(row1, sizeof(row1), "IDLE P%02u            ",
               (unsigned)profiles[currentProfile].id);  // EXACT 20
                                                        //        12345678901234567890
    }
  } else {
    char bBh[5], bTk[5];
    fmtTemp4(tempBathC, bBh);
    fmtTemp4(tempTankC, bTk);
    snprintf(row1, sizeof(row1), "Bh:%4sC Tk:%4sC   ", bBh, bTk);  // 20 (NOTE 3 spaces)
  }
  lcd.setCursor(0, 1);
  lcd.print(row1);

  // ---------- Row 2 ----------
  if (!tempDiagMode) {
    char bDev[5], bHT[5];
    fmtTemp4(tempDevC, bDev);
    fmtTemp4(tempHtrSetC, bHT);
    snprintf(row2, sizeof(row2), "Tdev:%4sC HT:%4sC ", bDev, bHT);  // 20
  } else {
    char bBo[5], bMn[5];
    fmtTemp4(tempBottleC, bBo);
    fmtTemp4(tempMeanC, bMn);
    snprintf(row2, sizeof(row2), "Bo:%4sC Mean:%4sC ", bBo, bMn);  // 20
  }
  lcd.setCursor(0, 2);
  lcd.print(row2);

  // ---------- Row 3 ----------
  if (!tempDiagMode) {
    char bMn[5], bBo[5];
    fmtTemp4(tempMeanC, bMn);
    fmtTemp4(tempBottleC, bBo);
    snprintf(row3, sizeof(row3), "Mean:%4sC Bo:%4sC ", bMn, bBo);  // 20
  } else {
    snprintf(row3, sizeof(row3), "0:Main D:Pg         ");  // 20 (NOTE 9 spaces)
  }
  lcd.setCursor(0, 3);
  lcd.print(row3);
}

// ---------------------
// PAGE_DEV (v5) - wrapper (Phase 1)
// ---------------------
void renderPageDEV() {
  // IMPORTANT: preserve v4 behavior by calling existing display functions
  // exactly as you already do, based on currentState.

  switch (currentState) {
    case ST_DISPLAY_MAINMENU:
      displayMenu();
      break;

    case ST_SETTIMING_M:
      displayTimingMenuM();
      break;

    case ST_SETTIMING_S:
      displayTimingMenuS();
      break;

    case ST_STARTMOTOR:
      if (runMode == MODE_DEV) {
        displayCounterMenu();  // countdown + 5s beep only for DEV
      } else {
        displayMenu();  // if PREHEAT is running and you view DEV page
      }
      break;

    case ST_WAIT:
      if (runMode == MODE_DEV) {
        displayCounterMenu();  // countdown + 5s beep only for DEV
      } else {
        displayMenu();  // if PREHEAT is running and you view DEV page
      }
      break;

    case ST_IDLE:
      if (runMode == MODE_DEV) {
        displayCounterMenu();  // countdown + 5s beep only for DEV
      } else {
        displayMenu();  // if PREHEAT is running and you view DEV page
      }
      break;

    default:
      displayMenu();
      break;
  }
}

// ---------------------
// Render active page (v5)
// ---------------------
void renderCurrentPage() {
  if (currentPage == PAGE_DEV) {
    renderPageDEV();
  } else {
    renderPageTEMP();
  }
}

// ---------------------
// PREHEAT motor functions
// ---------------------
void startPreheatMotor() {
  preheatReady = false;
  preheatReadyBeepDone = false;

  // mark motor running
  preheatStartTimeMs = millis();

  // mark mode
  runMode = MODE_PREHEAT;

  // kick to overcome static friction
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 0);
  delay(250);

  // start single-direction rotation immediately
  // Direction choice: RPWM forward, LPWM = 0
  analogWrite(RPWM, pwmPreheat);
  analogWrite(LPWM, 0);
}

void runPreheatMotor() {
  // keep single-direction rotation (no reversals)
  analogWrite(RPWM, pwmPreheat);
  analogWrite(LPWM, 0);
}

void stopPreheatMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  preheatStartTimeMs = 0;  // reset PREHEAT elapsed timer

  runMode = MODE_IDLE;
}

/*
  display a title : profile
*/
void displayTitleDEV() {
  lcd.clear();
  delay(3);

  char row0[21];
  const DevProfile &p = profiles[currentProfile];

  // Rounded integer Celsius for compact title
  int tC = (p.setC_tenths >= 0) ? (p.setC_tenths + 5) / 10 : (p.setC_tenths - 5) / 10;

  // EXACTLY 20 chars:
  // "P" (1) + id (1) + " " (1) = 3
  // process "%-3s" = 3  -> 6
  // " " = 1              7
  // "%2dC" = 3           10
  // " " = 1              11
  // "%4u" tank = 4       15
  // "/" = 1              16
  // "%3u" ml = 3         19
  // " " = 1              20
  snprintf(row0, sizeof(row0), "P%u %-3s %2dC %4u/%3u ",
           (unsigned)p.id,
           p.process,  // must be 3 chars like "C41"
           tC,
           (unsigned)p.tank,
           (unsigned)p.vol_ml);

  lcd.setCursor(0, 0);
  lcd.print(row0);
}

/*
  display main menu
*/
void displayMenu() {
  // current line where to write on LCD; every time that we write, currentLine will be incremented
  byte currentLine = 0;

  // text buffer for 20 characters and string terminator
  char textbuffer[21];

  // display the title on the first line and update currentLine
  //displayTitle();  // row 0
  displayTitleDEV();
  currentLine = 1;

  // print the current settings on the second line    row 1: time
  lcd.setCursor(0, currentLine++);
  sprintf(textbuffer, "Time = %d min", (int)tMinutes);
  lcd.print(textbuffer);
  lcd.setCursor(14, 1);
  sprintf(textbuffer, "%d s", (int)tSeconds);
  lcd.print(textbuffer);

  // row 2: Rmax
  lcd.setCursor(0, currentLine++);
  sprintf(textbuffer, "Rmax = %d/min", wTank + wCorFactor);
  lcd.print(textbuffer);

  // row 3: compact menu help text (max 20 chars)
  lcd.setCursor(0, 3);
  lcd.print(F("A:m B:s C:Go/St D:Pg"));
}  //            12345678901234567890
/*
  display a 'menu' where the user can enter a timing
*/
void displayTimingMenuM() {
  // display the title on the 1st line
  //displayTitle();
  displayTitleDEV();
  // display additional info on 3rd line
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("# Finish    * Cancel"));
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print(F("Set minutes: "));
}

/*
  display a 'menu' where the user can enter a speed
*/
void displayTimingMenuS() {
  // display the title on the 1st line
  //displayTitle();
  displayTitleDEV();
  // display additional info on 3rd line
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("# Finish    * Cancel"));
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print(F("Set seconds: "));
}

//  display a 'menu' with countdown timer
void displayCounterMenu() {

  static unsigned long nextCounterUpdate = 0;
  static unsigned long lastRemaining = 0;

  unsigned long currTime;
  unsigned long elapsedTime;
  unsigned long remainingTime;
  int t1, t2;
  char textbuffer[21];

  currTime = millis();
  elapsedTime = currTime - motorRunStartTime;

  // avoid underflow if we are at or past the end
  if (elapsedTime >= theTiming * 1000UL) {
    remainingTime = 0;
  } else {
    remainingTime = theTiming * 1000UL - elapsedTime;
  }

  // ---------- 5 s warning tone (one-shot, DOES NOT stop motor) ----------
  // Fire once when we cross from >5 s to <=5 s
  if (theTiming > 5 && remainingTime > 0 && remainingTime <= 5000UL && lastRemaining > 5000UL && !beepActive) {
    startBeep(200UL);
  }
  // remember for next call
  lastRemaining = remainingTime;
  // -------------------------------------------------------------------

  // update LCD once per second
  if (currTime >= nextCounterUpdate) {
    nextCounterUpdate = currTime + 1000UL;

    // get remaining min:sec
    t1 = (remainingTime / 1000UL) / 60;
    t2 = (remainingTime / 1000UL) % 60;

    // ---- left side: Ravg = xxx/min (fixed width) ----
    lcd.setCursor(0, 3);
    sprintf(textbuffer, "Ravg = %2d/min  ", (int)(wMean));
    lcd.print(textbuffer);

    // ---- right side: time at columns 15..19 ----
    lcd.setCursor(15, 3);  // last 5 columns
    sprintf(textbuffer, "%02d:%02d", t1, t2);
    lcd.print(textbuffer);
  }
}

/*
  Reads the analog potentiometer to adapt the tank angular speed
  and updates the display. Small ADC noise is filtered out so the
  displayed RPM does not jitter.
*/
void setTankSpeed() {

  static unsigned long nextSpeedUpdate = 0;
  static unsigned long currTime;
  static int lastWTank = 0;  // last accepted tank speed [RPM]
  static int newWTank;
  char textbuffer[21];
  const unsigned long UPDATE_INTERVAL = 200;  // ms
  const int RPM_THRESHOLD = 2;                // minimal change to accept (anti-jitter)

  potValue = analogRead(POT_pin);
  currTime = millis();

  if (currTime - nextSpeedUpdate >= UPDATE_INTERVAL) {
    //nextTime += 1000;
    nextSpeedUpdate = currTime;

    // estimate change of tank speed based on Pot. reading
    delta = map(potValue, 0, 1023, -deltaMax, deltaMax);

    // candidate new speed
    newWTank = wTankInt + delta;

    // clamp tank speed to safe range
    newWTank = constrain(newWTank, wTankLow, wTankHigh);

    // --------- ANTI-JITTER: only update if change is "large enough" ----------
    if (abs(newWTank - lastWTank) >= RPM_THRESHOLD) {

      wTank = newWTank;
      lastWTank = newWTank;

      theCycleTiming = (nFullRev * 60000UL) / wTank + accelCycleTime;
      breakCycleTime = theCycleTiming - accelCycleTime;

      wMean = (float)(wTank * (theCycleTiming - accelCycleTime) / theCycleTiming);

      // if motor is running, update the motor PWM target
      if (motorRunStartTime != 0) {
        wMotor = map(wTank, wTankLow, wTankHigh, wMotorMin, wMotorMax);
        wMotor = constrain(wMotor, 0, 255);  // optional safety
      }
#if DEBUG_MOTOR
      Serial.print("wMean= ");
      Serial.println(wMean);
      Serial.print("theCycleTiming= ");
      Serial.println(theCycleTiming);
#endif

      if (currentState == ST_WAIT && currentPage == PAGE_DEV) {
        lcd.setCursor(7, 2);
        sprintf(textbuffer, "%d/min", wTank + wCorFactor);
        lcd.print(textbuffer);
      }
    }
  }
}

// ------------------------------------------------------------------------
// MOTOR CONTROL
// ------------------------------------------------------------------------
/*
  starts or stops the motor or runs the motor for a given time
  parameter
    option: option to control motor
    MOTOR_FORCESTOP: force an (emergency) stop; oberrides any other options
    MOTOR_START: if not started, start it; if already started, it's ignored
    for any other option value, motor keeps on doing what it was doing
*/
// void runMotor(byte options) {
void runMotor(MotorCommand command) {

  static unsigned long motorCycleStartTime;  // the 'time' that the motor cycle
  static unsigned long tElapsed;             // Elapsed time in the cycle
  int wMotorVar;                             // changing speed during Acc/DeAcc phases

  unsigned long now = millis();

  // if START (CW) received and motor not started yet
  // NOTE: the "&" operator is a logical as well as a bitwise operator,
  //       whereas the "&&" operator is only a logical operator
  if (command == MOTOR_START && motorRunStartTime == 0) {
    // for debugging
#if DEBUG_MOTOR
    Serial.println(F("START received while motor was not running"));
    Serial.print(F("Timing: "));
    Serial.println(theTiming);
#endif
    // set the start 'time'
    motorRunStartTime = now;
    motorCycleStartTime = now;

#if DEBUG_MOTOR
    // display the start time
    Serial.print(F("Start Time: "));
    Serial.println(motorRunStartTime);
#endif

    // estimate angular velovity of the motor in [0,255]
    wMotor = map(wTank, wTankLow, wTankHigh, wMotorMin, wMotorMax);
    wMotor = constrain(wMotor, 0, 255);  //  constrain

#if DEBUG_MOTOR
    Serial.print(F("Set angular speed motor: "));
    Serial.println(wMotor);
    Serial.print(F("Cycle time in ms at current speed: "));
    Serial.println(theCycleTiming);
#endif

    // start the motor at current speed
    if (wMotor >= 0 && wMotor <= 255) {
      // INITIALIZATION: set motor in CW state
      currentStateMotor = MOTOR_CW;
    }
  }

  // if motor is forced stopped, do nothing
  if (currentStateMotor == MOTOR_FORCESTOP) {
    return;
  }

  // ---- ACCELERATION PHASE ----
  tElapsed = now - motorCycleStartTime;
  if (tElapsed <= accelCycleTime) {
    // increase speed proportional to elapssed time
    wMotorVar = map(tElapsed, 0, accelCycleTime, 0, wMotor);  //map(value, fromLow, fromHigh, toLow, toHigh)
    switch (currentStateMotor) {
      case MOTOR_CW:
        // reverse direction to CCW (Forward)
        digitalWrite(LPWM, LOW);
        analogWrite(RPWM, wMotorVar);
        break;
      case MOTOR_CCW:
        // reverse direction to CW (Reverse)
        digitalWrite(RPWM, LOW);
        analogWrite(LPWM, wMotorVar);
        break;
    }
  }

  // ---- CONSTANT SPEED PHASE ----
  else if (tElapsed > accelCycleTime && tElapsed <= breakCycleTime) {
    switch (currentStateMotor) {
      case MOTOR_CW:
        // reverse direction to CCW (Forward)
        digitalWrite(LPWM, LOW);
        analogWrite(RPWM, wMotor);
        break;
      case MOTOR_CCW:
        // reverse direction to CW (Reverse)
        digitalWrite(RPWM, LOW);
        analogWrite(LPWM, wMotor);
        break;
    }
  }

  // ---- BRAKING PHASE ----
  else if (tElapsed > breakCycleTime && tElapsed < theCycleTiming) {
    // decrease speed proportional to elapssed time
    wMotorVar = map(tElapsed, breakCycleTime, theCycleTiming, wMotor, 0);
    switch (currentStateMotor) {
      case MOTOR_CW:
        // reverse direction to CCW (Forward)
        digitalWrite(LPWM, LOW);
        analogWrite(RPWM, wMotorVar);
        break;
      case MOTOR_CCW:
        // reverse direction to CW (Reverse)
        digitalWrite(RPWM, LOW);
        analogWrite(LPWM, wMotorVar);
        break;
    }
  }

  // ---- CHANGE DIRECTION AFTER ONE CW/CCW CYCLE ----
  // if the rotation cycle has run for the specified cycle duration
  if (currentStateMotor != MOTOR_FORCESTOP && now - motorCycleStartTime >= theCycleTiming) {

    // short stop between directions;
    analogWrite(RPWM, 0);  // Stop = 0
    analogWrite(LPWM, 0);

    // reset the cycle 'time'
    motorCycleStartTime = now;  // reset for the next half-cycle

    // reverse directions
    switch (currentStateMotor) {
      case MOTOR_CW:
        // reverse direction to CCW
        // switch to CCW state
        currentStateMotor = MOTOR_CCW;
        // Serial.print("Rot: ");
        // Serial.println(MOTOR_CCW);
        break;
      case MOTOR_CCW:
        // reverse direction to CW
        // switch to CW state
        currentStateMotor = MOTOR_CW;
        // Serial.print("Rot: ");
        // Serial.println(MOTOR_CW);
        break;
    }
  }

  // ---- STOP WHEN TOTAL PROGRAMMED TIME ELAPSES ----
  if (motorRunStartTime != 0 && (now - motorRunStartTime) >= (theTiming * 1000UL)) {
#if DEBUG_MOTOR
    Serial.println(F("Motor stopped"));
    Serial.print(F("Time: "));
    Serial.println(now);
#endif

    analogWrite(RPWM, 0);  // Stop = 0
    analogWrite(LPWM, 0);

    motorRunStartTime = 0;
    currentStateMotor = MOTOR_FORCESTOP;
    digitalWrite(BUZZER_PIN, LOW);  // OFF
    beepActive = false;

    // remember stop time so we can restore the menu a bit later
    showMenuAfterStop = true;
    stopTimeMs = now;
    runMode = MODE_IDLE;  // <-- IMPORTANT: DEV finished => not running anymore

    return;
  }

  // ---- FORCESTOP (key D) ----
  if (command == MOTOR_FORCESTOP) {
    // for debugging
#if DEBUG_MOTOR
    Serial.println(F("FORCESTOP received"));
#endif
    // stop the motor
    analogWrite(LPWM, 0);  // Stop = 0
    analogWrite(RPWM, 0);

    // reset start 'time'
    motorRunStartTime = 0;
    currentStateMotor = MOTOR_FORCESTOP;

    digitalWrite(BUZZER_PIN, LOW);  // OFF
    beepActive = false;
    // nothing more to do
    return;
  }
}

// -----------------------------------------------------------------
// LCD ROUTINES
// -----------------------------------------------------------------

/*
  get a keystroke from the keypad
  echo the key that was pressed to the LCD
  the echo will be on the current position of the LCD
  returns
    the key that was pressed or NO_KEY
*/
char getKeyWithEcho() {
  // read a key
  char key = keypad.getKey();

  // if no key pressed
  if (key != NO_KEY) {
    // for debugging, output to serial monitor
    Serial.print("Key: ");
    Serial.println(key);
    // display on current position of LCD
    lcd.print(key);
  }
  return key;
}

/*
  get a text from the keypad (max 20 characters)
  '#' finishes the entry of data
  '*' cancels the entry of data
  returns
    NULL if not complete
    empty text if canceled
    entered text (might be empty)
*/
char *getKeypadText() {
  // a buffer for 20 keypresses and one
  static char keypadbuffer[21];  // 20 chars + '\0'
  // index in above buffer
  static uint8_t index = 0;

  // read a key
  char key = getKeyWithEcho();

  // if nothing pressed
  if (key == NO_KEY) {
    // indicate no complete data
    return NULL;  // nothing yet
  }

  // if 'cancel' key
  if (key == '*') {
    // reset index
    index = 0;
    // create empty text
    keypadbuffer[index] = '\0';
    // return text
    return keypadbuffer;  // empty string = canceled
  }
  // if 'enter' key
  if (key == '#') {
    // add a string terminator
    keypadbuffer[index] = '\0';  // terminate
    // reset index for next time
    index = 0;
    // return the text
    Serial.println(keypadbuffer);
    return keypadbuffer;
  }

  // normal digit key: add if room left
  if (index < sizeof(keypadbuffer) - 1) {
    keypadbuffer[index++] = key;
    keypadbuffer[index] = '\0';  // keep it terminated
  }
  // if no room left, extra keys are ignored until # or *

  return NULL;  // entry not complete yet
}
