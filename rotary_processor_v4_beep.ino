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
  HISTORY   2024/01.  Original code (3.0)
            2025/12.  Bugs and improvements
  VERSION   4.0
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
};

const uint16_t SETTINGS_VERSION = 2;  // bumped from 1 to 2
Settings settings;

// States of the application
enum AppState : byte {
  ST_DISPLAY_MAINMENU,  // 0
  ST_WAIT,              // 1
  ST_SETTIMING_M,       // 2
  ST_SETTIMING_S,       // 3
  ST_STARTMOTOR,        // 4
  ST_IDLE               // 5
};

// Motor control states
enum MotorCommand : byte {
  MOTOR_CONTINUE,   // 0
  MOTOR_START,      // 1
  MOTOR_FORCESTOP,  // 2
  MOTOR_CW,         // 3
  MOTOR_CCW         // 4
};

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
void loadSettings() {
  EEPROM.get(0, settings);

  if (settings.version != SETTINGS_VERSION) {
    // First time / incompatible old data -> set defaults
    settings.version = SETTINGS_VERSION;
    settings.tMinutes = 5;
    settings.tSeconds = 0;

    // initialize all steps as "undefined"
    for (int i = 0; i < 10; ++i) {
      settings.devSteps[i].minutes = 0;
      settings.devSteps[i].seconds = 0;
      settings.devSteps[i].defined = 0;
    }

    EEPROM.put(0, settings);
  }

  tMinutes = settings.tMinutes;
  tSeconds = settings.tSeconds;
}

void saveSettings() {
  settings.version = SETTINGS_VERSION;
  settings.tMinutes = (uint16_t)tMinutes;
  settings.tSeconds = (uint16_t)tSeconds;
  EEPROM.put(0, settings);
}

// -----------------------------------------------------------------
// FUNCTION PROTOTYPES
// -----------------------------------------------------------------

char getKeyWithEcho();
char *getKeypadText();
void displayTitle();
void displayMenu();
void displayTimingMenuM();
void displayTimingMenuS();
void displayCounterMenu();
void setTankSpeed();
void runMotor(MotorCommand command);

void setup() {

  // -------------------
  // INITIALIZE
  // -------------------

  // Serial port
  Serial.begin(57600);

  // LCD
  lcd.backlight();
  lcd.begin(20, 4);
  Serial.println(F("20x4 display"));
  backlightOn = true;

  // TEST: blink backlight once at startup
  // lcd.backlight();
  // delay(500);
  // lcd.noBacklight();
  // delay(5000);
  // lcd.backlight();


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
  wTank = wTankInt;                        // start at nominal RPM
  wMotorMin = wMotorMax * wTankLow / wTankHigh;

  // initialize cycle times here
  theCycleTiming = (nFullRev * 60000UL) / wTank + accelCycleTime;
  breakCycleTime = theCycleTiming - accelCycleTime;
  wMean = (float)(wTank * (theCycleTiming - accelCycleTime) / theCycleTiming);
}

void loop() {

  // text from keypad
  char *text;

  // Adjust tank speed (with potentiometer)
  setTankSpeed();

  // let the motor do what it was doing
  runMotor(MOTOR_CONTINUE);

  // countdown timer
  if (motorRunStartTime > 0) {
    displayCounterMenu();
  }

  //Serial.println(currentState);

  switch (currentState) {
    case ST_DISPLAY_MAINMENU:
      // display main menu
      displayMenu();
      // switch to wait state
      currentState = ST_WAIT;
      break;

    case ST_WAIT:
      {
        unsigned long now = millis();

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
            // in darkroom, single '*' -> turn light ON
            lcd.backlight();
            backlightOn = true;
            awaitOffCombo = false;
          } else {
            // light is ON: start waiting for '#' to switch OFF
            awaitOffCombo = true;
            comboStartTime = now;
          }
          // do NOT use '*' for anything else in WAIT
          break;
        }

        if (rawKey == '#' && awaitOffCombo) {
          // '*' was pressed before and still within 1s -> turn OFF
          if (backlightOn) {
            lcd.noBacklight();
            backlightOn = false;
          }
          awaitOffCombo = false;
          // do NOT use this '#' for anything else
          break;
        }

        // if '#' arrives but no combo pending, we let it fall through
        // (so it can be used for "store step" below)
        awaitOffCombo = false;

        // -------------------------------------------------
        // 2) STEP PROGRAMMING: press '#' then digit 1..9
        // -------------------------------------------------
        if (rawKey == '#') {
          // enter "store mode"
          stepStoreMode = true;
          lcd.setCursor(0, 3);
          lcd.print("Store step: 1...9   ");
          break;  // wait for next key
        }

        if (stepStoreMode) {
          if (rawKey >= '1' && rawKey <= '9') {
            int idx = rawKey - '0';  // '1'->1 ... '9'->9
            settings.devSteps[idx].minutes = (uint8_t)tMinutes;
            settings.devSteps[idx].seconds = (uint8_t)tSeconds;
            settings.devSteps[idx].defined = 1;
            stepStoreMode = false;
            // persist to EEPROM
            saveSettings();

            // re-display menu to restore normal bottom line
            displayMenu();
            break;
          } else {
            // any other key cancels store mode and falls through
            stepStoreMode = false;
            displayMenu();
            // do NOT "break" here; we still want to interpret this key
            // as normal (e.g. A/B/C/D, or recall digit)
          }
        }

        // -------------------------------------------------
        // 3) STEP RECALL: short press 1..9 in WAIT
        // -------------------------------------------------
        if (rawKey >= '1' && rawKey <= '9') {
          int idx = rawKey - '0';
          if (settings.devSteps[idx].defined) {
            tMinutes = settings.devSteps[idx].minutes;
            tSeconds = settings.devSteps[idx].seconds;
            theTiming = tMinutes * 60UL + tSeconds;
            saveSettings();
            displayMenu();  // show updated time
          }
          // stay in ST_WAIT
          break;
        }

        // -------------------------------------------------
        // 4) NORMAL MENU KEYS A/B/C/D (no echo in WAIT)
        // -------------------------------------------------
        if (rawKey == 'A') {
          displayTimingMenuM();
          currentState = ST_SETTIMING_M;
        } else if (rawKey == 'B') {
          displayTimingMenuS();
          currentState = ST_SETTIMING_S;
        } else if (rawKey == 'C') {
          displayMenu();
          currentState = ST_STARTMOTOR;
        } else if (rawKey == 'D') {
          displayMenu();
          currentState = ST_IDLE;
        }
        // any other key is ignored in WAIT

        break;
      }

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
      runMotor(MOTOR_START);
      currentState = ST_DISPLAY_MAINMENU;
      break;
    case ST_IDLE:
      // stop motor
      runMotor(MOTOR_FORCESTOP);
      currentState = ST_DISPLAY_MAINMENU;
      break;
  }  // end_of_switch
}

// -------------------------------------------------------------------------
// FUNCTIONS
// -------------------------------------------------------------------------

/*
  display a title on the first line of the display; it always clears the LCD
*/
void displayTitle() {
  // clear the lcd
  lcd.clear();
  // print the project title on the first line
  lcd.setCursor(0, 0);
  lcd.print(F("Film-Paper Processor"));
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
  displayTitle();  // row 0
  currentLine = 1;

  // print the current settings on the second line    row 1: time
  lcd.setCursor(0, currentLine++);
  sprintf(textbuffer, "Time = %d min", tMinutes);
  lcd.print(textbuffer);
  lcd.setCursor(14, 1);
  sprintf(textbuffer, "%d s", tSeconds);
  lcd.print(textbuffer);

  // row 2: Rmax
  lcd.setCursor(0, currentLine++);
  sprintf(textbuffer, "Rmax = %d/min", wTank + wCorFactor);
  lcd.print(textbuffer);

  // row 3: compact menu help text (max 20 chars)
  lcd.setCursor(0, 3);
  lcd.print(F("A:m B:s C:Run D:Stop"));
}
/*
  display a 'menu' where the user can enter a timing
*/
void displayTimingMenuM() {
  // display the title on the 1st line
  displayTitle();
  // display additional info on 3rd line
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("# Finish    * Cancel");
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print(F("Set minutes: "));
}

/*
  display a 'menu' where the user can enter a speed
*/
void displayTimingMenuS() {
  // display the title on the 1st line
  displayTitle();
  // display additional info on 3rd line
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("# Finish    * Cancel");
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print(F("Set seconds: "));
}

//  display a 'menu' with countdown timer
void displayCounterMenu() {

  static unsigned long nextCounterUpdate = 0;
  static unsigned long currTime;
  static unsigned long elapsedTime;
  static unsigned long remainingTime;
  static unsigned long lastRemaining = 0;
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

    digitalWrite(BUZZER_PIN, HIGH);  // active buzzer ON
    beepActive = true;
    beepEndTime = millis() + 200UL;  // ~200 ms beep (quieter in time)
  }

  // Switch buzzer off when its time window is over
  if (beepActive && millis() >= beepEndTime) {
    digitalWrite(BUZZER_PIN, LOW);  // active buzzer OFF
    beepActive = false;
  }

  // remember for next call
  lastRemaining = remainingTime;
  // -------------------------------------------------------------------

  // update LCD once per second

  // get remaning min:sec
  if (currTime > nextCounterUpdate) {
    nextCounterUpdate += 1000;
    t1 = (remainingTime / 1000) / 60;
    t2 = (remainingTime / 1000) % 60;

    // ---- clear whole line 3 ----
    lcd.setCursor(0, 3);
    lcd.print("                    ");  // 20 spaces

    // ---- left side: Ravg ----
    lcd.setCursor(0, 3);
    sprintf(textbuffer, "Ravg = %d/min", (int)(wMean));
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

      // Serial.print("wMean= ");
      // Serial.println(wMean);
      // Serial.print("theCycleTiming= ");
      // Serial.println(theCycleTiming);

      if (currentState == ST_WAIT) {
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
    Serial.println(F("START received while motor was not running"));
    Serial.print("Timing: ");
    Serial.println(theTiming);

    // set the start 'time'
    motorRunStartTime = now;
    motorCycleStartTime = now;

    // display the start time
    Serial.print("Start Time: ");
    Serial.println(motorRunStartTime);

    // estimate angular velovity of the motor in [0,255]
    wMotor = map(wTank, wTankLow, wTankHigh, wMotorMin, wMotorMax);
    wMotor = constrain(wMotor, 0, 255);  //  constrain

    Serial.print("Set angular speed motor: ");
    Serial.println(wMotor);
    Serial.print("Cycle time in ms at current speed: ");
    Serial.println(theCycleTiming);

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

    Serial.println(F("Motor stopped"));
    Serial.print("Time: ");
    Serial.println(now);

    analogWrite(RPWM, 0);  // Stop = 0
    analogWrite(LPWM, 0);

    motorRunStartTime = 0;
    currentStateMotor = MOTOR_FORCESTOP;
    digitalWrite(BUZZER_PIN, LOW);  // OFF
    beepActive = false;
    return;
  }

  // ---- FORCESTOP (key D) ----
  if (command == MOTOR_FORCESTOP) {
    // for debugging
    Serial.println(F("FORCESTOP received"));

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
