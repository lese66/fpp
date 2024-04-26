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

  LICENCE   GNU General Public License. Copyright(c) 2024, Luis Samaniego: All rights reserved.

  AUTHOR    Luis Samaniego, 2024/01
  UPDATES   2024/03: Improvement of the timming.
  VERSION   3.0 

*/

// ---------------------
// Include the libraries
// ---------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// ---------------------
// PARAMETERS
// ---------------------

// States of the application
const byte ST_DISPLAY_MAINMENU = 0;
const byte ST_WAIT = 1;
const byte ST_SETTIMING_M = 2;
const byte ST_SETTIMING_S = 3;
const byte ST_STARTMOTOR = 4;
const byte ST_IDLE = 5;

// Motor control states
const int MOTOR_CONTINUE = 0;
const int MOTOR_START = 1;
const int MOTOR_FORCESTOP = 2;
const int MOTOR_CW = 3;
const int MOTOR_CCW = 4;

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

// Development
const unsigned long tMinutesMax = 15;
const unsigned long tSecondsMax = 60;

// Tank
const int deltaMax   = 17;  // max adjustement of the angular velocity of the tank [RPM]
const int wTankHigh  = 84;  // max angular velocity of the tank = max motor velocity [RPM]
const int wCorFactor =  0;  // correction factor based on measuments

// Motor speed
const int wMotorMax = 255;        // max motor speed index (in Arduino). Motor max 84 RPM

unsigned long theCycleTiming = 3000UL;  // total time to rotate in a CW or CCW cycle [ms]
unsigned long accelCycleTime = 300UL;   // [0,accelCycleTime] ms.              Acceleration time, 10% total time [s]
unsigned long breakCycleTime = 2700UL;  // [breakCycleTime,theCycleTiming] ms. Deacceleration time, 10% total time [s]

// ---------------------
// GLOBAL VARIABLES
// ---------------------

// Potentiometer (rotational speed control)
int potValue;      // analog pot reading
int potValueLast;  // analog pot reading

// Keypad
char customKey;  // Character to hold key input

// Tank
int delta;  // Change in rotor angular speed [RPM]
int wTankLow;   // min angular speed of the tank [RPM]
int wTank;      // adjusted angular speed of the tank [RPM]
float wMean;    // average angular speed per cycle [RPM]
int wTankInt;   // (75) default Jobo recommendation in [RPM]

// Motor
int wMotor;                             // motor speed index in the range [0,255] = f(theSpeed)
unsigned long theTiming;                // total time to rotate in [s]

int wMotorMin;                          // min motor speed index (~ 50% of max). Motor min 50 RPM
unsigned long motorRunStartTime;  // the 'time' that the motor started running


// Development control
unsigned long tMinutes = 5;  // Default prewarming time
unsigned long tSeconds = 0;

// States of the application
byte currentState = ST_DISPLAY_MAINMENU;  // the state of the application; start with main menu display
int currentStateMotor;                    // the state of the motor; start with CW rotation once starting

// ---------------------
// DEFINE PINS
// ---------------------

// Potentiomenter
const int POT_pin = A0;  // analog potentiometer

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

void setup() {

  // -------------------
  // INITIALIZE
  // -------------------

  // Serial port
  Serial.begin(57600);

  // LCD
  lcd.backlight();
  lcd.begin(20, 4);
  Serial.println("20x4 display");

  // PINS
  pinMode(POT_pin, INPUT);  // set potentiometer input
  pinMode(RPWM, OUTPUT);    // set motor connections as outputs
  pinMode(LPWM, OUTPUT);

  // Initial motor state set to stop
  analogWrite(RPWM, 0);  // Stop = 0
  analogWrite(LPWM, 0);

  // Initialize Variables
  theTiming = tMinutes * 60UL + tSeconds;  // Default prewarming time in [s]
  delta = 0;
  wTankInt = wTankHigh - deltaMax;
  wTankLow = wTankInt - deltaMax;
  wMotorMin = wMotorMax*wTankLow/wTankHigh;

}

void loop() {
  // single key from keypad
  char key;
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
      // get key
      key = getKeyWithEcho();
      // if timing (min) setting selected
      if (key == 'A') {
        // display 'timing' menu minutes
        displayTimingMenuM();
        // change to state where user can enter timing in min
        currentState = ST_SETTIMING_M;
      }
      // if timing (sec) setting selected
      if (key == 'B') {
        // display 'timing' menu seconds
        displayTimingMenuS();
        // change to state where user can enter timing in sec
        currentState = ST_SETTIMING_S;
      }
      // if START key is selected
      if (key == 'C') {
        // display main menu
        displayMenu();
        // change to state where user can start the motor
        currentState = ST_STARTMOTOR;
      }
      // if END key is selected
      if (key == 'D') {
        // display main menu
        displayMenu();
        // change to state where user can reset parameters
        currentState = ST_IDLE;
      }
      // Note: state does not change if entry is not '1', '2', 'A', 'B, 'C', or 'D'
      break;
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
        currentState = ST_DISPLAY_MAINMENU;
      }
      // actualize the total timing
      theTiming = tMinutes * 60UL + tSeconds;
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
        currentState = ST_DISPLAY_MAINMENU;
      }
      // actualize the total timing
      theTiming = tMinutes * 60UL + tSeconds;
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
  lcd.print("Film-Paper Processor");
}

/*
  display main menu
*/
void displayMenu() {
  // current line where to write on LCD; every time that we write, currentLine will be incremented
  byte currentLine = 0;
  // text buffer for 20 characters and string terminator
  char textbuffer[21];
  char textbuffer2[21];

  // display the title on the first line and update currentLine
  displayTitle();
  currentLine = 1;

  // print the current settings on the second line
  lcd.setCursor(0, currentLine++);
  //sprintf(textbuffer, "Time = %d min %d s", tMinutes, tSeconds);
  sprintf(textbuffer, "Time = %d min", tMinutes);
  lcd.print(textbuffer);
  lcd.setCursor(14, 1);
  sprintf(textbuffer, "%d s", tSeconds);
  lcd.print(textbuffer);

  lcd.setCursor(0, currentLine++);
  sprintf(textbuffer, "Rmax = %d/min", wTank + wCorFactor);
  lcd.print(textbuffer);
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
  lcd.setCursor(0, 4);
  lcd.print("# Finish    * Cancel");
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print("Set minutes: ");
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
  lcd.setCursor(0, 4);
  lcd.print("# Finish    * Cancel");
  // prompt user to set speed (2nd line)
  lcd.setCursor(0, 1);
  lcd.print("Set seconds: ");
}

//  display a 'menu' with countdown timer
void displayCounterMenu() {

  static unsigned long nextTime;
  static unsigned long currTime;
  static unsigned long elapsedTime;
  static unsigned long remainingTime;
  int t1, t2;
  char textbuffer[21];

  currTime = millis();
  elapsedTime = currTime - motorRunStartTime;
  remainingTime = (theTiming * 1000 - elapsedTime);

  // get remaning min:sec

  if (currTime > nextTime) {
    nextTime += 1000;
    t1 = (remainingTime / 1000) / 60;
    t2 = (remainingTime / 1000) % 60;
    // Print integers with leading zeros
    lcd.setCursor(15, 4);
    sprintf(textbuffer, "%02d:%02d", t1, t2);
    lcd.print(textbuffer);

    lcd.setCursor(0, 4);
    sprintf(textbuffer, "Ravg = %d/min", (int)(wMean));
    lcd.print(textbuffer);
  }
}

/*
  Reads the analog potentiometer to adapt the tank angular speed 
*/
void setTankSpeed() {

  static unsigned long nextTime;
  static unsigned long currTime;
  char textbuffer[21];

  potValue = analogRead(POT_pin);
  currTime = millis();

  if (currTime > nextTime) {
    nextTime += 1000;

    // estimate change of tank speed based on Pot. reading
    delta = map(potValue, 0, 1023, -deltaMax, deltaMax);
    wTank = wTankInt + delta;
    //wTankLow = wTankHigh - 2 * deltaMax;
    wMean = (float)(wTank * (theCycleTiming - accelCycleTime) / theCycleTiming);

    // Serial.print("wMean= ");
    // Serial.println(wMean);

    if (currentState == ST_WAIT) {
      lcd.setCursor(7, 2);
      sprintf(textbuffer, "%d/min", wTank + wCorFactor);
      lcd.print(textbuffer);
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
void runMotor(byte options) {

  static unsigned long motorCycleStartTime;  // the 'time' that the motor cycle
  static unsigned long tElapsed;             // Elapsed time in the cycle
  int wMotorVar;                             // changing speed during Acc/DeAcc phases

  // if START (CW) received and motor not started yet
  // NOTE: the "&" operator is a logical as well as a bitwise operator,
  //       whereas the "&&" operator is only a logical operator
  if ((options & MOTOR_START) == MOTOR_START && motorRunStartTime == 0) {
    // for debugging
    Serial.println("START received while motor was not running");
    Serial.print("Timing: ");
    Serial.println(theTiming);

    // set the start 'time'
    motorRunStartTime = millis();
    motorCycleStartTime = motorRunStartTime;
    // display the start time
    Serial.print("Start Time: ");
    Serial.println(motorRunStartTime);

    // estimate angular velovity of the motor in [0,255]
    wMotor = map(wTank, wTankLow, wTankHigh, wMotorMin, wMotorMax);
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

  // Acceleration
  if (currentStateMotor != MOTOR_FORCESTOP) {
    tElapsed = millis() - motorCycleStartTime;
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
  }

  // Breaking
  if (currentStateMotor != MOTOR_FORCESTOP) {
    tElapsed = millis() - motorCycleStartTime;
    if ((tElapsed > breakCycleTime) && (tElapsed < theCycleTiming)) {
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
  }

  // Constant speed
  if (currentStateMotor != MOTOR_FORCESTOP) {
    tElapsed = millis() - motorCycleStartTime;
    if ((tElapsed > accelCycleTime) && (tElapsed <= breakCycleTime)) {
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
  }


  //breakCycleTime

  // CHANGE DIRECTION: if the rotation cycle has run for the specified cycle duration
  if ((millis() - motorCycleStartTime) >= theCycleTiming) {
    if (currentStateMotor != MOTOR_FORCESTOP) {

      // stop the motor;
      analogWrite(RPWM, 0);  // Stop = 0
      analogWrite(LPWM, 0);

      // reset the cycle 'time'
      motorCycleStartTime = millis();

      // reverse directions
      switch (currentStateMotor) {
        case MOTOR_CW:
          // reverse direction to CCW
          // digitalWrite(LPWM, LOW);
          // analogWrite(RPWM, wMotor);
          // switch to CCW state
          currentStateMotor = MOTOR_CCW;
          // Serial.print("Rot: ");
          // Serial.println(MOTOR_CCW);
          break;
        case MOTOR_CCW:
          // reverse direction to CW
          // digitalWrite(RPWM, LOW);
          // analogWrite(LPWM, wMotor);
          // switch to CW state
          currentStateMotor = MOTOR_CW;
          // Serial.print("Rot: ");
          // Serial.println(MOTOR_CW);
          break;
      }
    }
  }

  // STOP: if the motor has run for the specified duration
  if (millis() - motorRunStartTime >= (theTiming * 1000UL)) {
    // inform the user that motor has stopped
    if (motorRunStartTime != 0) {
      Serial.println("Motor stopped");
      Serial.print("Time: ");
      Serial.println(millis());
    }
    // stop the motor;
    analogWrite(RPWM, 0);  // Stop = 0
    analogWrite(LPWM, 0);
    // reset the start 'time'
    motorRunStartTime = 0;
    currentStateMotor = MOTOR_FORCESTOP;
  }
  // if FORCESTOP received
  if ((options & MOTOR_FORCESTOP) == MOTOR_FORCESTOP) {
    // for debugging
    Serial.println("FORCESTOP received");
    // stop the motor
    analogWrite(LPWM, 0);  // Stop = 0
    analogWrite(RPWM, 0);
    // reset start 'time'
    motorRunStartTime = 0;
    currentStateMotor = MOTOR_FORCESTOP;
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
  static char keypadbuffer[21];
  // index in above buffer
  static char index = 0;

  // read a key
  char key = getKeyWithEcho();
  // if nothing pressed
  if (key == NO_KEY) {
    // indicate no complete data
    return NULL;
  }
  // if 'cancel' key
  if (key == '*') {
    // reset index
    index = 0;
    // create empty text
    keypadbuffer[index] = '\0';
    // return text
    return keypadbuffer;
  }
  // if 'enter' key
  if (key == '#') {
    // add a string terminator
    keypadbuffer[index] = '\0';
    // reset index for next time
    index = 0;
    // return the text
    Serial.println(keypadbuffer);
    return keypadbuffer;
  }
  // check for buffer overflow
  if (index >= sizeof(keypadbuffer)) {
    // add a string terminator
    keypadbuffer[sizeof(keypadbuffer) - 1] = '\0';
    // reset index for next time
    index = 0;
    // return the text
    return keypadbuffer;
  }

  // add the character to the buffer
  keypadbuffer[index++] = key;

  // indicate that text is not complete
  return NULL;
}
