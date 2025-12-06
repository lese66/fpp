# FPP-3 Film/Paper Processor – Arduino Controller

**Author:** *Luis Samaniego*

FPP-3 is an Arduino-based controller for a home-built rotary film/paper processor
using JOBO tanks (TM). It drives a DC motor through an H-bridge, shows timing
and speed on a 20x4 I2C LCD, and is operated via a 4x4 keypad.

The controller supports fully light-tight operation (LCD backlight can be turned
off on demand), bidirectional rotation with smooth acceleration/braking, and
9 programmable development steps stored in EEPROM.

The source and file structure has been written by the author based on
the references provided in the code. Many suggestions and improvements
have been contributed by several members of the Arduino Forum and
other sources. 

---

## Features

- Supports multiple JOBO 2500/2800 series tanks (TM) on a rotary base
- Variable tank speed using a potentiometer, approx. 56–86 RPM
- Smooth acceleration and braking on every direction change
- Automatic direction reversal after a fixed number of revolutions
- Countdown timer with remaining time display and 5 s warning beep
- 9 programmable development steps (1..9), stored in EEPROM
- Light-tight darkroom mode: LCD backlight can be switched off and on via keypad
- Last-used main timing (minutes, seconds) persisted in EEPROM

---

## Hardware overview

- Arduino UNO (or compatible)
- IBT-2 / BTS7960 H-bridge motor driver
- DC motor driving the JOBO tank roller base
- 20x4 I2C LCD (LiquidCrystal_I2C)
- 4x4 matrix keypad
- Potentiometer (tank speed control)
- Buzzer on analog pin A3
- Power supply: 12 V for motor + 5 V for logic (e.g. via DC-DC converter)

Pin assignments are defined at the top of the sketch:

- POT_pin: A0 (tank speed control)
- BUZZER_PIN: A3
- Motor PWM: RPWM = D10, LPWM = D11
- Keypad rows: D9, D8, D7, D6
- Keypad cols: D5, D4, D3, D2
- LCD I2C address: 0x27

---

## Software overview

Main components:

- State machine with the following states (see figure):

  - `ST_DISPLAY_MAINMENU`
  - `ST_WAIT`
  - `ST_SETTIMING_M` (edit minutes)
  - `ST_SETTIMING_S` (edit seconds)
  - `ST_STARTMOTOR`
  - `ST_IDLE`

stateDiagram-v2
    [*] --> ST_DISPLAY_MAINMENU
    ST_DISPLAY_MAINMENU --> ST_WAIT: displayMenu()

    ST_WAIT --> ST_SETTIMING_M: A
    ST_WAIT --> ST_SETTIMING_S: B
    ST_WAIT --> ST_STARTMOTOR: C
    ST_WAIT --> ST_IDLE: D

    ST_SETTIMING_M --> ST_DISPLAY_MAINMENU: # or *
    ST_SETTIMING_S --> ST_DISPLAY_MAINMENU: # or *

    ST_STARTMOTOR --> ST_DISPLAY_MAINMENU: runMotor(MOTOR_START)
    ST_IDLE --> ST_DISPLAY_MAINMENU: runMotor(MOTOR_FORCESTOP)

    ST_WAIT --> ST_WAIT: 1..9 recall\n# then 1..9 store\n* / * then #


- Motor control:
  - Bidirectional rotation (CW / CCW)
  - Acceleration, constant speed, and braking phases
  - Direction reversal after `nFullRev` tank revolutions

- EEPROM persistence:
  - `tMinutes`, `tSeconds` (last-used timing)
  - 9 development steps (1..9) with minutes, seconds, and "defined" flag

The main loop performs:

1. Reads the potentiometer and updates tank RPM (with anti-jitter filter).
2. Calls `runMotor(MOTOR_CONTINUE)` to maintain motor state.
3. Updates the countdown display while the motor is running.
4. Handles keypad input depending on the current state.

---

## Keypad operation

The controller uses a 4x4 keypad with:

- Digits: 0..9
- Function keys: `A`, `B`, `C`, `D`, `*`, `#`

### Key commands by state

#### Main waiting state (`ST_WAIT`)

| State | Key        | Effect                                                         |
|-------|------------|----------------------------------------------------------------|
| WAIT  | `*`        | If backlight is OFF: turn ON. If ON: arm backlight OFF combo. |
| WAIT  | `*` then `#` (within 1 s) | Turn LCD backlight OFF (for darkroom use).      |
| WAIT  | `#`        | Enter "store mode" for development steps.                     |
| WAIT  | store + `1`..`9` | Store current time (min, sec) into step slot 1..9.      |
| WAIT  | `1`..`9`   | Recall stored step 1..9 (if defined) and load its timing.     |
| WAIT  | `A`        | Edit minutes (enter `ST_SETTIMING_M`).                         |
| WAIT  | `B`        | Edit seconds (enter `ST_SETTIMING_S`).                         |
| WAIT  | `C`        | Start motor with current timing.                               |
| WAIT  | `D`        | Stop motor (emergency/normal stop).                            |

Notes:

- Backlight ON: press `*`.
- Backlight OFF: press `*`, then `#` within about 1 second.
- Development steps:
  - Store: `#` then digit 1..9.
  - Recall: press digit 1..9 directly.

#### Timing edit states (`ST_SETTIMING_M` and `ST_SETTIMING_S`)

When editing minutes or seconds, the lower line shows:

- `# Finish    * Cancel`

| State      | Key       | Effect                                          |
|------------|-----------|-------------------------------------------------|
| SET_M/S    | digits    | Type numeric value (minutes or seconds).        |
| SET_M/S    | `#`       | Finish entry, apply value, return to main menu. |
| SET_M/S    | `*`       | Cancel entry, keep previous value, return.      |

---

## How it operates (typical workflow)

1. Power on the controller. The main menu shows:
   - Current time (minutes, seconds)
   - Rmax (current tank RPM)
   - A compact help line: `A:m B:s C:Run D:Stop`

2. Set the desired development time:
   - Press `A` to set minutes, type digits, then `#` to confirm.
   - Press `B` to set seconds, type digits, then `#` to confirm.
   - The total time in seconds is updated and stored in EEPROM.

3. Optionally store this time in a step slot:
   - From the main WAIT state, press `#`.
   - The bottom line shows "Store step: 1...9".
   - Press a digit 1..9 to store the current time in that slot.

4. To recall a stored development step:
   - From WAIT, press the desired digit 1..9.
   - If defined, the time for that step is loaded and shown.

5. To start a run:
   - Press `C` in WAIT.
   - The motor starts, rotates CW and CCW with smooth acceleration
     and braking, and stops automatically when the countdown reaches zero.
   - The LCD shows `Ravg` and a MM:SS countdown on the last line.
   - A short beep sounds about 5 s before the end.

6. To stop early:
   - Press `D`. The motor stops and the timer stops.

7. To use in a fully dark environment:
   - While in WAIT, press `*` then `#` to turn the LCD backlight OFF.
   - Later, press `*` to turn the backlight ON again.

---

## Installation

1. Open the Arduino IDE.
2. Load the FPP-3 sketch (`.ino`) into the IDE.
3. Ensure the following libraries are installed:
   - Wire.h (bundled with Arduino)
   - EEPROM.h (bundled with Arduino)
   - LiquidCrystal_I2C (common third-party library)
   - Keypad (Arduino keypad library)
4. Select the correct board (Arduino UNO) and serial port.
5. Compile and upload the sketch.
6. Connect the USB port of the Arduino UNO to the FPP controller if needed
   for power and/or debugging via the Serial Monitor.

---

## Licensing

- FPP-3 software is licensed under the GNU General Public License,
  version 3 or (at your option) any later version.
  See the file `COPYING` for the full license text.

- FPP-3 hardware design files (mechanical and electrical) are licensed
  under the license documented in `LICENSE-HARDWARE.md`.

- Additional details and third-party attributions are documented in
  `LICENSE.md`.
