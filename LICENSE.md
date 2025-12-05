# LICENSE

[TOC]

FPP-3 software is licensed under the GNU General Public License,
version 3 or (at your option) any later version.
See the file COPYING for the full license text.

## PROJECT

FPP-3 is a C/C++ sketch for the Arduino Uno microcontroller that
automates a rotary film and photographic paper processor for multiple
JOBO® tanks (2500–2800 series). It controls drum rotation speed and
direction, timing, and user interaction via a keypad and 20x4 LCD,
with programmable development steps and EEPROM persistence of settings.

## COPYRIGHT

This file is part of the Luis Samaniego's library for analog
photography. FPP-3 is a C code for the Arduino UNO microcontroler that
automatize a film or paper development processor designed and made by
Luis Samaniego.

## COPYRIGHT HOLDERS

Copyright(c) 2024, Luis Samaniego: All rights reserved.

The code is a property of:

> Luis Samaniego  
> Leipzig

The FPP Arduino Sketch (code) is free software. You can redistribute
it and/or modify it under the terms of the GNU General Public License
as published by the free Software Foundation either version 3 of the
License, or (at your option) any later version.

The program is distributed in the hope it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
for more details.

In particular, this means that you may commercialize it, you have the
right to use it for profit, distribute it, modify it, and create your
own versions under the following condition:

* You must pass on the same freedom with your derivative work.
  That is, any modified software must also be free with GPL license.

The free use of the programm includes commercial, non-commercial,
personal, and research purposes.

The user agrees to obey all laws, rules, and regulations of their
jurisdiction while using the program for any purpose, including
commercial and non-commercial.

## SOFTWARE LICENSE TERMS – GNU GENERAL PUBLIC LICENSE (GPL) v3

The FPP-3 software is licensed under the terms of the GNU General Public
License, version 3 (GPLv3), or (at your option) any later version. This
license grants you the freedom to use, study, share, and modify the
software, under the conditions defined by the Free Software Foundation.

Key points include (informal summary, not a substitute for the license text):

1. **Freedom to run the program for any purpose**  
   You may use the software without restriction, whether for personal,
   educational, research, artistic, or commercial applications.

2. **Freedom to study and modify the program**  
   You have access to the source code and may modify it to suit your
   needs. If you distribute a modified version, you must also provide the
   corresponding source code under GPLv3 (or later).

3. **Freedom to redistribute copies**  
   You may redistribute the original program or your modified versions,
   provided that:
   * The software remains licensed under GPLv3 (or later).
   * You clearly state any changes you have made.
   * You provide (or make available) the corresponding source code.

4. **No warranty**  
   The software is provided “as is,” without any warranty of any kind,
   either expressed or implied. The authors and contributors are not
   liable for any damages arising from the use of this software.

For the complete terms and conditions, please refer to the file
`COPYING` distributed with this program, or visit the official GNU
website:

<https://www.gnu.org/licenses/gpl-3.0.html>

If there is any conflict between this summary and the GPLv3 text, the
GPLv3 text always prevails.

## THIRD-PARTY LIBRARIES AND COMPONENTS

FPP-3 uses and depends on several third-party libraries, which are
licensed under their own terms and provided by the following authors
and organizations:

* **Arduino Core / avr-gcc toolchain / Arduino IDE**  
  Copyright Arduino AG (formerly Arduino LLC)
  and contributors.  
  These provide the base runtime, compiler, and core functions
  (`setup()`, `loop()`, `pinMode()`, `digitalWrite()`, etc.).

* **Wire.h (I2C)**  
  Arduino TWI/I²C library originally written by **Nicholas Zambetti**,
  now maintained as part of the official Arduino core by Arduino AG and
  contributors.

* **EEPROM.h**  
  Part of the official Arduino core libraries, maintained by Arduino AG
  and contributors, used for persistent storage of settings in the
  microcontroller EEPROM.

* **LiquidCrystal_I2C**  
  Arduino LCD I2C library by Frank de Brabander and contributors
  (based on earlier work by Francisco Malpartida), used to drive 20x4
  I2C-compatible character displays.

* **Keypad**  
  Arduino Keypad library by Mark Stanley and Alexander Brevig, used for
  scanning matrix keypads and decoding key events.

You must comply with the licenses of these components when redistributing
FPP-3 or any derivative work. In most cases, these licenses are
GPL-compatible and allow redistribution under GPLv3, but you should
check the specific versions you use in your build environment.

## CONTRIBUTIONS

Contributions to FPP-3 (for example, via patches, pull requests, or
direct collaboration) are assumed to be provided under GPLv3 (or later)
unless explicitly stated otherwise in writing.

By contributing, you confirm that:

* You have the right to contribute the code (you are the author or have
  appropriate permissions), and  
* Your contribution may be distributed and sublicensed under GPLv3 (or
  later) as part of FPP-3.

## TRADEMARKS AND NAMING

“FPP-3” and any associated logos or artwork are used for identification
of this particular project. This license does not grant rights to use
any project name, logo, or trademark in a way that suggests endorsement
by the author without explicit permission.

You are free to fork and rename the project, provided that you respect
the GPLv3 terms and avoid creating confusion about the origin of your
derivative version.

JOBO is a registered trademark of its respective owner. FPP-3 is an
independent project and is not affiliated with or endorsed by JOBO.

## WARRANTY DISCLAIMER (RESTATED)

THE PROGRAM IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT
WITHOUT ANY WARRANTY; WITHOUT EVEN THE IMPLIED WARRANTY OF
MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. SEE THE GNU GENERAL
PUBLIC LICENSE FOR MORE DETAILS.

IN NO EVENT WILL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DAMAGES, INCLUDING BUT NOT LIMITED TO LOSS OF DATA, LOSS OF PROFITS, OR
ANY OTHER INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
INABILITY TO USE THE PROGRAM.

---

For questions about licensing, usage in research, or collaboration,
please contact the author.
