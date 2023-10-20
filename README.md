# ESP_EEPROM
An improved EEPROM emulation Arduino library for ESP8266

The ESP8266 family doesn't have genuine EEPROM memory so for Arduino it is normally emulated by using a section of flash memory.

With the standard ESP8266 EEPROM library, the sector needs to be re-flashed every time the changed EEPROM data needs to be saved.  For small amounts of EEPROM data this is very slow and will wear out the flash memory more quickly.  This library writes a new copy of your data when you save (commit) it and keeps track of where in the sector the most recent copy is kept using a bitmap. The flash sector only needs to be erased when there is no more space for copies in the flash sector.  You can keep track of this yourself to do a time-consuming erase when most convenient or the library will do it for you when there is no more space for the data when you commit it.

See
https://github.com/arduino/Arduino/wiki/Library-Manager-FAQ
for details on creating and updating a library to be made available via the Arduino Library Manager.

> Note: requires esp8266 core 3.1+ (latest is 3.1.2 at time of this release)
