// Example use of ESP_EEPROM library for ESP8266
//
// Normally writing to the 'emulated' EEPROM on ESP8266 requires an erase of the flash page used to hold
// the EEPROM data followed by a re-write of the changed data
// The erasure takes a significant amount of time (10s of ms) and during this time
// interrupts are/must be blocked.
// In some cases this interferes with the sketch operation (e.g. produces a noticeable
// blackout/flash for any PWM controlled lights as ESP8266 PWM relies on interrupts)
//
// The ESP_EEPROM library writes each new version of the EEPROM data to a new area until flash
// is full and so avoids wiping flash until necessary
//
// It's best for use when there are only a few things to save in EEPROM
// (i.e. total the size of the saved info is much smaller than the available flash size)
//

#include <ESP_EEPROM.h>

const int BLUE_LED_PIN = 2;

// The neatest way to access variables stored in EEPROM is using a structure
struct {
  int   anInteger;
  float aFloating;
  int   anotherInteger;
  byte  someBytes[12];
  boolean state;
} eepromVar1, eepromVar2;

void setup() {
  Serial.begin(74880);

  // Set up the initial (default) values for what is to be stored in EEPROM
  eepromVar1.anInteger = 99;
  eepromVar1.aFloating = 99.99;
  eepromVar1.anotherInteger = 42;
  eepromVar1.state = true;

  // All the library functions are accessed via the EEPROM object created when
  // you include the library header ESP_EEPROM.h

  // The library needs to know what size you need for your EEPROM variables
  // Using a structure makes this easy.

  // The begin() call will find data previously saved in EEPROM if the same size was
  // previously used.  If the size has changed from what is found, then this call acts
  // the same as beginReset() and no data is initialised
  EEPROM.begin(sizeof(eepromVar1));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromVar1);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "Commit OK" : "Commit failed");

  EEPROM.get(0, eepromVar2);

}


void loop() {


}
