/*
  EEPROM3.cpp - esp8266 EEPROM emulation

  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
   re-write to avoid the significant period with no interrupts
   required for the flash erasure - and avoid unnecessary re-flashing

   // >>> Layout <<<
  // 2 bytes - size of a block
  // bitmap - bit 0 never written - shows state of flash after erase
  // subsequent bits are set to the opposite for each block containing data
  // the highest block is the latest version
  //
  // 4096 bytes per sector
  // size - blocks - bytes in map
  // 8    - 500 - 63
  // 16   - 250 - 32
  // 10   - 400 - 50
  // 100  - 40  - 5
  // 1000 - 4   - 1
  //
  // begin(size)
  //

*/

#include "Arduino.h"
#include "ESP_EEPROM.h"

extern "C" {
#include "c_types.h"
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "spi_flash.h"
}

extern "C" uint32_t _SPIFFS_end;

//------------------------------------------------------------------------------
EEPROMClass::EEPROMClass(uint32_t sector):
  _sector(sector),
  _data(0),
  _size(0),
  _bitmapSize(0),
  _bitmap(0),
  _offset(0),
  _dirty(false)
{
}

//------------------------------------------------------------------------------
EEPROMClass::EEPROMClass(void)  :
  _sector((((uint32_t) & _SPIFFS_end - 0x40200000) / SPI_FLASH_SEC_SIZE)),
  _data(0),
  _size(0),
  _bitmapSize(0),
  _bitmap(0),
  _offset(0),
  _dirty(false)
{
}

//------------------------------------------------------------------------------
void EEPROMClass::begin(size_t size) {
  if (size <= 0 || size > (SPI_FLASH_SEC_SIZE - 8)) {
        // max size is smaller by 4 bytes for size and 4 byte bitmap - to keep 4 byte aligned
    return;
  } else if (size < EEPROM_MIN_SIZE) {
    size = EEPROM_MIN_SIZE;
  }

  size = (size + 3) & ~3; // align to 4 bytes
  _bitmapSize =  computeBitmapSize(size);

  noInterrupts();
  spi_flash_read(_sector * SPI_FLASH_SEC_SIZE, reinterpret_cast<uint32_t*>(&_size), 4);
  interrupts();

  if (_size != size) {
    // flash structure is all wrong - will need to re-do
    _size = size;
    _offset = 0;    // offset of zero => flash data is garbage

    // ensure we drop any old allocation if wrong
    if (_bitmap) {
      delete[] _bitmap;
      _bitmap = new uint8_t[_bitmapSize];
    }
    if (_data) {
      delete[] _data;
      _data = new uint8_t[_size];
    }
    return;
  } else {
    // read the bitmap from flash
    noInterrupts();
    spi_flash_read(_sector * SPI_FLASH_SEC_SIZE + 4, reinterpret_cast<uint32_t*>(_bitmap), _bitmapSize);
    interrupts();

    // flash should contain a good version - find it using the bitmap
    _offset = offsetFromBitmap();

    if (_offset == 0 || _offset + _size > SPI_FLASH_SEC_SIZE) {
      // something is screwed up
      // flag that _data[] is bad / uninitialised
      _offset = 0;
    } else {

      noInterrupts();
      spi_flash_read(_sector * SPI_FLASH_SEC_SIZE + _offset, reinterpret_cast<uint32_t*>(_data), _size);
      interrupts();
    }
  }
  }

  // all good 
  _dirty = false;
}

//------------------------------------------------------------------------------
int EEPROMClass::percentUsed() {
  if(_offset == 0 || _size==0) return 0;
  else {
    int nCopies = (SPI_FLASH_SEC_SIZE - 4 - _bitmapSize) / _size;
    int copyNo = 1 + (_offset - 4 - _bitmapSize) / _size;
    return (100 * copyNo) / nCopies;
  }
}

//------------------------------------------------------------------------------
void EEPROMClass::end() {
  if (!_size)
    return;

  commit();
  if (_data) {
    delete[] _data;
  }
  if (_bitmap) {
    delete[] _bitmap;
  }
  _bitmap = 0;
  _bitmapSize = 0;
  _data = 0;
  _size = 0;
  _dirty = false;
}


//------------------------------------------------------------------------------
uint8_t EEPROMClass::read(int const address) {
  if (address < 0 || (size_t)address >= _size)
    return 0;
  if (!_data)
    return 0;

  return _data[address];
}

//------------------------------------------------------------------------------
void EEPROMClass::write(int const address, uint8_t const value) {
  if (address < 0 || (size_t)address >= _size)
    return;
  if (!_data)
    return;

  // Optimise _dirty. Only flagged if data written is different.
  if (_data[address] != value)
  {
    _data[address] = value;
    _dirty = true;
  }
}
//------------------------------------------------------------------------------
bool EEPROMClass::commitReset() {
  // set an offset that ensures flash will be erased before commit
  uint32_t oldOffset = _offset;   // if commit fails, _offset won't be updated
  _offset = SPI_FLASH_SEC_SIZE;
  _dirty = true;                  // ensure writing takes place
  if ( commit() ) {
    return (true);
  } else {
    _offset = oldOffset;
    return (false);
  }
}


//------------------------------------------------------------------------------
bool EEPROMClass::commit() {
  // everything has to be in place to even try a commit
  if (!_size || !_dirty || !_data || !_bitmap || _bitmapSize == 0) {
    return false;
  }

  SpiFlashOpResult flashOk = SPI_FLASH_RESULT_OK;
  uint32_t oldOffset = _offset;   // if write fails, _offset won't be updated

  noInterrupts();

  // If not enough room for new version, erase and start anew
  if (_offset == 0) {
    // first time writing - but flash has already been erased and initialised in begin()
    _offset = 4 + _bitmapSize;
  } else if (_offset + _size + _size > SPI_FLASH_SEC_SIZE) {

    flashOk = spi_flash_erase_sector(_sector);
    if (flashOk != SPI_FLASH_RESULT_OK) {
      interrupts();
      return false;
    }

    // write size
    flashOk = spi_flash_write(_sector * SPI_FLASH_SEC_SIZE, reinterpret_cast<uint32_t*>(&_size), 4);
    if (flashOk != SPI_FLASH_RESULT_OK) {
      interrupts();
      return false;
    }
    _offset = 4 + _bitmapSize;

    // finally clear down the bitmap - for speed we assume the bitmap bit 0 was good (no re-read)
    uint8_t init = (_bitmap[0] & 1) ? 0xff : 0;
    for (int i = 0; i < _bitmapSize; i++ ) _bitmap[i] = init;

  } else {
    _offset += _size;
  }

  flashOk = spi_flash_write(_sector * SPI_FLASH_SEC_SIZE + _offset, reinterpret_cast<uint32_t*>(_data), _size);

  if (flashOk != SPI_FLASH_RESULT_OK) {
    interrupts();
    _offset = oldOffset;
    return false;
  }

  // Data written OK so need to update bitmap
  int bitmapByteUpdated = flagUsedOffset();

  bitmapByteUpdated &= ~3;    // align to 4 byte for write
  flashOk = spi_flash_write(_sector * SPI_FLASH_SEC_SIZE + bitmapByteUpdated + 4, reinterpret_cast<uint32_t*>(&_bitmap[bitmapByteUpdated]), 4);
  if (flashOk != SPI_FLASH_RESULT_OK) {
    interrupts();
    return false;
  }

  // all good!
  interrupts();
  _dirty = false;
  return true;
}

//------------------------------------------------------------------------------
// Compute the offset of the current version of data using the bitmap
uint16_t EEPROMClass::offsetFromBitmap() {

  if (!_bitmap || _bitmapSize <= 0) return 0;

  uint16_t offset = 4 + _bitmapSize;
  boolean flash = (_bitmap[0] & 1); // true => 'after flash' state is 1 (else it must be 0)

  // Check - the very first entry in the bitmap should indicate a valid _data
  if ((flash && ((_bitmap[0] & 2) != 0)) || (!flash && ((_bitmap[0] & 2) == 0))  ) {
    // something's wrong - Bitmap doesn't have bit recording first data version
    return 0;
  }

  for (int bmByte = 0; bmByte < _bitmapSize; bmByte++) {
    for (int bmBit = (bmByte == 0) ? 4 : 1; bmBit < 0x0100; bmBit <<= 1) {
      // looking for bit state that matches the 'after flash' state (i.e. first untouched bit)
      if ((flash && ((_bitmap[bmByte] & bmBit) != 0)) || (!flash && ((_bitmap[bmByte] & bmBit) == 0))  ) {
        return offset; // offset pointed at last written
      } else {
        offset += _size;
      }
    }
  }

  // dropped off the bottom - return the offset - but it will be useless
  return offset;
}

//------------------------------------------------------------------------------
// flag within the bitmap the appropriate bit for the current _data version at _offset
// return the byte index within _bitmap that has been changed
int EEPROMClass::flagUsedOffset() {
  int bitNo = 1 + (_offset - 4 - _bitmapSize) / _size;
  int byteNo = bitNo >> 3;

  uint8_t bitMask = 1 << (bitNo & 0x7);
  if (_bitmap[0] & 1) {
    // need to clear the bitmap bit
    _bitmap[byteNo] &= ~bitMask;
  } else {
    // need to set the bitmap bit
    _bitmap[byteNo] |= bitMask;
  }

  return byteNo;
}

//------------------------------------------------------------------------------
// Computing the exact correct size in bytes is a bit messy
// we just go for something safe & simple but not v elegant
uint16_t EEPROMClass::computeBitmapSize(size_t size) {
  // TODO This may be the correct formula??? - will test properly some time...
  //  ( 3 + (SPI_FLASH_SEC_SIZE + 9 * size) / (8 * size)) & ~3 ;  // TODO
  uint16_t bitmapSize = 0;

  int nVersions;
  do {
    // bitmap size must be 4 byte aligned
    bitmapSize += 4;
    nVersions = (SPI_FLASH_SEC_SIZE - 4 - bitmapSize) / size;
  } while ( (nVersions + 1) > (bitmapSize * 8) );
  // bitmap needs one bit per version plus 1 for the unchanged 'reference' bit at the start

  return bitmapSize;
}

//------------------------------------------------------------------------------
#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
EEPROMClass EEPROM;
#endif
