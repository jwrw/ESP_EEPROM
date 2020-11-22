/*
 ESP_EEPROM.cpp - esp8266 EEPROM emulation
 
 Copyright (c) 2018 James Watson. All rights reserved.
 
 Based on ESP8266 EEPROM library, part of standard
 esp8266 core for Arduino environment by Ivan Grokhotkov.
 
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

/** @class EEPROMClass
 * The ESP does not have a genuine EEPROM memory so this needs to be emulated
 * using a segment of flash memory; this library improves upon the the standard
 * library by avoiding excessive re-flashing of the flash memory.
 *
 * The library maintains a copy of your 'EEPROM' data in normal RAM which you read() and write() to.
 * To ensure this buffered data gets saved when you power-off or reset the system you must
 * call commit() to write the buffer to flash memory that will survive the reset.
 * When you call begin() the library will check the flash memory and read the data there
 * into the buffer so it is available to be read() by your program.
 *
 * ## Calling the library
 * Including this library will create a variable call 'EEPROM' in your program which
 * you use to access the EEPROM functions, such as EEPROM.begin(), EEPROM.read(), etc.
 *
 * ## Why ESP_EEPROM
 * It is not possible to rewrite flash memory without first erasing it back to a known state.
 * With the normal library, each commit() of the data to flash requires an erase of the flash
 * sector. With this revised library the sector in flash memory is used to hold multiple copies
 * of the EEPROM data.  Each time you commit() data, it is written to a new area of flash until the
 * sector is full. Only then does the library erase the sector to allow the next copy to be
 * written.
 *
 * The main drawback of erasing the flash is that it is quite slow (several 10s of ms) and during
 * all operations on flash memory the interrupts have to be halted.
 * Stopping interrupts will disrupt an PWM (analogWrite) outputs which can produce a noticeable
 * disturbance in any lights attached to these.
 * Flash memory can also only accommodate a limited number of re-flashes before it fails.
 * This library helps extend the life of the flash memory by reducing the number of times it
 * needs to be erased.
 *
 * ## Layout
 * This implementation detail is hidden from you by the library but it may be helpful to
 * understand what is happening 'under the hood'.
 *
 * The structure held in the flash segment varies in size depending on the size
 * requested in the call to begin().
 *
 * - 4 bytes - size of a block
 * - bitmap - bit 0 never written - shows state of flash after erase
 *   subsequent bits are set to the opposite for each block containing data
 *   the highest block is the latest version
 * - Data versions follow consecutively - size rounded up to 4 byte boundaries
 *   and a minimum size
 *
 * During the begin() call, the library checks if the requested size matches the size of blocks
 * held in the flash.  If so, the bitmap is used to find the most recently written block and this
 * is copied to the buffer held by the library.
 * When data is written to flash using commit(), then the library updates the bitmap and writes the
 * data to the next available area in the flash segment. If there isn't room for a new copy then the
 * sector is erased, the size and new bitmap are written, followed by the data.
 *
 * ## When to use
 * Most of the time we need a small amount to EEPROM memory to retain settings
 * between re-boots of the system.
 * If your application uses a lot of EEPROM, e.g. more than half a flash segment,
 * then you will get no benefit from using this library.
 * Normally the sector size is 4096 bytes so don't bther with this library if your EEPROM
 * requirement is over ~2000 bytes.
 *
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

extern "C" uint32_t _FS_end;

//------------------------------------------------------------------------------
/**
 * Create an instance of the EEPROM class at using a specified sector of flash memory.
 *
 * This constructor is not normally used, as including the library instantiates the required
 * 'EEPROM' variable.
 *
 * @param sector The flash sector to use to hold the EEPROM data
 */
EEPROMClass::EEPROMClass(uint32_t sector) :
		_sector(sector), _data(0), _size(0), _bitmapSize(0), _bitmap(0), _offset(
				0), _dirty(false) {
}

//------------------------------------------------------------------------------
/**
 * Create an instance of the EEPROM class based on the default EEPROM flash sector.
 *
 * This constructor is not normally used, as including the library instantiates the required
 * 'EEPROM' variable.
 *
 * The EEPROM variable is then used to access the functions of this library.
 *
 * e.g.
 * + EEPROM.begin(50);
 * + EEPROM.put(4, myVariable);
 * + EEPROM.get(4, myOtherVariable);
 * + EEPROM.commit();
 *
 * There should not be any reason for creating a second instance of the ESP_EEPROM class
 * and as the library assumes there is only one instance and only one EEPROM sector
 * results may be 'unpredictable'.
 *
 */
EEPROMClass::EEPROMClass(void) :
		_sector((((uint32_t) & _FS_end - 0x40200000) / SPI_FLASH_SEC_SIZE)), _data(
				0), _size(0), _bitmapSize(0), _bitmap(0), _offset(0), _dirty(
				false) {
}

//------------------------------------------------------------------------------
/**
 * Initialise the EEPROM system, reading from flash if there appears to be suitable
 * data already there.
 *
 * To correctly initialise the library, you need to specify the size of the EEPROM area your
 * program will need.
 *
 * This function checks the flash sector to verify if it contains correct size data.
 * If the size is good and the bitmap is
 * valid then the data buffer in the library is initialised from the flash memory.
 * This is a simplistic check and you may wish to include some check value or version number
 * within your EEPROM data to provide additional confidence that the data is really good.
 *
 * If the size is wrong or the bitmap appear broken then the data buffer is zeroed.
 * Nothing is written to the flash until you call the commit() function, which will erase
 * the sector and write the new data.
 *
 * @param size
 */
void EEPROMClass::begin(size_t size) {
	_dirty = true;
	if (size <= 0 || size > (SPI_FLASH_SEC_SIZE - 8)) {
		// max size is smaller by 4 bytes for size and 4 byte bitmap - to keep 4 byte aligned
		return;
	} else if (size < EEPROM_MIN_SIZE) {
		size = EEPROM_MIN_SIZE;
	}

	size = (size + 3) & ~3; // align to 4 bytes
	_bitmapSize = computeBitmapSize(size);

	// drop any old allocation and re-allocate buffers
	if (_bitmap) {
		delete[] _bitmap;
	}
	_bitmap = new uint8_t[_bitmapSize];
	if (_data) {
		delete[] _data;
	}
	_data = new uint8_t[size];

	noInterrupts();
	spi_flash_read(_sector * SPI_FLASH_SEC_SIZE,
			reinterpret_cast<uint32_t*>(&_size), 4);
	interrupts();

	if (_size != size) {
		// flash structure is all wrong - will need to re-do
		_size = size;
		_offset = 0;    // offset of zero => flash data is garbage

	} else {
		// Size is correct so get bitmap/data from flash
		// First read the bitmap from flash
		noInterrupts();
		spi_flash_read(_sector * SPI_FLASH_SEC_SIZE + 4,
				reinterpret_cast<uint32_t*>(_bitmap), _bitmapSize);
		interrupts();

		// flash should contain a good version of the data - find it using the bitmap
		_offset = offsetFromBitmap();

		if (_offset == 0 || _offset + _size > SPI_FLASH_SEC_SIZE) {
			// something is screwed up
			// flag that _data[] is bad / uninitialised
			_offset = 0;
		} else {
			noInterrupts();
			spi_flash_read(_sector * SPI_FLASH_SEC_SIZE + _offset,
					reinterpret_cast<uint32_t*>(_data), _size);
			interrupts();

			// all good
			_dirty = false;
		}
	}
}

//------------------------------------------------------------------------------
/**
 * Returns the percentage of EEPROM flash memory area that has been used by copies of
 * our EEPROM data.
 *
 * Each commit() will write a new copy to the next free area of the segment of flash
 * memory given over to EEPROM.  This routine allows you to keep track of how much has been used
 * to anticipate when the library will next need to do a flash erase of the EEPROM sector.
 *
 * Since version 2 the return value of -1 is used to indicate that no erase/write has
 * been done to the flash with the current sized EEPROM data. It is important to
 * distinguish this from the case where 1 or 2 copies of a small sized EEPROM data has
 * been written but still might amount to 0% used (when rounded to an integer)
 *
 * @return The percentage used (0-100) or -1 if the flash does not hold any copies of the data.
 */
int EEPROMClass::percentUsed() {
	if (_offset == 0 || _size == 0)
		return -1;
	else {
		int nCopies = (SPI_FLASH_SEC_SIZE - 4 - _bitmapSize) / _size;
		int copyNo = 1 + (_offset - 4 - _bitmapSize) / _size;
		return (100 * copyNo) / nCopies;
	}
}

//------------------------------------------------------------------------------
/**
 * Free up storage used by the library.
 */
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
/**
 * Read a byte of data from an offset in the buffered EEPROM data
 *
 * The data is actually read from the flash during the call to begin()
 * so this call just read from that buffered copy and so is pretty fast.
 *
 * @see get()
 *
 * @param address The offset in the data buffer to read from
 * @return The byte at the specified address.
 */uint8_t EEPROMClass::read(int const address) {
	if (address < 0 || (size_t) address >= _size)
		return 0;
	if (!_data)
		return 0;

	return _data[address];
}

//------------------------------------------------------------------------------
/**
 * Write a byte of data to an address within the the library's EEPROM data buffer
 *
 * This call just updates the internal data buffer.  You must call commit() to actual
 * write your data to flash so that it can be retained between restarts.
 *
 * @param address The offset with the EEPROM to which to write the data
 * @param value The byte of data to write to the address
 */
void EEPROMClass::write(int const address, uint8_t const value) {
	if (address < 0 || (size_t) address >= _size)
		return;
	if (!_data)
		return;

	// Optimise _dirty. Only flagged if data written is different.
	if (_data[address] != value) {
		_data[address] = value;
		_dirty = true;
	}
}

//------------------------------------------------------------------------------
/**
 * Perform an erase of the flash sector before committing the data
 *
 * @return True is all OK; false if there was a problem.
 */
bool EEPROMClass::commitReset() {
	// set an offset that ensures flash will be erased before commit
	uint32_t oldOffset = _offset;   // if commit fails, _offset won't be updated
	_offset = SPI_FLASH_SEC_SIZE;
	_dirty = true;                  // ensure writing takes place
	if (commit()) {
		return (true);
	} else {
		_offset = oldOffset;
		return (false);
	}
}

//------------------------------------------------------------------------------
/**
 * Write the EEPROM data to the flash memory.
 *
 * The flash segment for EEPROM data is erased if necessary before performing the write.
 * The library maintains a record of whether the buffer has been changed and the write
 * to flash is only performed if the flash does not yet have a copy of the data or
 * if the data in the buffer has changed from what is stored in the flash memory.
 *
 * @return True if successful (or if no write was needed); false if the write was unsuccessful.
 */
bool EEPROMClass::commit() {
	// everything has to be in place to even try a commit
	if (!_size || !_data || !_bitmap || _bitmapSize == 0) {
		return false;
	}
	if (!_dirty) {
		return true;
	}

	SpiFlashOpResult flashOk = SPI_FLASH_RESULT_OK;
	uint32_t oldOffset = _offset;   // if write fails, _offset won't be updated

	// If initial version or not enough room for new version, erase and start anew
	if (_offset == 0 || _offset + _size + _size > SPI_FLASH_SEC_SIZE) {

		noInterrupts();
		flashOk = spi_flash_erase_sector(_sector);
		interrupts();
		if (flashOk != SPI_FLASH_RESULT_OK) {
			return false;
		}

		// write size
		noInterrupts();
		flashOk = spi_flash_write(_sector * SPI_FLASH_SEC_SIZE,
				reinterpret_cast<uint32_t*>(&_size), 4);
		interrupts();
		if (flashOk != SPI_FLASH_RESULT_OK) {
			return false;
		}

		// read first 4 bytes of bitmap
		noInterrupts();
		spi_flash_read(_sector * SPI_FLASH_SEC_SIZE + 4,
				reinterpret_cast<uint32_t*>(_bitmap), 4);
		interrupts();

		// init the rest of the _bitmap based on value of first byte
		for (int i = 4; i < _bitmapSize; i++)
			_bitmap[i] = _bitmap[0];

		// all reset ok - point to where the data needs to go
		_offset = 4 + _bitmapSize;
	} else {
		_offset += _size;
	}

	noInterrupts();
	flashOk = spi_flash_write(_sector * SPI_FLASH_SEC_SIZE + _offset,
			reinterpret_cast<uint32_t*>(_data), _size);
	interrupts();

	if (flashOk != SPI_FLASH_RESULT_OK) {
		_offset = oldOffset;
		return false;
	}

	// Data written OK so need to update bitmap
	int bitmapByteUpdated = flagUsedOffset();

	bitmapByteUpdated &= ~3;    // align to 4 byte for write
	noInterrupts();
	flashOk = spi_flash_write(
			_sector * SPI_FLASH_SEC_SIZE + bitmapByteUpdated + 4,
			reinterpret_cast<uint32_t*>(&_bitmap[bitmapByteUpdated]), 4);
	interrupts();
	if (flashOk != SPI_FLASH_RESULT_OK) {
		return false;
	}

	// all good!
	interrupts();
	_dirty = false;
	return true;
}

//------------------------------------------------------------------------------
/**
 * Force an immediate erase of the flash sector - but nothing is written
 *
 * The internal library variables & data are initialised (zeroed) but the commit() function must be called
 * to write structure (size and bitmap etc.) and any new data to the flash.
 *
 * @return True is success; false if the erase operation failed.
 */
bool EEPROMClass::wipe() {
	if (_size == 0 || _bitmapSize == 0)
		return false;      // must have called begin()

	// drop any old allocation and re-allocate buffers
	if (_bitmap) {
		delete[] _bitmap;
	}
	_bitmap = new uint8_t[_bitmapSize];
	if (_data) {
		delete[] _data;
	}
	_data = new uint8_t[_size];

	noInterrupts();
	SpiFlashOpResult flashOk = spi_flash_erase_sector(_sector);
	interrupts();

	// flash is clear - need a commit() to write structure (size and bitmap etc.)
	_dirty = true;
	_offset = 0;
	return (flashOk == SPI_FLASH_RESULT_OK);
}

//------------------------------------------------------------------------------
/**
 * Compute the offset of the current version of data using the bitmap
 *
 * @return The offset of the current version of data
 */
uint16_t EEPROMClass::offsetFromBitmap() {

	if (!_bitmap || _bitmapSize <= 0)
		return 0;

	uint16_t offset = 4 + _bitmapSize;
	boolean flash = (_bitmap[0] & 1); // true => 'after flash' state is 1 (else it must be 0)

	// Check - the very first entry in the bitmap should indicate a valid _data
	if ((flash && ((_bitmap[0] & 2) != 0))
			|| (!flash && ((_bitmap[0] & 2) == 0))) {
		// something's wrong - Bitmap doesn't have bit recording first data version
		return 0;
	}

	for (int bmByte = 0; bmByte < _bitmapSize; bmByte++) {
		for (int bmBit = (bmByte == 0) ? 4 : 1; bmBit < 0x0100; bmBit <<= 1) {
			// looking for bit state that matches the 'after flash' state (i.e. first untouched bit)
			if ((flash && ((_bitmap[bmByte] & bmBit) != 0))
					|| (!flash && ((_bitmap[bmByte] & bmBit) == 0))) {
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
/**
 * Flag within the bitmap the appropriate bit for the current _data version at _offset
 *
 * @return The byte index within _bitmap that has been changed
 */
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
/**
 * Compute size of bitmap needed for the number of copies that can be held
 *
 * @param size The size of the EEPROM required
 * @return Number of bytes required for the bitmap
 */
uint16_t EEPROMClass::computeBitmapSize(size_t size) {

	// With 1 bit in bitmap and 8 bits per byte
	// This is the max number of copies possible
	uint32_t nCopies = ((SPI_FLASH_SEC_SIZE - 4L) * 8L - 1L) / (size * 8L + 1L);

	// applying alignment constraints - this is the bitmap size needed
	uint32_t bitmapSize = (((nCopies + 1L) + 31L) / 8L) & ~3;

	return bitmapSize & 0x7fff;
}

//------------------------------------------------------------------------------
#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
EEPROMClass EEPROM;
#endif
