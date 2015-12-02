//
// eeprom.h
//

#ifndef _EEPROM_H_
#define _EEPROM_H_
/*
 * Use a page of flash memory to emulate eeprom data.
 *
 * On the STM32F0 ARM processors...
 *  - Flash write size is 16 bits, aka a 'word'
 *  - Flash erase size is one page, 1024 bytes on the STM32F030
 *  - Flash can be reliably erase up to 10,000 times
 *  - An unprogrammed flash word will return 0xFFFF
 *
 * The first 16 bit word of emulated eeprom page contains a unique signature
 * marking this page as emulated eeprom.
 *
 * The second 16 bit word is a counter showing the number of times this page
 * has been erased.
 *
 * A signature may occur in the page more than once. The emulated eeprom read
 * routine must scan the entire page and return the value with a matching
 * signature at the highest address.
 *
 * Eeprom initialize algorithm:
 *  - At startup:
 *    - Verify that the eeprom page address is in flash memory
 *    - If the eeprom signature is correct and the erase count is < 10,000
 *      return OK.
 *    - If the erase count is >= 10,000 move to the next page (or lock-up, or
 *      ignore it)
 *    - If the eeprom signature is 0xFFFF it has not been intialized. Write the
 *      signature at offset 0, write an erase count of 0 at offset 1 and,
 *      optionally, add any intial data.
 *
 * Eeprom read algorithm:
 * 	- Verify signature at page offset 0
 * 	- Verify erase count <= 10,000
 * 	- Scan signatures, record address of most recently seen matching signature
 * 	  until either the end of page is reached or a signature value of 0xFFFF is
 * 	  seen.
 *  - If no matching signature was seen return FAIL
 *  - Return the value associated with the signature
 *
 *  Eeprom write algorithm
 * 	- Verify signature at page offset 0
 * 	- Verify erase count <= 10,000
 * 	- Scan signatures until the value 0xFFFF is found or end of eeprom page is
 * 	  seen.
 * 	- If end of eeprom page call the eeprom erase page procedure and restart the
 * 	  write algorithm from the beginning.
 * 	- Verify there is room to hold an additional signature/value pair.
 * 	- If there is not enough room call the eeprom erase page procedure and
 * 	  restart the write algorithm from the beginning.
 * 	- Write the value's signature followed by the value.
 *
 * Eeprom erase algorithm
 *  - Record signature at page offset 0 and erase count at page offset 1.
 *  - Scan eeprom memory, at each signature record the signature/value pair.
 *    Overwrite any duplicates with the value at the highest memory location.
 *  - Erase the page
 *  - Increment the saved page erase count
 *  - Write eeprom signature at offset 0 and page erase count at offset 1.
 *  - Write all saved signature/value pairs to eeprom.
 */

#endif _EEPROM_H_
