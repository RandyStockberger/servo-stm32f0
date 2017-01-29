<<<<<<< Updated upstream
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
 * Flash reset algorithm
 *  - Save signature at page offset 0 and erase count at page offset 1.
 *  - Scan eeprom memory, at each signature save the signature/value pair.
 *    Overwrite any duplicates with the value at the highest memory location.
 *  - Erase the page
 *  - Increment the saved page erase count
 *  - Write the saved eeprom signature at offset 0 and the page erase count at
 *    offset 1.
 *  - Write all saved signature/value pairs to eeprom.
 */

// =============================================================================
// flashErase -- Erase a page in flash
static bool flashErase( uint16_t * page );

// =============================================================================
// flashWriteWord -- Program one word in flash
static bool flashWriteWord( uint16_t * pWord, uint16_t value );

// =============================================================================
// eepromInitialize -- Insure that the flash emulating the eeprom is properly configured
//  return	true on success
// 			false on failure
bool eepromInitialize( void );

// =============================================================================
// eepromRead -- Read one word from eeprom
//  return	true on success
// 			false on failure
// When returning true the uint16_t pointed to by pvalue contains the value read
// When returning false the uint16_t pointed to by pvalue is undefined
bool eepromRead( uint16_t signature, uint16_t * pvalue );

// =============================================================================
// eepromWrite -- Write a value to the 'address' specified by signature
//  return	true on success
// 			false on failure
bool eepromWrite( uint16_t signature, uint16_t value );

// =============================================================================
// flashReset -- Erase flash and re-record values
//  return	true on success
// 			false on failure
bool flashReset( void );

#endif _EEPROM_H_
//=======
#if 0
/**
  ******************************************************************************
  * @file    STM32F0xx_EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Exported constants --------------------------------------------------------*/
/* Define the size of the sectors to be used */
#define PAGE_SIZE             ((uint32_t)0x0400)  /* Page size = 1KByte */

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  ((uint32_t)0x08002000) /* EEPROM emulation start address:
                                                        from sector2, after 8KByte of used 
                                                        Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0400))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001)

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)0x03)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
