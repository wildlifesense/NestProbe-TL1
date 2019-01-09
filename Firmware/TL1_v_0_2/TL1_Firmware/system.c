/*
 * system.c
 *
 * Created: 31/03/2018 10:05:03
 *  Author: Nikos
 */ 
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include "uart.h"

#define SYSTEM_LABEL_ADDR_MSB	0
#define SYSTEM_LABEL_ADDR_LSB	1

uint16_t system_label;					// Two-byte label to identify logger 
uint16_t system_bootloader_address;		// Address of first byte of bootloader. Used for backwards-seek of last firmware byte.


/*
 * Read label bytes from MCU EEPROM to system module variables.
 */
void systemLabelLoad(void) {
	system_label = eeprom_read_byte((uint8_t *) SYSTEM_LABEL_ADDR_MSB);
	system_label <<= 8;
	system_label |= eeprom_read_byte((uint8_t *) SYSTEM_LABEL_ADDR_LSB);
}

/*
 * Get label as a 16-bit number.
 */
uint16_t systemLabelGet(void) {
	return system_label;
}

/*
 * Read label from EEPROM into module, then return it.
 */
uint16_t systemLabelRead(void) {
	systemLabelLoad();
	return systemLabelGet();
}

/*
 * Write two bytes of label to EEPROM. First byte is MSB, second byte is LSB.
 */
void systemLabelWrite(uint8_t label_msb, uint8_t label_lsb) {
	eeprom_update_byte((uint8_t *) SYSTEM_LABEL_ADDR_MSB, label_msb);
	eeprom_update_byte((uint8_t *) SYSTEM_LABEL_ADDR_LSB, label_lsb);
	// Verify EEPROM write busy doesn't cause any problems here.
	systemLabelLoad();
}

uint16_t systemBootloaderGetAddress(void) {
	// Load bootloader start address and firmware available pages
	uint8_t bootsz_bits = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
	bootsz_bits = bootsz_bits & 0b00000110;					// Keep only bootsz bits. 1: unseet, 0: set
	bootsz_bits = bootsz_bits >> 1;									// Move bootsz bits to base
	bootsz_bits = ~(bootsz_bits) & 0b11;							// Reverse bootsz bits and filter out other bits
	// Bootsz bits are now 0-3 representing 4, 8, 16, or 32 pages of boot sector.
	system_bootloader_address = 0x8000 - (1<<bootsz_bits)*512;		// bootsector size = math.pow(2, bootsz) * 512
	return system_bootloader_address;
}


/*
#include <avr/boot.h>
uint8_t		system_mcu_id[3];						// Identifies a 328PB.
	// Load MCU ID.
system_mcu_id[0] = boot_signature_byte_get(0);
system_mcu_id[1] = boot_signature_byte_get(2);
system_mcu_id[2] = boot_signature_byte_get(4);
*/