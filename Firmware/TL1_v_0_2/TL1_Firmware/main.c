/********************************************************************************

  main.c: The main routine of the NestProbe TL1 firmware.
 
  Copyright (C) 2017-2018 Nikos Vallianos <nikos@wildlifesense.com>
  
  This file is part of NestProbe TL1 firmware.
  
  NestProbe TL1 firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version.
  
  NestProbe TL1 firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  in the file COPYING along with this program.  If not,
  see <http://www.gnu.org/licenses/>.

  ********************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define SIGRD 5					// reading ID may not work without this
#include <avr/boot.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>		// Used to read bytes from flash memory.
#include <util/crc16.h>			// Used to calculate firmware's CRC.
#include <util/delay.h>
#include "system.h"
#include "error.h"
#include "uart.h"
#include "adc.h"
#include "memory.h"
#include "max30205.h"
#include "indicator.h"
#include "spi.h"

#define system_soft_reset() \
do                          \
{                           \
	wdt_enable(WDTO_15MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)


char     logger_mode;
uint16_t logger_countdown;				// 8-second counts until logger begins recording temperatures.
uint16_t logger_interval;				// How many eight seconds counts till logging interval reached.
uint16_t logger_temperature;
uint16_t logger_memory_location;		// Initialize to 0 or not?
uint16_t logger_battery_level;
volatile uint16_t logger_eightseconds_count;		// Number of eightsecond counts since last log

#define FIRMWARE_VERSION_MAJOR 0		// 0-16 so it can squeeze into left-half of 8-bit value.
#define FIRMWARE_VERSION_MINOR 2		// 0-16 so it can squeeze into right-half of 8-bit value.
#define FIRMWARE_GET_VERSION ((uint8_t)(FIRMWARE_VERSION_MAJOR<<4|FIRMWARE_VERSION_MINOR))	// Get version in one byte.

#define LOGGER_HEADER_SIZE	22
#define LOGGER_FOOTER_SIZE	22
#define RX_RECEIVE_TIMEOUT	100					// Time window to receive command, in ms
#define RX_BUFFER_LENGTH	20
volatile uint8_t rx_buffer_index;						// Points to current writing location in buffer.
volatile uint8_t rx_buffer_array[RX_BUFFER_LENGTH];		// Buffer for bytes received through uart.

uint8_t mcu_serial_number[10];			// Unique serial number.

// MCU flash memory firmware and bootloader parameters.
uint16_t mcu_bootloader_first_byte;			// Byte address of first bootloader byte. Depends on BOOTSZ fuses.
uint16_t mcu_firmware_available_pages;		// Number of pages available for application firmware.
uint16_t mcu_firmware_last_byte;			// Byte address of last byte of current firmware. Helps calculate firmware size and CRC.
uint16_t mcu_firmware_crc16_xmodem;			// Xmodem-CRC16 of mcu firmware.
//uint16_t last_data_byte;					// Last byte with data on memory chip. Stores scan at boot to know if there's resident data on memory chip.
void rxBufferClear(void);				// Clear rx buffer and index.
uint8_t rxBufferIsAllZeros(void);		// Rx buffer after 1st character is filled with zeros. Serves as indirect confirmation.
uint8_t rxBufferCommandIsReady(void);	// Rx buffer has received a complete command string.
uint8_t rxBufferTimestampIsValid(void); // Bytes 1-15 in Rx buffer are a valid timestamp.
void hostCommandReceive(void);			// Send host info string and receive a command.


/*
 * Timer 2 is the Real Time Counter, set to fire every 8 seconds.
 */
ISR(TIMER2_OVF_vect) {
	logger_eightseconds_count++;
}



ISR(USART0_RX_vect) {
	if (rx_buffer_index<RX_BUFFER_LENGTH) {
		rx_buffer_array[rx_buffer_index] = UDR0;
		rx_buffer_index++;	
	} else {
		errorSetFlag(ERROR_FLAG_RX_BUFFER_OVF);
	}
}




/*
 * MAIN function, consists of initialization code and an eternal while(1) loop.
 */
int main(void) {
	// Initialize the system. So many things!
	MCUSR = 0;
	wdt_disable();			// Disable watchdog timer, in case this was preserved after soft-reset.
	// Disable brown-out detector in sleep mode.
	MCUCR |= (1<<BODSE)|(1<<BODS);
	MCUCR |= (1<<BODS);
	MCUCR &= ~(1<<BODSE);

	// Disable ADC
	ADCSRA &= ~(1<<ADEN);					// Disable ADC, stops the ADC clock.

	// TODO: Disable watchdog timer (Fuse FUSE_WDTON, defaults to ...) Also WDTCSR
	// TODO: Disconnect the bandgap reference from the Analog Comparator (clear the ACBG bit in	ACSR (ACSR.ACBG)).
	ACSR |= (1<<ACD);						// Disable analog comparator (set to disable).

	PRR0 |= (1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRSPI0)|(1<<PRUSART0)|(1<<PRADC);
	PRR1 |= (1<<PRTWI1)|(1<<PRPTC)|(1<<PRTIM4)|(1<<PRSPI1)|(1<<PRTIM3);

	// SM[2:0] = b'010' for power_down and b'011' for power_save
	//SMCR &= ~((1<<SM2)|(1<<SM0));
	//SMCR |= (1<<SM1);

	// Set all ports to input and high before initializing modules that may override these as necessary.
	// Turn all unused pins into inputs with pull-ups.
	DDRB  = 0x00;
	PORTB = 0xFF;
	DDRC  &= ~((1<<DDRC0)|(1<<DDRC1)|(1<<DDRC2)|(1<<DDRC3)|(1<<DDRC4)|(1<<DDRC5)|(1<<DDRC6));		// Port C is 7 bits (0-6).
	PORTC |= ((1<<PORTC0)|(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)|(1<<PORTC5)|(1<<PORTC6));
	DDRD  = 0x00;
	PORTD = 0xFF;
	DDRE  &= ~((1<<DDRE0)|(1<<DDRE1)|(1<<DDRE3));
	PORTE |= ((1<<PORTE0)|(1<<PORTE1)|(1<<PORTE3));
	DDRE |= (1<<DDRE2);
	PORTE &= ~(1<<PORTE2);					// Stays here
	
	
	//Digital input buffers can be disabled by writing to the Digital Input Disable Registers (DIDR0 for ADC, DIDR1 for AC). (found at http://microchipdeveloper.com/8avr:avrsleep)
	//If the On-chip debug system is enabled by the DWEN Fuse and the chip enters sleep mode, the main clock source is enabled and hence always consumes power. In the deeper sleep modes, this will contribute significantly to the total current consumption.

	errorInitFlags();
	indicatorInitialize();
		
	// Start the Real Time Counter. Takes 1000ms+ to allow crystal to stabilize.
	PRR0	&= ~(1<<PRTIM2);								// Clear timer2 bit at power reduction register.
	ASSR	|= (1<<AS2);									// Clock from external crystal.
	TCCR2B	|= (1<<CS20)|(1<<CS21)|(1<<CS22);				// Tosc/1024 prescaler = 8sec to overflow.
	TIMSK2	= 0;
	TIMSK2	|= (1<<TOIE2);									// Enable overflow interrupt
	//_delay_ms(1000);										// Allow RTC crystal to stabilize (RTC AN p.5).
	TCNT2	= 0;											// Clear counter value.
	sei();	
	
	max30205Init();									// Initialize temperature sensor chip into lowest power consumption.
	
	// Initialize memory chip and set it to low power.
	memoryInitialize();								// Initialize memory chip.
	logger_memory_location = memoryScan();			// Find byte address of first blank word. Should be 0.
	memoryUltraDeepPowerDownEnter();				// Place memory chip into lowest power consumption.
	
	// Check memory if empty. If not, set logger in 'Holding' mode.
	if (logger_memory_location) {
		logger_mode = 'H';
	} else {
		logger_mode = 'I';
	}


	// Load MCU serial number.
	for (uint8_t i=0; i<10; i++) {
		mcu_serial_number[i] = boot_signature_byte_get(i+14);	// Read signature bytes 14-23
	}
	systemLabelLoad();
	
	mcu_bootloader_first_byte = systemBootloaderGetAddress();
	mcu_firmware_available_pages = mcu_bootloader_first_byte / SPM_PAGESIZE;

	// Find the last byte of firmware in flash.
	for (uint16_t k=(mcu_bootloader_first_byte-1); k>0; k--) {
		if (pgm_read_byte(k) != 0xFF) {
			mcu_firmware_last_byte = k;
			break;
		}
	}

	// Calculate firmware xmodem CRC16
	mcu_firmware_crc16_xmodem = 0;
	for (uint16_t i=0; i<=mcu_firmware_last_byte; i++) {
		mcu_firmware_crc16_xmodem = _crc_xmodem_update(mcu_firmware_crc16_xmodem, pgm_read_byte(i));
	}

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	/*******************************************
	*               MAIN LOOP                  *
	*******************************************/
    while (1) {
		sleep_mode();
		if (logger_mode == 'L') {
			if (logger_countdown>0) {
				logger_countdown--;
				logger_eightseconds_count = 0;
			} else if (logger_eightseconds_count<logger_interval) {
					// Do nothing. logger_eightseconds_count is incremented at ISR
			} else {
				if (logger_memory_location < LOGGER_HEADER_SIZE) {	// Wrapped through end of memory? (No way!)
					errorSetFlag(ERROR_FLAG_MEM_ADDRESS_ERR);
					logger_mode = 'H';
				} else {
					memoryUltraDeepPowerDownExitBegin();
					max30205StartOneShot();
					_delay_ms(60);										// Wait for MAX30205 to read temperature.
					logger_temperature = max30205ReadTemperature();
					_delay_ms(20);										// Wait for memory to recover from deep power down.
					spiEnable();
					memoryWriteWord(logger_memory_location, logger_temperature);
					spiDisable();
					logger_memory_location += 2;
					logger_eightseconds_count = 0;
					_delay_ms(7);	// TODO: Any chance memory write isn't done and pwr down is skipped? 
					memoryUltraDeepPowerDownEnter();
				}
			}
		} else if (logger_mode=='H') {
			; // Do nothing, for now
		} else if (logger_mode=='I') {
			;
		} // End of mode actions if-implemented 'switch'
		
		
		if ( !(PIND & (1<<PIND2))) { // PD2 is down - the uart cable is connected.
			hostCommandReceive();
		}
		indicatorShortBlink();
	}// END OF MAIN LOOP
}// END OF MAIN




/*
 * Clear the rx buffer and its index.
 */
void rxBufferClear(void) {
	for(uint8_t i=0; i<RX_BUFFER_LENGTH; i++) {
		rx_buffer_array[i] = 0;
	}
	rx_buffer_index = 0;
}

uint8_t rxBufferIsAllZeros(void) {
	for (uint8_t i=1; i<20; i++) {
		if (rx_buffer_array[i]!=0) return 0;
	}
	return 1;
}

uint8_t rxBufferCommandIsReady(void) {
	if (rx_buffer_index==20) {
		if ((rx_buffer_array[0]>'A') && (rx_buffer_array[0]<'Z')) {	// Commands are only cap letters
			return 1;
		}
	}
	return 0;
}

/*
 * Check that a timestamp in the rx buffer is valid. Also check the 'time source'
 * character, which I consider part of the timestamp. By protocol, they are
 * always at the same location in the array.
 */
uint8_t rxBufferTimestampIsValid(void) {
	// Check that rx_buffer_array bytes 1-18 are all ASCII numerals ('0' to '9')
	// This check can be improved in future.
	for (uint8_t i=1; i<15; i++) { // Check timestamp section of command string
		if ((rx_buffer_array[i] < '0') || (rx_buffer_array[i] > '9')) {
			return 0;
		}
	}
	if ((rx_buffer_array[15]!='A') && (rx_buffer_array[15]!='L')) {
		return 0;
	}
	return 1;
}

/*
 * Send host a string of info and receive a command.
 */
void hostCommandReceive(void) {
	// Send logger info string (4 bytes of model, 3 bytes of MCU ID (why?), 10 bytes of MCU serial number (OK), 2 bytes of firmware CRC, 6 status-dependent bytes.
	logger_battery_level = adcReadVoltage();
	rxBufferClear();									// Clear rx_buffer_array and rx_buffer_index.
	uartEnable();										// Enable uart module.
	uartRxInterruptEnable();							// Enable RX receive interrupt.

	uartSendString("TL01");							// 4 bytes of system model.
	for (uint8_t i=0; i<10; i++) {
		uartSendByte(mcu_serial_number[i]);				// 10 bytes of mcu serial number
	}
	uartSendWord(systemLabelGet());					// 2 bytes of local serial.
	uartSendByte(FIRMWARE_GET_VERSION);					// 1 byte of firmware version
	uartSendWord(mcu_firmware_crc16_xmodem);			// 2 bytes of firmware CRC16-xmodem
	uartSendWord(logger_battery_level);					// 2 bytes of battery level or 0 if not available
	uartSendWord(errorGetFlags());						// 2 bytes of error flags (0x0000 for no errors)
	uartSendWord(0x0000);								// 2 bytes reserved for future use
	// Hereon depends on status. Work on this.
	uartSendByte(logger_mode);						// Status: [I]dle, [L]ogging, [H]olding, [D]ownloaded.
	if (logger_mode=='L') {
		uartSendWord(logger_memory_location);			// 2 bytes of data length (bytes, including header)
		uartSendWord(logger_countdown);					// 2 bytes of logger countdown (deferral)
		uartSendWord(logger_eightseconds_count);		// 2 bytes of current eightseconds count.
		uartSendWord(logger_interval);					// 2 bytes of logger interval.		
	} else if (logger_mode=='H') {
		uartSendWord(logger_memory_location);			// 2 bytes of data length (bytes, including header)
		uartSendWord(0x0000);							// 2 bytes of data CRC
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
	} else if (logger_mode=='I') {
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
	} else {
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
		uartSendWord(0x0000);							// 2 bytes reserved for future use
	}

	_delay_ms(RX_RECEIVE_TIMEOUT);		// Wait for command string to arrive at rx_buffer_array via RX ISR.
										// Delay instead of polling or counting bytes prevents lock-down.
	uartDisable();						// End of command string receival. Shut the uart module down.
	
	if (rxBufferCommandIsReady()) {
		// Commands applicable to all modes
		if (rx_buffer_array[0] == 'N') {
			; // Do nothing
		} else if (rx_buffer_array[0] == 'X') {
			if (rxBufferIsAllZeros()) {
				errorClearAll();
			}
		} else if (rx_buffer_array[0] == 'E') {
			if (logger_mode=='L' && rxBufferTimestampIsValid()) {
				/*
				 * TODO: Why doesn't this code put memory back to ultra-deep power down?
				 */
				spiEnable();
				// Write Data CRC to memory (2 bytes)
				memoryWriteByte(logger_memory_location, 'C');
				logger_memory_location++;
				memoryWriteByte(logger_memory_location, 'C');
				logger_memory_location++;
				// Write number of records to memory (2 bytes)
				memoryWriteByte(logger_memory_location, 'N');
				logger_memory_location++;
				memoryWriteByte(logger_memory_location, 'N');
				logger_memory_location++;
				// Append rx_buffer_array bytes 
				memoryWriteArray(logger_memory_location, (uint8_t *) &rx_buffer_array[1], 15);
				logger_memory_location += 15;
				memoryWriteByte(logger_memory_location, 0x00);
				logger_memory_location++;
				memoryWriteByte(logger_memory_location, 'C');
				logger_memory_location++;
				memoryWriteByte(logger_memory_location, 'C');
				logger_memory_location++;
				spiDisable();
				// Also set status/mode to 'H'
				logger_mode = 'H';
			}
		} else if (rx_buffer_array[0] == 'D') {
			if (rxBufferIsAllZeros()) {
				if (logger_mode=='I' /*DEBUG*/ || logger_mode=='L' || logger_mode=='H' || logger_mode=='D')	// Modify
					memoryDumpUpto(logger_memory_location);	// Stream out memory bytes, byte per byte, from 0 to logger_memory_location.
				if (logger_mode=='H')
					logger_mode='D';
			}
		} else if (rx_buffer_array[0] == 'C') { // Something in here gets stuck
			if (logger_mode=='D' && rxBufferIsAllZeros()) {
				/*
				 * TODO: Why doesn't this code put memory back to ultra-deep power down?
				 */
				spiEnable();
				memoryEraseChip();	//Clear memory
				spiDisable();
				logger_mode = 'I';
				logger_memory_location = 0;		// Rewind to 0
			}
		} else if (rx_buffer_array[0] == 'U') {
			;// Do a firmware update. Here, just reset device.
		} else if (rx_buffer_array[0] == 'B') {
			if (logger_mode=='I' && rxBufferTimestampIsValid()) {
					logger_countdown = rx_buffer_array[16];
					logger_countdown <<= 8;
					logger_countdown |= rx_buffer_array[17];
					logger_interval = rx_buffer_array[18];
					logger_interval <<= 8;
					logger_interval |= rx_buffer_array[19];
					// Copy format version (0x01), and rx_buffer_array 1-18 to first 22 bytes in EEPROM.
					/*
					 * TODO: Why doesn't this code put memory back to ultra-deep power down?
					 */
					spiEnable();
					memoryWriteByte(0x0000, 0x01);						// Format version
					memoryWriteArray(0x0001, (uint8_t *) &rx_buffer_array[1], 19);	// Copy timestamp, source, countdown, and interval to memory array.
					memoryWriteByte(0x0014, 'C');	// Memory address 20 (to be CRC MSB)
					memoryWriteByte(0x0015, 'C');	// Memory address 21 (to be CRC LSB)
					spiDisable();
					logger_memory_location = 22;				// For next (first) temperature log
					logger_mode = 'L';							// Set logger_status to 'L'
					logger_eightseconds_count = 0;				// Reset eightseconds counter.
			} 
		} else if (rx_buffer_array[0] == 'S') {		// Set logger label number.
			if (rx_buffer_array[2]) {	// 2nd byte must have a value. 1st can have a value or be 0.
				systemLabelWrite(rx_buffer_array[1], rx_buffer_array[2]);
			}
		}
	} else {  // Logger didn't receive complete command
        errorSetFlag(ERROR_FLAG_COMMAND_ERR);
    }
}
