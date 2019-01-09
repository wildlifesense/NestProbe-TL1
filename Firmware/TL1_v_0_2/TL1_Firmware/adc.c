/********************************************************************************

  adc.c: Internal analog to digital converter functions for the NestProbe TL1 firmware.
 
  Copyright (C) 2018 Nikos Vallianos <nikos@wildlifesense.com>
  
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

/*
 *  Initialize the ADC
 */
void adcEnable(void) {
    PRR0 &= ~(1<<PRADC);                  // Disable ADC power reduction mode. Not sure this is needed.
    ADMUX &= ~(1<<REFS1);					// Set AVcc as ...
	ADMUX |= (1<<REFS0);                   // ... reference voltage
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1);        // ADC clock prescaler / 64 = 9260SPS. Datasheet p.250
    ADCSRA |= (1 << ADEN);                  // Enable ADC
	ADMUX &= ~(0b1111);         // Reset channel bits
    ADMUX |= 0b1110;			// Set channel to measure internal 1.1V bandgap.
}

void adcDisable(void) {
	ADCSRA &= ~(1<<ADEN);		// Disable ADC
	PRR0 |= 1<<PRADC;			// Set ADC in power reduction register.
}

uint16_t adcReadVoltage(void) {
	adcEnable();
    uint16_t return_value = 0;
	for (uint8_t i=0; i<20; i++) {
		ADCSRA |= (1<<ADSC);                                // Start a single-ended conversion
		loop_until_bit_is_clear(ADCSRA, ADSC);
		return_value = ADC;
	}
	adcDisable();
	return return_value;
}


/*
 *  Set voltage reference for ADC
 *  'r' for AREF
 *  'v' for AVCC
 *  'i' for internal
 */
void adcVref(char vref) {
    if (vref=='r') {
        ADMUX  &= ~((1<<REFS1)|(1<<REFS0));     // Use AREF voltage reference pin
    } else if (vref=='v') {
        ADMUX  &= ~(1<<REFS1);                  // Use AVCC with external capacitor at AREF pin
        ADMUX  |= (1<<REFS0);
    } else if (vref=='i') {
        ADMUX  |= ((1<<REFS1)|(1<<REFS0));      // Use internal 1.1V ref with external capacitor at AREF pin
    }
}