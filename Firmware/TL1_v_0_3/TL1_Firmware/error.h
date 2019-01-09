/********************************************************************************

  error.h: Header file for the error module of the NestProbe TL1 firmware..
 
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
#ifndef ERROR_H_
#define ERROR_H_

#define ERROR_FLAG_LOGGING			0		// Is the device currently logging? 0: No, 1: Yes
#define ERROR_FLAG_ERROR_MEM		1		// Encountered memory error?		0: No, 1: Yes
#define ERROR_FLAG_ERROR_TMP		2		// Encountered error with temperature sensor?	0: No, 1: Yes
#define ERROR_FLAG_RX_BUFFER_OVF	3		// Command received via uart overflowed buffer.
#define ERROR_FLAG_COMMAND_ERR		4		// General command receive error. Most likely incomplete connection.
#define ERROR_FLAG_MEM_ADDRESS_ERR	5		// Memory (EEPROM) address error.
#define ERROR_FLAG_STH2				6

void errorInitFlags(void);
void errorSetFlag(uint16_t flag_to_set);
void errorClearFlag(uint16_t flag_to_clear);
void errorClearAll(void);
uint16_t errorGetFlags(void);

#endif /* ERROR_H_ */