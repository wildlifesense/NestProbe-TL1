/********************************************************************************

  uart.h: Header file for the uart interface module of NestProbe TL1 firmware.
 
  Copyright (C) 2017 Nikos Vallianos <nikos@wildlifesense.com>
  
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
#ifndef UART_H_
#define UART_H_

// UART module control
void uartEnable(void);
void uartDisable(void);

void uartRxInterruptEnable(void);
void uartRxInterruptDisable(void);
// UART data exchange
void uartSendByte(uint8_t data);
uint8_t uartReceiveByte(void);
void uartSendWord(uint16_t data);
void uartSendString(const char myString[]);
void uartPrintWord(uint16_t word);
void uartPrintBinaryByte(uint8_t byte);
void uartPrintBinaryWord(uint16_t word);

#endif /* UART_H_ */
