# NestProbe TL1
A low cost, low power, and small-sized temperature logger.

The NestProbe TL1 is a low cost, high accuracy, long-life temperature logger
developed at Wildlife Sense for sea turtle egg incubation research.

Development has been partly documented at [hackaday.io](https://hackaday.io/project/27560-low-costpowersize-temperature-logger).

The main components in the TL1 are:

|Microcontroller|ATmega328PB|
|---------------|-----------|
| Temperature sensor          | MAX30205     |

Microcontroller:    ATmega328PB
Temperature sensor: 
It is powered by a CR2032 coin-cell battery and uses an 
a MAX30205 temperature sensor, an AT25DN512C EEPROM memory and a
ECS-.327-7-34B-C-TR 32.768kHz crystal for the real-time counter.

Details of the WSTL18 can be found at
https://hackaday.io/project/27560-low-costpowersize-temperature-logger

Copyright (C) 2017 Nikos Vallianos

The WSTL18 firmware and Eagle files are released under the GNU General Public License version 3
(See COPYING for details).

