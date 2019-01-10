# NestProbe TL1

The NestProbe TL1 is a low cost, high accuracy, long-life temperature logger
developed at Wildlife Sense for sea turtle egg incubation research. This repository

Some aspects and decisions during development have been documented at [hackaday.io/project/27560-low-costpowersize-temperature-logger](https://hackaday.io/project/27560-low-costpowersize-temperature-logger).

The main components in the TL1 are:

|Microcontroller|ATmega328PB|
|:--------------|-----------|
| Temperature sensor          | MAX30205     |
| EEPROM | AT25DN512C |
| Crystal  | ECS-.327-7-34B-C-TR (32.768kHz) |

crystal for the real-time counter.

It is powered by a CR2032 coin-cell battery and uses an 

Details of the WSTL18 can be found at
https://hackaday.io/project/27560-low-costpowersize-temperature-logger

Copyright (C) 2017 Nikos Vallianos

The WSTL18 firmware and Eagle files are released under the GNU General Public License version 3
(See COPYING for details).

