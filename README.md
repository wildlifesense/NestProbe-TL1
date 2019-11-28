# NestProbe TL1

This repository has moved to [https://github.com/NikosVallianos/NestProbe/TL1](https://github.com/NikosVallianos/NestProbe/TL1)

I've used the TL1 reliably for over a year, but have also stared developing the TL2, which uses the Si7051 temperature sensor and the ATtiny816 MCU.
The NestProbe TL1 is a low cost, high accuracy, long-life temperature logger
developed at Wildlife Sense for sea turtle egg incubation research. This repository

Some aspects and decisions during development have been documented at [hackaday.io/project/27560-low-costpowersize-temperature-logger](https://hackaday.io/project/27560-low-costpowersize-temperature-logger).

The main components in the TL1 are:

| Component          | Model       |
|:-------------------|:------------|
| Microcontroller    | ATmega328PB |
| Temperature sensor | MAX30205    |
| EEPROM Memory      | AT25DN512C  |
| Crystal oscillator | ECS-.327-7-34B-C-TR (32.768kHz) |
| Battery type       | CR2032      |

Plus one SMD 0402 LED and a small number of pull-up resistors and decoupling capacitors.

The WSTL18 firmware and Eagle files are released under the GNU General Public
License version 3. This roughly means that you can use these files and designs,
manufacture the TL1 or derivatives and even sell them but you always have to
make the original source code or derivatives available under the same license.
See COPYING for details. 

