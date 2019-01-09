/*
 * system.h
 *
 * Created: 31/03/2018 10:05:27
 *  Author: Nikos
 */ 


#ifndef SYSTEM_H_
#define SYSTEM_H_

void systemLabelLoad(void);
uint16_t systemLabelGet(void);
uint16_t systemLabelRead(void);
void systemLabelWrite(uint8_t label_msb, uint8_t label_lsb);
uint16_t systemBootloaderGetAddress(void);

#endif /* SYSTEM_H_ */