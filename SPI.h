#ifndef __SPI_H__
#define __SPI_H__
//define addresses
//SPI0 : 0x08c0
//PORTA : 0x0400

void CLOCK_init(void);
void SPI0_init(void);
void PORTA_init(void);
void SPI0_write(uint8_t ,uint8_t);
uint8_t SPI0_read(uint8_t);
void SPI0_readbytes(uint8_t, int, uint8_t*);
void Slave_select(void);
void Slave_deselect(void);





#endif