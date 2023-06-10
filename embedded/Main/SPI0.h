#ifndef __SPI0_H__
#define __SPI0_H__

#include <stdint.h>
#include <avr/io.h>

#define READ_FLAG (uint8_t) (0x80)

#define WRITE_FLAG ~((uint8_t) (0x80))
//define addresses
//SPI0 : 0x08c0
//#define SPI0_CTRLA (uint8_t *) (0x08C0)
//#define SPI0_CTRLB (uint8_t *) (0x08C1)
//#define SPI0_INTCTRL (uint8_t *) (0x08C2)
//#define SPI0_INTFLAGS (uint8_t *) (0x08C3)
//#define SPI0_DATA (uint8_t *) (0x08C4)
//
//#define SPI_DORD_bm (0x01 << 6)
//#define SPI_MASTER_bm (0x01 << 5)
//#define SPI_PRESCDIV16_gc (0x01 << 1)
//#define SPI_ENABLE_bm (0x01)
//
//#define SPI_MODE3_gc (0x03)
//#define SPI_BUFEN_bm (0x01 << 7)
//
//#define SPI_SSD_bm (0x01 << 2)
//#define SPI_RXCIE_bm (0x01 << 7)
//#define SPI_TXCIE_bm (0x01 << 6)
//#define SPI_DREIE_bm (0x01 << 5)
//#define SPI_SSIE_bm (0x01 << 4)
//#define SPI_IE_bm (0x01)
//#define SPI_IF_bm (0x01 << 7)
//#define SPI_WRCOL_bm (0x01 << 6)
//
////PORTA : 0x0400
//#define PORTA_DIR (uint8_t *) (0x0400)
//#define PORTA_OUT (uint8_t *) (0x0404)
//#define PIN4_bm (0x01 << 4)
//#define PIN5_bm (0x01 << 5)
//#define PIN6_bm (0x01 << 6)
//#define PIN7_bm (0x01 << 7)
//
////PORTMUX : 0x05E0
//#define PORTMUX_TWISPIROUTEA (uint8_t *) (0x05E4)
//#define PORTMUX_SPI0DEFAULT_gc (0x0)


void CLOCK_init(void);
void SPI0_init(void);
void PORTA_init(void);
void SPI0_write(uint8_t, uint8_t);
uint8_t SPI0_read(uint8_t);
void SPI0_readbytes(uint8_t, uint16_t, uint8_t*);
void Slave_select(void);
void Slave_deselect(void);





#endif
