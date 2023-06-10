#include "SPI.h"
#define READ_FLAG (uint8_t) (0x80)
#define WRITE_FLAG ~((uint8_t) (0x80))

void SPI0_init(void){
  PORTA_init();//PORTA initialization
  *SPI0_CTRLA = SPI_ENABLE_bm//set SPI enable
              | SPI_PRESCDIV16_gc//set SPI clock frequency as 1MHz; maximum rate of MPU-9250 spi mode is 1MHz
              //default frequency of main clock source is 16Mhz
              | SPI_MASTER_bm//set SPI0 as master mode
              | SPI_DORD_bm;  //set Data order as LSB first
  *SPI0_CTRLB |= SPI_MODE3_gc | SPI_SSD_bm;
}

void PORTA_init(void){
  *PORTA_DIR |= PIN4_bm;//MOSI
  *PORTA_DIR &= ~PIN5_bm;//MISO
  *PORTA_DIR |= PIN6_bm;//SCK
  *PORTA_DIR |= PIN7_bm;//SS
}

uint8_t SPI0_read(uint8_t address){
  uint8_t resdata;//to store read data value
  Slave_select();//Set Slave select low(active)
  *SPI0_DATA = READ_FLAG | address;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  *SPI0_DATA = (uint8_t) 0x00;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  resdata = *SPI0_DATA;
  Slave_deselect();//Set Slave select High(inactive)
  return resdata;
}

void SPI0_readbytes(uint8_t address, int size, uint8_t* data){
  Slave_select();//Set Slave select low(active)
  *SPI0_DATA = READ_FLAG | address;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  for(int i=0; i<size; i++){
  *SPI0_DATA = (uint8_t) 0x00;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  data[i] = *SPI0_DATA;
  }
  Slave_deselect();//Set Slave select High(inactive)
}

void SPI0_write(uint8_t address, uint8_t data){
  Slave_select();
  *SPI0_DATA = WRITE_FLAG & address;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  *SPI0_DATA = data;
  while(!(*SPI0_INTFLAGS | SPI_IF_bm));
  Slave_deselect();
}

void Slave_select(void){
  *PORTA_DIR &= ~PIN7_bm;
}

void Slave_deselect(void){
  *PORTA_DIR |= PIN7_bm;
}
