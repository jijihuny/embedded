#include "SPI0.h"


void CLOCK_init(void){
  CPU_CCP = CCP_IOREG_gc;
  //Set Configuration Change Protection register as IOREG mode
  //Un-protect protected I/O register
  CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
  //Let Prescaler enable and set division as 16(CLK_PER = 1MHz)
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
  //Waiting Clock is available and prescaler is switched
}

void SPI0_init(void){
  PORTA_init();
  PORTMUX_TWISPIROUTEA = PORTMUX_SPI0_DEFAULT_gc;
  SPI0_CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_CLK2X_bm;// Set SPI frequency 1MHz
  SPI0_CTRLA &= ~SPI_DORD_bm;
  SPI0_CTRLB |= SPI_MODE_3_gc | SPI_SSD_bm;
  SPI0_CTRLA |= SPI_ENABLE_bm;
  
}


void PORTA_init(void){
  PORTA_DIR |= PIN4_bm | PIN6_bm | PIN7_bm;
  PORTA_DIR &= ~PIN5_bm;
  
}


void SPI0_write(uint8_t address, uint8_t data){
  Slave_select();
  SPI0_DATA = WRITE_FLAG & address;
  while(!(SPI0_INTFLAGS & SPI_IF_bm));
  SPI0_DATA = data;
  while(!(SPI0_INTFLAGS & SPI_IF_bm));
  Slave_deselect();
}

uint8_t SPI0_read(uint8_t address){
  uint8_t resdata;
  Slave_select();
  SPI0_DATA = READ_FLAG | address;
  while(!(SPI0_INTFLAGS & SPI_IF_bm));
  SPI0_DATA = 0xff;
  while(!(SPI0_INTFLAGS & SPI_IF_bm));
  resdata = SPI0_DATA;
  Slave_deselect();
  return resdata;
}

void SPI0_readbytes(uint8_t address, uint16_t len, uint8_t* data){
  Slave_select();//Set Slave select low(active)
  SPI0_DATA = READ_FLAG | address;
  while(!(SPI0_INTFLAGS & SPI_IF_bm));
  for(int i=0; i<len; i++){
    SPI0_DATA = 0xff;
    while(!(SPI0_INTFLAGS & SPI_IF_bm));
    data[i] = SPI0_DATA;
  }
  Slave_deselect();//Set Slave select High(inactive)
}

void Slave_select(void){
  PORTA_OUT &= ~PIN7_bm;
}

void Slave_deselect(void){
  PORTA_OUT |= PIN7_bm;
}
