#ifndef __MG995_H__
#define __MG995_H__

#include <avr/io.h>
#include <stdint.h>

//PORTA 0x0400
//#define PORTA_DIR (uint8_t *) (0x0400)
//#define PIN0_bm (0x01)
//#define PIN1_bm (0x01<<1)
//#define PIN2_bm (0x01<<2)
//
////PORTMUX 0x05E0
//#define PORTMUX_TCAROUTEA (uint8_t *) (0x05E4)
//#define PORTMUX_TCA0_PORTA_gc (0x0)
//
////TCA0 0x0A00
//#define TCA0_CTRLA (uint8_t *) (0x0A00)
//#define TCA0_CTRLB (uint8_t *) (0x0A01)
//#define TCA0_CTRLC (uint8_t *) (0x0A02)
//#define TCA0_CTRLD (uint8_t *) (0x0A03)
//#define TCA0_CTRLESET (uint8_t *) (0x0A05)
//#define TCA0_EVCTRL (uint8_t *) (0x0A09)
//#define TCA0_INTCTRL (uint8_t *) (0x0A0A)
//#define TCA0_INTFLAGS (uint8_t *) (0x0A0B)
#define TCA0_SINGLE_PERL (uint8_t *) (0x0A26)
#define TCA0_SINGLE_PERH (uint8_t *) (0x0A27)
#define TCA0_SINGLE_CMP0L (uint8_t *) (0x0A28)
#define TCA0_SINGLE_CMP0H (uint8_t *) (0x0A29)
#define TCA0_SINGLE_CMP1L (uint8_t *) (0x0A2A)
#define TCA0_SINGLE_CMP1H (uint8_t *) (0x0A2B)
#define TCA0_SINGLE_CMP2L (uint8_t *) (0x0A2C)
#define TCA0_SINGLE_CMP2H (uint8_t *) (0x0A2D)
//#define TCA0_PERBUFL (uint8_t *) (0x0A36)
//#define TCA0_PERBUFH (uint8_t *) (0x0A37)
//#define TCA0_CMP0BUFL (uint8_t *) (0x0A38)
//#define TCA0_CMP0BUFH (uint8_t *) (0x0A39)
////The addresses of TCA0 for using Single slope PWM(16-bit) mode
//
//#define TCA_ENABLE_bm (0x01) //CTRLA
//#define TCA_CLKSEL_DIV8_gc (0x03 << 1) //CTRLA
//
//#define TCA_CMP0EN_bm (0x01 << 4) //CTRLB
//#define TCA_CMP1EN_bm (0x01 << 5)
//#define TCA_CMP2EN_bm (0x01 << 6)
//#define TCA_WGMODE_SINGLESLOPE_gc (0x03) //CTRLB
//
//#define TCA_CMP0OV_bm (0x01) //CTRLC
//#define TCA_CMP1OV_bm (0x01 << 1) //CTRLC
//#define TCA_CMP2OV_bm (0x01 << 2) //CTRLC
//
//#define TCA_SPLITM_bm (0x01)//CTRLD
//
//#define TCA_CMD_RESET_gc (0x03<<2)//CTRLE
//
//#define TCA_CNTEI_bm (0x01)//EVCTRL
//
////For INTCTRL and INTFLAGS
//#define TCA_CMP0_bm (0x01 << 4) 
//#define TCA_CMP1_bm (0x01 << 5)
//#define TCA_CMP2_bm (0x01 << 6)
//#define TCA_OVF_bm (0x01)
//
#define PERIOD (0x09c4) // for setting period(top) register to getting frequency 50Hz
#define MAXDUTY (0x00F0) // Max duty cycle 2ms of Waveform generator to fit servo
#define MINDUTY (0x00b4) // Min duty cycle 1ms


void TCA0_init(void);

class PID{
  public:
  float dt;
  int16_t PTerm;
  int16_t ITerm;
  int16_t DTerm;
  float Kp;
  float Ki;
  float Kd;
  float error;
  PID(float dt, float Kp, float Ki, float Kd);
  uint16_t control(float angle, float Setpoint);
};

#endif
