#ifndef __MG995_H__
#define __MG995_H__

#include <stdint.h>

//PORTA 0x0400
#define PORTA_DIR (uint8_t *) (0x0400)
#define PIN0_bm (0x01)
#define PIN1_bm (0x01<<1)
#define PIN2_bm (0x01<<2)
//TCA0 0x0A00
#define TCA0_CTRLA (uint8_t *) (0x0A00)
#define TCA0_CTRLB (uint8_t *) (0x0A01)
#define TCA0_CTRLC (uint8_t *) (0x0A02)
#define TCA0_INTCTRL (uint8_t *) (0x0A0A)
#define TCA0_INTFLAGS (uint8_t *) (0x0A0B)
#define TCA0_PERL (uint8_t *) (0x0A26)
#define TCA0_PERH (uint8_t *) (0x0A27)
#define TCA0_CMP0L (uint8_t *) (0x0A28)
#define TCA0_CMP0H (uint8_t *) (0x0A29)
#define TCA0_CMP1L (uint8_t *) (0x0A2A)
#define TCA0_CMP1H (uint8_t *) (0x0A2B)
#define TCA0_CMP2L (uint8_t *) (0x0A2C)
#define TCA0_CMP2H (uint8_t *) (0x0A2D)
//The addresses of TCA0 for using Single slope PWM(16-bit) mode

#define TCA_ENABLE_bm (0x01) //CTRLA
#define TCA_CLKSEL_DIV8_gc (0x03 << 1) //CTRLA
#define TCA_CMPEN_bm (0x01 << 4) //CTRLB
#define TCA_WGMODE_SINGLESLOPE_gc (0x03) //CTRLB
#define TCA_CMP0OV_bm (0x01) //CTRLC
#define TCA_CMP1OV_bm (0x01 << 1) //CTRLC
#define TCA_CMP2OV_bm (0x01 << 2) //CTRLC

//For INTCTRL and INTFLAGS
#define TCA_CMP0_bm (0x01 << 4) 
#define TCA_CMP1_bm (0x01 << 5)
#define TCA_CMP2_bm (0x01 << 6)
#define TCA_OVF_bm (0x01)

#define PERIOD (0x9c40U) // for setting period(top) register to getting frequency 50Hz
#define MAXDUTY (0x1450) // Max duty cycle 13% of Waveform generator to fit servo
#define MINDUTY (0x0578) // Min duty cycle 3.5%

void TCA0_init(void);
class Servo{
    private:
    int Pin;
    uint8_t* CMPL;
    uint8_t* CMPH;
    uint16_t angle;

    public:
    Servo(void);
    Servo(int);

    void ControlMortor(float);
};


#endif