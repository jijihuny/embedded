#include "mg995.h"



void TCA0_init(void){
    *PORTA_DIR = PIN0_bm |
                 PIN1_bm |
                 PIN2_bm; //Set pin 0, 1, 2 direction of PORTA to output
    *TCA0_CTRLA = TCA_CLKSEL_DIV8_gc | TCA_ENABLE_bm;//Set TCA0 clock source as 2MHz by prescaler, Enable TCA0
    *TCA0_CTRLB = TCA_CMPEN_bm | TCA_WGMODE_SINGLESLOPE_gc;//Enable WO, set TCA mode as single slope PWM mode
    *TCA0_INTCTRL = TCA_CMP2_bm |
                    TCA_CMP1_bm |
                    TCA_CMP0_bm |
                    TCA_OVF_bm;//Enable CMPn interrupts and OVF interrupt
    *TCA0_PERL = PERIOD;
    *TCA0_PERH = (PERIOD >> 8);//Set WO period to 20 msec
}

Servo::Servo(void){
    this->Pin = 0;
    this->CMPL = TCA0_CMP0L;
    this->CMPH = TCA0_CMP0H;
    this->angle = 0x0;
}

Servo::Servo(int Pin){
        this->Pin = Pin;
        switch (this->Pin)
        {
        case 0:
            this->CMPL = TCA0_CMP0L;
            this->CMPH = TCA0_CMP0H;
            break;
        case 1:
            this->CMPL = TCA0_CMP1L;
            this->CMPH = TCA0_CMP1H;
            break;
        case 2:
            this->CMPL = TCA0_CMP2L;
            this->CMPL = TCA0_CMP2H;
        default:
            this->CMPL = TCA0_CMP0L;
            this->CMPH = TCA0_CMP0H;
            break;
        }
        this->angle = 0;
    }

void Servo::ControlMortor(float angle){
        if(angle>60.0f | angle<-60.0f){
            return;
        }//to avoid overflow control

        this->angle = (uint16_t)MINDUTY + ((uint16_t)((angle+60.0f)/120.0))*(uint16_t)(MAXDUTY-MINDUTY);
        *this->CMPL = this->angle;
        *this->CMPH = (this->angle >> 8);
    }