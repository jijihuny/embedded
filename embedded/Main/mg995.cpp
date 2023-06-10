#include "mg995.h"


void TCA0_init(void){
    PORTA_DIR |= PIN0_bm |
                 PIN1_bm |
                 PIN2_bm; //Set pin 0, 1, 2 direction of PORTA to output
    PORTMUX_TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    //Set PORTA as a WO pin
    TCA0_SINGLE_CTRLA &= ~TCA_SINGLE_ENABLE_bm;
    TCA0_SINGLE_CTRLESET = TCA_SINGLE_CMD_RESET_gc;
    TCA0_SINGLE_CTRLD &= ~TCA_SINGLE_SPLITM_bm;
    TCA0_SINGLE_CTRLB = TCA_SINGLE_CMP0EN_bm 
                        | TCA_SINGLE_CMP1EN_bm
                        | TCA_SINGLE_CMP2EN_bm
                        | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0_SINGLE_EVCTRL &= ~TCA_SINGLE_CNTEI_bm;
    TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0_SINGLE_INTFLAGS |= TCA_SINGLE_OVF_bm;
  
//    *TCA0_SINGLE_PERL = 0xC4;//Set TOP as 2500
//    *TCA0_SINGLE_PERH = 0x09;
//    *TCA0_SINGLE_CMP0L = 0x00;
//    *TCA0_SINGLE_CMP0H = 0x00;
    TCA0_SINGLE_PER = 0x09C4;
    TCA0_SINGLE_CMP0 = 0x00bb;
    TCA0_SINGLE_CMP1 = 0x00bb;
    TCA0_SINGLE_CMP2 = 0x00bb;
    TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
}


PID::PID(float dt, float Kp, float Ki, float Kd){
    this->dt = dt;
    this->PTerm = 0;
    this->ITerm = 0;
    this->DTerm = 0;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error = 0;
}
  
uint16_t PID::control(float angle, float Setpoint){
  
  float prev_error = this->error;
  
  this->error = Setpoint - angle;
  
  this->PTerm = (int16_t)(this->Kp * this->error);

  this->ITerm += (int16_t)(this->Ki * this->error * this->dt);
  
  float dError = this->error - prev_error;
  
  this->DTerm = (int16_t)(this->Kd * (dError / this->dt));

  int16_t sum = (this->PTerm + this->ITerm + this->DTerm);
  if(0xbb + sum > 250)return 250;
  else if(0xbb + sum < 130)return 130;
  else return 0xbb + sum;
}
