#include "mg995.h"
//Servo mortor
#include "mpu9250.h"
#include "USART.h"

int16_t acc_bias[3] = {0, 0, 0};
int16_t gyro_bias[3] = {0, 0, 0};


float angle[2] = {0, 0};
float Setpoint[2] = {0, 0};

PID motor0(0.02, 4, 1, 0);
PID motor1(0.02, 4, 1, 0);
PID motor2(0.02, -4, -1, -0);

uint16_t motorcontrol[3] = {0, 0, 0};

void setup() {
  sei();
  // put your setup code here, to run once:
  CLOCK_init();
  USART1_init();
  TCA0_init();
  SPI0_init();
  MPU9250_init(AFS_2G, GFS_250DPS, 0);
  delay(200);
  Calibrate();
  delay(100);
}


void loop() {
  // put your main code here, to run repeatedly:
  ComplimentaryFilter(0.5, 0.02);//Filter, Sampling rate 50Hz
  motorcontrol[0] = motor0.control(angle[0], 0.0f);
  motorcontrol[1] = motor1.control(angle[1], 0.0f);
  motorcontrol[2] = motor2.control(angle[1], 0.0f);
  delay(20);
}


ISR(TCA0_OVF_vect){

  TCA0_SINGLE_CMP0 = motorcontrol[0];
  TCA0_SINGLE_CMP1 = motorcontrol[1];
  TCA0_SINGLE_CMP2 = motorcontrol[2];
  
  TCA0_SINGLE_INTFLAGS |= TCA_SINGLE_OVF_bm;//Clear OVF interrupt flag
}
