#include "USART.h"
#include "arduino.h"
void USART1_init(void){

  PORTC_DIR |= PIN0_bm; //set rxd pin direction to input
  PORTC_DIR &= ~PIN1_bm; // set txd pin direction to output

  USART1_BAUD = (uint16_t) USART1_BAUD_RATE(BAUD_RATE);//set baud rate
  delay(20);
  USART1_CTRLB |= USART_TXEN_bm|USART_RXEN_bm; // Enable Receiver and Transmitter
  //USART1_CTRLC = USART_CHSIZE_9BITH_gc;
////                 
}


void USART1_Transmit(unsigned char c){
  while(!((USART1_STATUS) & USART_DREIF_bm)){};//waiting DREIF flag is set
  USART1_TXDATAL = c;
}

void USART1_string_send(char* str){
 for(int i=0;str[i]!='\0';i++){
 while(!((USART1_STATUS) & USART_DREIF_bm)){};//waiting DREIF flag is set
  USART1_TXDATAL = str[i];
  }
 // while(!(*USART1_STATUS & USART_TXCIF_bm));
 return;
}
void USART1_HEX_send(uint8_t hex){
  uint8_t temp[2];
  temp[0] = hex / 16;
  temp[1] = hex & 0x0f;
  for(uint8_t i=0; i<2; i++){
    if(temp[i] >= 0xA){
      temp[i] += 0x37;
    }
    else{
      temp[i] += 0x30;
    }
  }//Convert number to Hex(ASCII)
 USART1_Transmit(0x00);//Clear Buffer
 USART1_Transmit(0x30);//Send '0'
 USART1_Transmit(0x78);//Send 'x'
 for(int i=0;i<2;i++){
 USART1_Transmit(temp[i]);
 }
 return;
}

void USART1_DEC_send(uint32_t dec){
  uint8_t temp[8];
  bool sign = false;
  if(dec >= 0x80000000){
    sign = true;
    dec = ~dec;
    dec += 1;
  }
  temp[0] = (dec / 10000000) % 10;
  temp[1] = (dec / 1000000) % 10;
  temp[2] = (dec / 100000) % 10;
  temp[3] = (dec / 10000) % 10;
  temp[4] = (dec / 1000) % 10;
  temp[5] = (dec / 100) % 10;
  temp[6] = (dec / 10) % 10;
  temp[7] = dec % 10;
  uint8_t flag = 1;
  USART1_Transmit(0x00);//Clear buffer
  if(dec == 0){
    USART1_Transmit(0x30);
    return;
  }
  if(sign == true){
        USART1_Transmit(0x2D);
      }
  for(uint8_t i=0; i<8; i++){
      if(temp[i]==0 & flag == 1){
        continue;
      }
      else if(temp[i]!=0 & flag==1){
        flag = 0;
      }
      USART1_Transmit(temp[i]+0x30);
    }
  }


void USART1_bytes_send(uint8_t* bytes, uint8_t len){
  for(uint8_t i=0; i<len; i++){
    USART1_HEX_send(bytes[i]);
    USART1_Transmit(' ');
  }
  USART1_Transmit('\n');
}

void USART1_Accel_send(uint8_t* bytes){
  float data = ((((int16_t) bytes[0]) << 8) | bytes[1]);// * ACCEL_DIV;
  uint8_t exponent = (uint8_t) ((uint32_t)data >> 23);

  char* ex = "*10^";
  
  if((uint32_t)data & (uint32_t) 0x80000000){
    USART1_Transmit('-');
    USART1_DEC_send(((uint32_t)data>>16) & 0x7FFFFF);
    USART1_string_send(ex);
    if(exponent & (uint8_t) 0x40){
      USART1_Transmit('-');
    }
    else{
      USART1_Transmit('+');
    }
    USART1_DEC_send((uint32_t)exponent & 0x7F);
  }
  else{
    USART1_Transmit('+');
    USART1_DEC_send(((uint32_t)data) & 0x7FFFFF);
    USART1_string_send(ex);
    if(exponent & (uint8_t) 0x80){
      USART1_Transmit('-');
    }
    else{
      USART1_Transmit('+');
    }
    USART1_DEC_send((uint32_t)exponent & 0x7F);
  }
}
