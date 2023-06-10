#include "mpu9250.h"
#include "arduino.h"

 
void MPU9250_init(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate){
    //wake up device
    SPI0_write(PWR_MGMT_1, 0x00);//Clear sleep mode bit(6), enable all sensors
    delay(100);//wait for all registers to reset

    //get stable time source
    SPI0_write(PWR_MGMT_1, 0x00);//Auto select clock source to be PLL gyroscope reference if ready else
    delay(200);

    //Configure Gyro and Thermometer
    //Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42Hz, respectively
    //minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    //be higher than 1 / 0.0059 = 170Hz
    //DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    //With the MPU9250, it is possible to get gyro sample rates of 32kHz, 8kHz, or 1kHz
    SPI0_write(CONFIG, 0x03);

    //Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    SPI0_write(SMPLRT_DIV, sampleRate);//Use a 200 Hz rate; a rate consistent with the filter update rate
                                       //determined inset in CONFIG above

    //Set gyroscope full scale range
    //Range selects FS_SEL and AFS_SEL are 0-3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = SPI0_read(GYRO_CONFIG);//get current GYRO_CONFIG register value
    c &= ~0xE0;//Clear self-test bits [7:5]
    c &= ~0x02;//Clear Fchoice bits [1:0]
    c &= ~0x18;//Clear GFS bits[4:3]
    c |= Gscale << 3;//Set full scale range for the gyro
    c &= ~0x03;//Set Fchoice for the gyro to 11 by its inverse to bits 1:0 of GYRO_CONFIG
    SPI0_write(GYRO_CONFIG, c);//write new GYRO_CONFIG value to register

    //Set accelerometer full-scale range configuration
    c = SPI0_read(ACCEL_CONFIG);//get current ACCEL_CONFIG register value

    c &= ~0x18;//Clear self-test bits [7:5]
    c |= Ascale << 3;//set full-scale range for the accelerometer
    SPI0_write(ACCEL_CONFIG, c);//write new ACCEL_CONFIG register value

    //set accelerometer sample rate configuration
    //It is possible to get a 4kHz sample rate from the accelerometer by choosing 1 for
    //accel_fchoice_b bit[3]; in this case the bandwidth is 1.13kHz
    c = SPI0_read(ACCEL_CONFIG2);
    c &= ~0x0F;//Clear accel_fchoice_b (bit[3]) and A_DLPFG (bits [2:0])
    c |= 0x03;//Set accelerometer rate to 1kHz and bandwidth to 44.8Hz
    SPI0_write(ACCEL_CONFIG2, c);//Write new ACCEL_CONFIG2 register value

    //The accelerometer, gyro, and thermometer are set to 1kHz sample rates,
    //but all these rates are further reduced by a factor of 5 to 200Hz because of the SMPLRT_DIV setting

    //Configure Interrupts and Bypass Enable
    //Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    //clear on read of INT_STATUS, and enable SPI mode only
    //All can be controlled by the ATmega4809 as master
    SPI0_write(INT_PIN_CFG, 0x12);//INT is 50 microsecond pulse and any read to clear
    SPI0_write(INT_ENABLE, 0x01);//Enable data ready(bit[0]) interrupt
    delay(100);

    SPI0_write(USER_CTRL, 0x10);//Disable I2C slave module and put the serial interface in SPI mode only
}

//void calibrate_sensor(float* accel, float* gyro){
//    uint8_t data[12];//data array to hold accel and gyro x, y, z data
//    uint16_t packet_count, fifo_count;
//    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
//
////    SPI0_write(PWR_MGMT_1, 0x80);//reset device
////    delay(100);
////    
////    //get stable time source; Auto select clock source to be PLL gyroscope reference if ready
////    //else use the internel osillator, bits 2:0 = 001
////    SPI0_write(PWR_MGMT_1, 0x01);//Auto select clock source
////    SPI0_write(PWR_MGMT_2, 0x00);//set X, Y, Z accel and gyro on
////    delay(100);
////
////    //Configure device for bias calculation
////    SPI0_write(INT_ENABLE, 0x00);//Disable all interrupts
////    SPI0_write(FIFO_EN, 0x00);//Disable FIFO
////    SPI0_write(PWR_MGMT_1, 0x00);//Turn on internal clock source
////    SPI0_write(I2C_MST_CTRL, 0x00);//Disable I2C master
////    SPI0_write(USER_CTRL, 0x00);//Disable FIFO access from serial interface(SPI)
////    SPI0_write(USER_CTRL, 0x07);//Reset FIFO and DMP
////    delay(15);
////
////    //Configure gyro and accel for bias calculation
////    SPI0_write(CONFIG, 0x01);//Set low-pass filter to 184Hz
////    SPI0_write(SMPLRT_DIV, 0x00);//Set Sample rate to 1KHz
////    SPI0_write(GYRO_CONFIG, 0x00);//Set Gyro full-scale to 250 degrees per second, maximum sensitivity
////    SPI0_write(ACCEL_CONFIG, 0x00);//Set Accelerometer full-scale to 2g, maximum sensitivity
//
//    //Configure FIFO to capture accelerometer and gyro data for bias calculation
//    SPI0_write(USER_CTRL, 0x40);//Set FIFO enable
//    SPI0_write(FIFO_EN, 0x78);//Enable gyro and accelerometer sensors for FIFO (max size 512bytes in MPU-9250)
//    delay(40);// accumulate 40 samples in 40 milliseconds = 480 bytes
//
//    //Turn off FIFO sensor read
//    SPI0_write(FIFO_EN, 0x00);// Disable gyro and accelermeter for FIFO
//    SPI0_readbytes(FIFO_COUNT_H, 2, &data[0]);//read FIFO sample count
//
//    fifo_count = ((uint16_t) data[0]<<8) | data[1];//how many bytes in fifo
//    packet_count = fifo_count/12;//How many sets of full gyro and accelerometer data
//
//    for(uint16_t i=0; i<packet_count; i++){
//        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//        SPI0_readbytes(FIFO_R_W, 12, &data[0]);//read data set for averaging
//
//        for(uint8_t j=0; j<3; j++){
//            accel_temp[j] = (int16_t) (((int16_t) data[2*j]<<8) | data[2*j+1]);
//            gyro_temp[j] = (int16_t) (((int16_t) data[2*j+6]<<8) | data[2*j+7]);
//        }//Form signed 16-bit integer for each sample in FIFO
//        
//        for(uint8_t j=0; j<3; j++){
//            accel_bias[j] += (int32_t) accel_temp[j];
//            gyro_bias[j] += (int32_t) gyro_temp[j];
//        }//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//    }
//    
//    for(uint8_t i=0; i<3; i++){
//        accel_bias[i] /= (int32_t) packet_count;
//        gyro_bias[i] /= (int32_t) packet_count;
//    }//Normalize sums to get average count biases
//
//    if(accel_bias[2]>0L){accel_bias[2] -= (int32_t) 1/ACCEL_DIV;}
//    else{accel_bias[2] += (int32_t) 1/ACCEL_DIV;}//Remove gravity from the z-axis accelerometer bias calculation
//    //Sensor must be kept flat with ground
//
//    //Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//    for(uint8_t i=0; i<3; i++){//Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//        data[2*i] = (-gyro_bias[i]/4 >> 8) & 0xFF;//Biases are additive, so change sign on calculated average gyro biases
//        data[2*i+1] = (-gyro_bias[i]/4) & 0xFF; 
//    }
//
//    //Push gyro biases to hardware registers
////    SPI0_write(XG_OFFSET_H, data[0]);
////    SPI0_write(XG_OFFSET_L, data[1]);
////    SPI0_write(YG_OFFSET_H, data[2]);
////    SPI0_write(YG_OFFSET_L, data[3]);
////    SPI0_write(ZG_OFFSET_H, data[4]);
////    SPI0_write(ZG_OFFSET_L, data[5]);
//
//    //Output scaled gyro biases for display in the main program
//    for(int i=0; i<3; i++){
//        gyro[i] = (float) gyro_bias[i] * GYRO_DIV; 
//    }
//
//    //Construct the accelerometer biases for push to the hardware accelerometer bias registers.
//    //factory trim values which must be added to the calculated accelerometer biases; on boot up these resisters will hold
//    // non-zero values. In addtion, bit 0 of the lower byte must be  preserved since it is used for temperature
//    // compensation calculations. accelerometer bias registers expect bias input as 2048 LSB per g, so that 
//    //the acceleromete biases calculated above must be divided by 8.
//    
//    int32_t accel_bias_reg[3] = {0, 0, 0};//A place to hold the factory accelerometer trim value
//    SPI0_readbytes(XA_OFFSET_H, 2, &data[0]);//read factory accelerometer trim value
//    accel_bias_reg[0] = (int32_t) (((int16_t) data[0]<<8) | data[1]);
//    SPI0_readbytes(YA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[1] = (int32_t) (((int16_t) data[0]<<8) | data[1]);
//    SPI0_readbytes(ZA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[2] = (int32_t) (((int16_t) data[0]<<8) | data[1]);
//
//    uint32_t mask = 1uL;//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers.
//    uint8_t mask_bit[3] = {0, 0, 0};//Define array to hold mask bit for each accelerometer bias axis.
//
//    for(int i=0; i<3; i++){
//        if((accel_bias_reg[i] & mask)) mask_bit[i] = 0x01;
//    }
//
//    //construct total accelerometer bias, including calculated average accelerometer bias from above.
//    for(int i=0; i<3; i++){
//        accel_bias_reg[i] -= (accel_bias[i]/8);//Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//    }
//
//    for(int i=0; i<3; i++){
//        data[2*i] = (accel_bias_reg[i] >> 8) & 0xFF;
//        data[2*i+1] = (accel_bias_reg[i]) & 0xFF;
//        data[2*i+1] |= mask_bit[i];//preserve temperature compensation bit when writing back to accelerometer bias registers.
//    }
//
//    //Push accelerometer biases to hardware register
////    SPI0_write(XA_OFFSET_H, data[0]);
////    SPI0_write(XA_OFFSET_L, data[1]);
////    SPI0_write(YA_OFFSET_H, data[2]);
////    SPI0_write(YA_OFFSET_L, data[3]);
////    SPI0_write(ZA_OFFSET_H, data[4]);
////    SPI0_write(ZA_OFFSET_L, data[5]);
//
//    //Output scaled accelerometer biases for display in the main program
//    for(int i=0; i<3; i++){
//        accel[i] = (float) accel_bias[0]*ACCEL_DIV;
//    }
//}   

void Calibrate(void){
  acc_bias[0] = 0;
  acc_bias[1] = 0;
  acc_bias[2] = 0;
  gyro_bias[0] = 0;
  gyro_bias[1] = 0;
  gyro_bias[2] = 0;
  uint32_t acc_data_temp[3] = {0, 0, 0};
  uint32_t gyro_data_temp[3] = {0, 0, 0};
  uint16_t fifo_count, packet_count;
  uint8_t data[12];
  
  SPI0_write(USER_CTRL, 0x40);//Set FIFO enable
  SPI0_write(FIFO_EN, 0x78);//Enable gyro and accelerometer sensors for FIFO (max size 512bytes in MPU-9250)
  delay(40);// accumulate 40 samples in 40 milliseconds = 480 bytes

  //Turn off FIFO sensor read
  SPI0_write(FIFO_EN, 0x00);// Disable gyro and accelermeter for FIFO
  SPI0_readbytes(FIFO_COUNT_H, 2, &data[0]);//read FIFO sample count

  fifo_count = ((uint16_t) data[0]<<8) | data[1];//how many bytes in fifo
  packet_count = fifo_count/12;//How many sets of full gyro and accelerometer data

  for(uint16_t i=0; i<packet_count; i++){
    SPI0_readbytes(FIFO_R_W, 12, data);
    for(uint8_t j=0; j<3; j++){
      acc_data_temp[j] += (uint32_t)data[2*j] | data[2*j+1];
      gyro_data_temp[j] += (uint32_t)data[2*j+6] | data[2*j+7];
    }
  }
  for(uint8_t j=0; j<3; j++){
    acc_data_temp[j] /= packet_count;
    gyro_data_temp[j] /= packet_count;
  }
  acc_bias[0] = acc_data_temp[0];
  acc_bias[1] = acc_data_temp[1];
  acc_bias[2] = acc_data_temp[2];
  gyro_bias[0] = gyro_data_temp[0];
  gyro_bias[1] = gyro_data_temp[1];
  gyro_bias[2] = gyro_data_temp[2];
}
//void GetSensorData(int16_t* data)


void GetAccelData(float* acceldata){
    uint8_t* temp;
    int16_t* raw;
    SPI0_readbytes(ACCEL_XOUT_H, 6, temp);
    for(int i=0; i<6; i+=2){
        raw[i] = ((int16_t)temp[i]<<8) | temp[i+1];
    }
    for(int i=0; i<2; i++){
        acceldata[i] = (float)raw[i]*ACCEL_DIV;
    }
}

void GetGyroData(float* gyrodata){
    uint8_t* temp;
    int16_t* raw;
    SPI0_readbytes(GYRO_XOUT_H, 6, temp);
    for(int i=0; i<6; i+=2){
        raw[i] = ((int16_t)temp[i]<<8) | temp[i+1];
    }
    for(int i=0; i<2; i++){
        gyrodata[i] = (float)raw[i]*GYRO_DIV;
    }
}



//void GetMagData(int16_t*);

void ComplimentaryFilter(float timeconst, float sensorperiod){
    uint8_t ACC_OUT_temp[6], GYR_OUT_temp[4];
    int16_t ACC_16bit[3], GYR_16bit[2];
    SPI0_readbytes(ACCEL_XOUT_H, 6, &ACC_OUT_temp[0]);
    SPI0_readbytes(GYRO_XOUT_H, 4, &GYR_OUT_temp[0]);

    ACC_16bit[0] = (int16_t)((int16_t)ACC_OUT_temp[0] << 8 | ACC_OUT_temp[1]) - acc_bias[0];
    ACC_16bit[1] = (int16_t)((int16_t)ACC_OUT_temp[2] << 8 | ACC_OUT_temp[3]) - acc_bias[1];
    ACC_16bit[2] = (int16_t)((int16_t)ACC_OUT_temp[4] << 8 | ACC_OUT_temp[5]) - acc_bias[2]; 
    //float angle_prev[2] = {angle[0], angle[1]};
    float acc_angle[2];
    acc_angle[0] = atan2((float)ACC_16bit[0], (float)ACC_16bit[2]) * (180 / PI); 
    acc_angle[1] = atan2((float)ACC_16bit[1], (float)ACC_16bit[2]) * (180 / PI);
    float gyr_angle_diff[2];
    
    gyr_angle_diff[0] = (float)(((int16_t)GYR_OUT_temp[0] << 8 | GYR_OUT_temp[1]) - gyro_bias[0]) * GYRO_DIV ;
    gyr_angle_diff[1] = (float)(((int16_t)GYR_OUT_temp[2] << 8 | GYR_OUT_temp[3]) - gyro_bias[1]) * GYRO_DIV ;
    float alpha = timeconst / (timeconst + sensorperiod);
    
    
    for(uint8_t i=0; i<2; i++){
      angle[i] = alpha * (angle[i] + gyr_angle_diff[i] * sensorperiod)
                  + (1 - alpha) * acc_angle[i];
    }
}
