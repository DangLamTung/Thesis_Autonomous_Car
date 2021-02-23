/*
 * mpu9250.hpp
 *
 *  Created on: Feb 9, 2020
 *      Author: tung
 */

#ifndef INC_MPU9250_HPP_
#define INC_MPU9250_HPP_

/*
 * mpu9250.h
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */



#include "mpu_data_type.hpp"
#include "LPF.hpp"

#define  MPU_ADDRESS     0xD0
#define  MAG_ADRRESS     0x18

/*MPU6050 register address*/
#define  WHO_AM_I_REG    0x75
#define  SAMPLE_RATE 0x19
#define  PWR_MGMT_1 0x6B
#define  CONFIG 0x1A
#define  GYRO_CONFIG  0x1B
#define  ACCEL_CONFIG  0x1C
#define  ACCEL_CONFIG2  0x1D
#define  INIT_ENB 0x38
#define  ACCEL_XOUT_H 0x3B
#define  GYRO_ZOUT_L 0x48


/*Magnetometer config*/
#define  USER_CTRL  0x6A
#define  INT_BYPASS 0x37
#define  CNTL1_AD   0x0A
#define  ROM_MODE   0x1F
#define  ASAX_AD    0x10
#define  MAGIC_OVERFLOW_MASK 0x8
/*MPU config */
#define  MPU_START 0x69
#define  inter 0x01
#define  gyro_con 0x18
#define  gyroXF 1
#define  sample_1khz 7
#define  lpf 0x01
#define  reg1 0x68
#define  reg2 0x69

#define  zero 0x00
#define  stop_i2c_master 0x22



uint8_t data;
uint8_t data_raw[13];


float temp,roll,pitch;
float asax,asay,asaz;

#define  PI 3.141592654
#define  RAD2DEC 57.29577951

#define mRes  1.49938949939 /*resolution for magnetometer*/
#define accel_factor 16384.0
#define gyro_factor 16.4

double bAx, bAy, bAz, bGx, bGy, bGz;

double ax_ref, ay_ref, az_ref;
double g_[3];
double g;
/*ellipsoid magnetometer calibration matrixes*/

float magnet_calib [3];

extern I2C_HandleTypeDef hi2c1;

char init_MPU();

IMU_data process_MPU();

float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z;
float Acc_x_,Acc_y_,Acc_z_,Gyro_x_,Gyro_y_,Gyro_z_,Mag_x_,Mag_y_,Mag_z_;
/*Complementary filter constant*/

volatile float com_angle_r;
volatile float com_angle_p;
volatile float com_angle_y;

volatile float gyro_angle_r;
volatile float gyro_angle_p;
volatile float gyro_angle_y;

/*Kalman filter constants*/
float t;
double x_k[4];
double P_plus_k[49];

float w,w1,w2;
float w_,w1_,w2_;
float w__,w1__,w2__;

static const double dv1[6] = { 1.0, 1.3085, 1.0, 1.0, -1.9382, 0.94 };

float A_m[9] = {
	    0.0042 ,   0.0003 ,  -0.0001,
	    0.0003 ,   0.0039 ,   0.0001,
	   -0.0001 ,   0.0001 ,   0.0041
};
float b_m[3] = {
	    1.4756,
	  154.7403,
	 -305.3349};

float b0 = 1.000;
float b1 = 1.3085;
float b2 = 1.0000;
float roll_acc;
float a1 = -1.9382;
float a2 = 0.9400;

uint8_t asa [3];
/******************************************Clear HAL_BUSY************************************************************/
typedef struct
 {
    I2C_HandleTypeDef* instance;
    uint16_t sdaPin;
    GPIO_TypeDef* sdaPort;
    uint16_t sclPin;
    GPIO_TypeDef* sclPort;
} I2C_Module_t;

I2C_Module_t i2c;

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;
    /* Wait until flag is set */
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = 0;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}

//static void I2C_ClearBusyFlagErratum(I2C_Module_t* i2c, uint32_t timeout)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    I2C_HandleTypeDef* handler = NULL;
//
//    handler = i2c->instance;
//
//    // 1. Clear PE bit.
//    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);
//
//    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
//    HAL_I2C_DeInit(handler);
//
//    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
//    GPIO_InitStructure.Pull = GPIO_NOPULL;
//
//    GPIO_InitStructure.Pin = i2c->sclPin;
//    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
//
//    GPIO_InitStructure.Pin = i2c->sdaPin;
//    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
//
//    // 3. Check SCL and SDA High level in GPIOx_IDR.
//    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);
//
//    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);
//    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);
//
//    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
//    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);
//
//    // 5. Check SDA Low level in GPIOx_IDR.
//    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET, timeout);
//
//    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
//    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);
//
//    // 7. Check SCL Low level in GPIOx_IDR.
//    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET, timeout);
//
//    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
//    HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);
//
//    // 9. Check SCL High level in GPIOx_IDR.
//    wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);
//
//    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
//    HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
//
//    // 11. Check SDA High level in GPIOx_IDR.
//    wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);
//
//    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
//    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
//
//    GPIO_InitStructure.Pin = i2c->sclPin;
//    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
//
//    GPIO_InitStructure.Pin = i2c->sdaPin;
//    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
//
//    // 13. Set SWRST bit in I2Cx_CR1 register.
//    SET_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
//    asm("nop");
//
//    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
//    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
//    asm("nop");
//
//    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
//    SET_BIT(handler->Instance->CR1, I2C_CR1_PE);
//    asm("nop");
//
//    // Call initialization function.
//    HAL_I2C_Init(handler);
//}

/******************************************End HAL_BUSY************************************************************/
char init_MPU(){
    char status = 1;
    uint8_t d[2];
    uint8_t device_address = MPU_ADDRESS;
    uint8_t magnet_address = MAG_ADRRESS;


    i2c.instance = &hi2c1;
    i2c.sdaPort =GPIOB;
    i2c.sclPort = GPIOB;
    i2c.sclPin = GPIO_PIN_8;
    i2c.sdaPin = GPIO_PIN_9;


  	char rx_data[25];
    while (HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) != HAL_OK) {
    	if(HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) == HAL_BUSY){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//			I2C_ClearBusyFlagErratum(&i2c, 10);
    	}
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
      }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);

//    HAL_UART_Transmit(&huart3,rx_data, strlen(rx_data),1000);

	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
    d[1] = 1;
	if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address , (uint8_t *)d, 2, 1000) != HAL_OK)
	{

	}

	/* Set data sample rate */

	d[0] = SAMPLE_RATE;
	d[1] = sample_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = CONFIG;
	d[1] = lpf;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = GYRO_CONFIG;
	d[1] = gyro_con;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG2;
	d[1] = 0x02;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);


	d[0] = INIT_ENB;
	d[1] = inter;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = USER_CTRL;
    d[1] = zero;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = INT_BYPASS;
	d[1] = stop_i2c_master;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	if (HAL_I2C_IsDeviceReady(&hi2c1, magnet_address, 3, 200) != HAL_OK) {
	    	strcpy( rx_data, "No Device \r \n");
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
	    	HAL_Delay(500);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	      }
	      else{
	    	    d[0] = CNTL1_AD;
	    	  	d[1] = ROM_MODE;
	    	  	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t) magnet_address,(uint8_t *)d,2,2)!=HAL_OK);

	    	  	HAL_Delay(100);


	    	  	uint8_t sensitive = ASAX_AD;

	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &sensitive, 1, 1000) != HAL_OK);
	    	      while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, (uint8_t *)asa,(uint16_t) 3, 1000) != HAL_OK);

	    	      asax = (asa[0]-128)*0.5/128+1;
	    	      asay = (asa[1]-128)*0.5/128+1;
	    	      asaz = (asa[2]-128)*0.5/128+1;

	    	      d[0] = CNTL1_AD;
	    	      d[1] = zero;
	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address,(uint8_t *)d,2,1000)!=HAL_OK);

	    	      HAL_Delay(100);

	    	      d[0] = CNTL1_AD;
	    	      d[1] = 0x16;
	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address,(uint8_t *)d,2,1000)!=HAL_OK);
	    	      HAL_Delay(100);
	      }


  return status;
}
void adding_raw(){
	  Gyro_x_ += Gyro_x;
	  Gyro_y_  += Gyro_y;
	  Gyro_z_  += Gyro_z;
	  Acc_x_  += Acc_x;
	  Acc_y_  += Acc_y;
	  Acc_z_  += Acc_z;
}

void delete_raw(){
	  Gyro_x_ = 0;
	  Gyro_y_ = 0;
	  Gyro_z_ = 0;
	  Acc_x_  = 0;
	  Acc_y_  = 0;
	  Acc_z_  = 0;

}


IMU_data process_MPU(bool EKF, bool LPF){
	IMU_data data_raw;
	uint8_t data[13];

	uint8_t reg = ACCEL_XOUT_H;
	uint8_t device_address = MPU_ADDRESS;


	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK){
//		if(HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) == HAL_BUSY){
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//			I2C_ClearBusyFlagErratum(&i2c, 10);
//	    }
//	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	}
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK){
//		if(HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) == HAL_BUSY){
//					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//					I2C_ClearBusyFlagErratum(&i2c, 10);
//			    }
//			    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	}

	Acc_x = (int16_t)(data[0] << 8 | data[1]);
	Acc_y = (int16_t)(data[2] << 8 | data[3]);
	Acc_z = (int16_t)(data[4] << 8 | data[5]);

    temp = (int16_t)(data[6] << 8 | data[7]);

    Gyro_x = (int16_t)(data[8] << 8 | data[9]);
    Gyro_y = (int16_t)(data[10] << 8 | data[11]);
    Gyro_z = (int16_t)(data[12] << 8 | data[13]);

	Acc_x= (Acc_x)/16384.0 - bAx ;
	Acc_y = (Acc_y)/16384.0 - bAy;
	Acc_z = (Acc_z)/16384.0 + bAz;
    if(!EKF){
	Gyro_x = (Gyro_x )/16.4- bGx;
	Gyro_y = (Gyro_y )/16.4- bGy;
	Gyro_z = (Gyro_z )/16.4- bGz;
    }
    else{
    	Gyro_x = (Gyro_x )/16.4;
    	Gyro_y = (Gyro_y )/16.4;
    	Gyro_z = (Gyro_z )/16.4;
    }

    if(!LPF){
		data_raw.Gyro_x = Gyro_x;
		data_raw.Gyro_y = Gyro_y;
		data_raw.Gyro_z = Gyro_z;
		data_raw.Acc_x = Acc_x;
		data_raw.Acc_y = Acc_y;
		data_raw.Acc_z = Acc_z;
    }

    return data_raw;
}

MAG_data magnet_get_raw(){
	MAG_data temp;
	uint8_t mag_data[7];

	uint8_t status;
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t magnet_address = MAG_ADRRESS;
	 reg = 0x02;
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
	    if(status == 3){
	    	reg = 0x03;
	    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, (uint8_t *)mag_data,7, 1000) != HAL_OK);
	    	if(!(mag_data[6] & MAGIC_OVERFLOW_MASK)){
	    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
	    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
	    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));
	    	}
	}
	    Mag_x = Mag_x*asax/4800.0;
	    Mag_y = Mag_y*asay/4800.0;
	    Mag_z = Mag_z*asaz/4800.0;

	    temp.Mag_x = Mag_x;
	    temp.Mag_y = Mag_y;
	    temp.Mag_z = Mag_z;
	    return temp;
}

MAG_data process_magnet(){
	MAG_data temp;
	uint8_t mag_data[7];

	uint8_t status;
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t magnet_address = MAG_ADRRESS;
	 reg = 0x02;
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
	    if(status == 3){
	    	reg = 0x03;
	    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, (uint8_t *)mag_data,7, 1000) != HAL_OK);
	//    	if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
	    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
	    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
	    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));

	            float m[3] = {Mag_x,Mag_y,Mag_z};
	        	float temp[3] ;
	            temp[0] = m[0] - b_m[0];
	            temp[1] = m[1] - b_m[1];
	            temp[2] = m[2] - b_m[2];

	            magnet_calib[0] = A_m[0]*temp[0] + A_m[1]*temp[1] +A_m[2]*temp[2];
	            magnet_calib[1] = A_m[3]*temp[0] + A_m[4]*temp[1] +A_m[5]*temp[2];
	            magnet_calib[2] = A_m[6]*temp[0] + A_m[7]*temp[1] +A_m[8]*temp[2];

	}
	    temp.Mag_x = magnet_calib[0];
	    temp.Mag_y = magnet_calib[1];
	    temp.Mag_z = magnet_calib[2];
	    return temp;
}
EULER_angle complementary_filter(IMU_data data, float dt, float alpha){
	EULER_angle temp;
	float pitch_acc, roll_acc;

	if(data.Acc_x != 0 && data.Acc_y != 0 && data.Acc_z !=0){
    roll_acc = atan2(data.Acc_y,data.Acc_z)*RAD2DEC;
    if(roll_acc<0){
    	roll_acc+=180;
    }
    else{
    	if(roll_acc!=0){
    		roll_acc-=180;
    	}
    }
    temp.pitch = com_angle_p;
    temp.roll = com_angle_r;
    pitch_acc = atan(data.Acc_x/sqrt(data.Acc_y*data.Acc_y + data.Acc_z*data.Acc_z))*RAD2DEC;

		com_angle_r = alpha*(com_angle_r + dt*data.Gyro_x) + (1-alpha)*roll_acc;
		com_angle_p = alpha*(com_angle_p + dt*data.Gyro_y) + (1-alpha)*pitch_acc;
	}
	if(!isnan(com_angle_r) & !isnan(com_angle_p)){
		temp.pitch = com_angle_p;
		temp.roll = com_angle_r;
	}
    return temp;
}

void notify_state(AHRS_con state){
    if(state.IMU == true && state.magnet == true && state.magnet == true){
    for(uint8_t i = 0; i<4; i++){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
 	HAL_Delay(100);
    }
    }
    HAL_Delay(1000);
}

AHRS_con check_AHRS(bool notify){
	AHRS_con state;
	uint8_t magnet_address = MAG_ADRRESS;
	uint8_t device_address = MPU_ADDRESS;
	uint8_t  BMP_ADDRESS  = 0xEE;
	 if (HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) != HAL_OK) {
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
	    	HAL_Delay(500);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	    	HAL_Delay(500);
	    	state.IMU = false;
	}
	if (HAL_I2C_IsDeviceReady(&hi2c1, magnet_address, 3, 200) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_Delay(500);
		state.magnet = false;
	}
	 if(HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t) BMP_ADDRESS, 3, 2) != HAL_OK) {
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
	    	HAL_Delay(500);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
	    	HAL_Delay(500);
	    	state.baro = false;
	      }
	 if(notify){
		 notify_state(state);
	 }
    return state;
}

EULER_angle quat2euler(quaternion q){
	float q0,q1,q2,q3,r,p,y;

	EULER_angle angle_e;
	q0 = q.q0;
	q1 = q.q1;
	q2 = q.q2;
	q3 = q.q3;

	float sinr = 2*(q0*q1 + q2 * q3);
	float cosr = 1 - 2*(q1*q1 + q2 * q2);
	r = atan2(sinr, cosr);


		float sinp = 2*( q0*q2 - q3*q1);
	    if (sinp >= 1)
	    	p = PI/2;
	    else{
	    	if(sinp <= -1){
	    	p = -PI/2;
	    }
	    else{
	    	p = asin(sinp);
	    }
	    }


		float siny = 2*( q0*q3 + q2*q1);
		float cosy = 1 - 2*( q1*q1 + q3*q3);
		y = atan2(siny, cosy);

		angle_e.roll = r*RAD2DEC;
		angle_e.pitch = p*RAD2DEC;
		angle_e.yaw = y*RAD2DEC;
		return angle_e;
}
IMU_calib_data calibration_IMU(){
	    IMU_calib_data data_;
    /*This function is performed when the sensor is fully stationary, we assume that MPU has been inited*/
//	    print_msg("Calibrating the sensor....\n");
		uint8_t data[13];
		uint8_t reg = ACCEL_XOUT_H;
		uint8_t device_address = MPU_ADDRESS;

        for(int i = 0; i<200; i++){

		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK);

		Acc_x = (int16_t)(data[0] << 8 | data[1])/accel_factor;
		Acc_y = (int16_t)(data[2] << 8 | data[3])/accel_factor;
		Acc_z = (int16_t)(data[4] << 8 | data[5])/accel_factor;

	    temp = (int16_t)(data[6] << 8 | data[7]);

	    Gyro_x = (int16_t)(data[8] << 8 | data[9])/gyro_factor;
	    Gyro_y = (int16_t)(data[10] << 8 | data[11])/gyro_factor;
	    Gyro_z = (int16_t)(data[12] << 8 | data[13])/gyro_factor;

	    bAx += Acc_x;
	    bAy += Acc_y;
	    bAz += Acc_z;

	    bGx += Gyro_x;
	    bGy += Gyro_y;
	    bGz += Gyro_z;
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	    HAL_Delay(30);
        }

     bAx /= 200;
     bAy /= 200;
     bAz /= 200;
     g = sqrt(bAx*bAx + bAy*bAy + bAz*bAz);
     bAz = -1 - bAz;
     bGx /= 200;
     bGy /= 200;
     bGz /= 200;

     data_.bAx = bAx;
     data_.bAy = bAy;
     data_.bAz = bAz;

     data_.bGx = bGx;
     data_.bGy = bGy;
     data_.bGz = bGz;

     data_.data[0] = bAx;
     data_.data[1] = bAy;
     data_.data[2] = bAz;

     data_.data[3] = bGx;
     data_.data[4] = bGy;
     data_.data[5] = bGz;

	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
	 return data_;
}

//void calibration_magnet(){
//	reg = 0x02;
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
//	if(status == 3){
//	reg = 0x03;
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &mag_data,7, 1000) != HAL_OK);
//	//    	if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
//	    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
//	    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
//	    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));
//}
//}




#endif /* INC_MPU9250_HPP_ */
