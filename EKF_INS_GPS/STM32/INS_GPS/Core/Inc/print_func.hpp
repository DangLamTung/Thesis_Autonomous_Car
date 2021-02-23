/*
 * print_func.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_PRINT_FUNC_HPP_
#define INC_PRINT_FUNC_HPP_
#include "mpu_data_type.hpp"
//#include "PID.h"
void float2Bytes(float val,uint8_t * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
float bytes2Float(uint8_t * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  for(uint8_t i = 0; i<4; i++){
	  u.temp_array[i] = bytes_array[i];
  }
  return   u.float_variable;
}

void print_raw(IMU_data data_raw){
	        char buffer[10];
            ftoa(data_raw.Gyro_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_z, buffer, 2);
            strcat(buffer,"\n");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////
//            ftoa(data_raw.Mag_x, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_y, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_z, buffer, 2);
//         	strcat(buffer,"\n");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}

void print_euler(EULER_angle data){
	        char buffer[10];

            ftoa(data.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.pitch, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.yaw, buffer, 2);
            strcat(buffer,"\n");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////
//            ftoa(data_raw.Mag_x, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_y, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_z, buffer, 2);
//         	strcat(buffer,"\n");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}
void print_raw_mag(IMU_data data_raw, MAG_data data){
	        char buffer[10];
            ftoa(data_raw.Gyro_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
//////
            ftoa(data.Mag_x, buffer, 2);
         	strcat(buffer," ");
         	HAL_UART_Transmit(&huart3,(uint8_t*)buffer, strlen(buffer),1000);

         	ftoa(data.Mag_y, buffer, 2);
         	strcat(buffer," ");
         	HAL_UART_Transmit(&huart3,(uint8_t*)buffer, strlen(buffer),1000);

         	ftoa(data.Mag_z, buffer, 2);
         	strcat(buffer,"\n");
         	HAL_UART_Transmit(&huart3,(uint8_t*)buffer, strlen(buffer),1000);

}


void print_magnet(MAG_data data_raw){
	char buffer[10];
    ftoa(data_raw.Mag_x, buffer, 2);
 	strcat(buffer," ");
 	HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

 	ftoa(data_raw.Mag_y, buffer, 2);
 	strcat(buffer," ");
 	HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

 	ftoa(data_raw.Mag_z, buffer, 2);
 	strcat(buffer,"\n");
 	HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
}
void print_every_thing(IMU_data data_raw,EULER_angle data){
    char buffer[10];
    ftoa(data_raw.Gyro_x, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Gyro_y, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Gyro_z, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_x, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_y, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_z, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

	print_euler(data);
}
void print_msg(char*msg){
	HAL_UART_Transmit(&huart3,(uint8_t*) msg, strlen(msg),1000);
}

void print_euler_compare(EULER_angle data, EULER_angle data1,EULER_angle data2){
	        char buffer[10];

            ftoa(data.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.pitch, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.yaw, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data1.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data1.pitch, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data1.yaw, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.roll, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.pitch, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.yaw, buffer, 2);
             strcat(buffer,"\n");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////


}
void print_pid_value(PID_value pid){
	        char buffer[10];

            ftoa(pid.Kp1, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(pid.Ki1, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(pid.Kd1, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(pid.Kp2, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(pid.Ki2, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Kd2, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Kp3, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Ki3, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Kd3, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Kp4, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Ki4, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(pid.Kd4, buffer, 2);
             strcat(buffer,"\n");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////


}
void cpstr(uint8_t* dst, char src[4], uint8_t start){
//   uint8_t len = strlen(src);
   uint8_t j =0;
//   if(len + start > strlen(dst))return;
   for(uint8_t i = start; i <start + 4; i++){
	   dst[i] = src[j];
	   j++;
   }
}

uint32_t sum_array(char temp[4]){
	uint32_t sum = 0;
//   if(len + start > strlen(dst))return;
   for(uint8_t i = 0; i < 4; i++){
       sum += temp[i];
   }
   return sum;
}
//void advance_print_general(float* data, int length){
//	uint8_t buffer[4*length + 3];
//	uint8_t temp[4];
//	uint32_t temp_crc = 0;
//	uint8_t crc = 0;
//    uint8_t i = 0;
//    for(int count = 0; count < length; count++){
//		float2Bytes(data[count],temp);
//
//		cpstr(buffer, temp, i);
//		i += 4;
//		if(i > 12){
//		    temp_crc += temp[3];
//		}
//    }
//
//    crc = temp_crc % 37;
//    buffer[24] = crc;
//    buffer[25] = '\n';
//
//    crc = 0;
//	HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
//}

//void advance_print(IMU_data data){
//
//	float data_buf[6];
//	data_buf[0] = data.Acc_x;
//	data_buf[1] = data.Acc_y;
//	data_buf[2] = data.Acc_z;
//
//	data_buf[3] = data.Gyro_x;
//	data_buf[4] = data.Gyro_y;
//	data_buf[5] = data.Gyro_z;
//
//	advance_print_general(data_buf, 6);
//}
//
//void print_raw(IMU_data data, MAG_data data_m){
//
//	float data_buf[9];
//	data_buf[0] = data.Acc_x;
//	data_buf[1] = data.Acc_y;
//	data_buf[2] = data.Acc_z;
//
//	data_buf[3] = data.Gyro_x;
//	data_buf[4] = data.Gyro_y;
//	data_buf[5] = data.Gyro_z;
//
////	data_buf[6] = data.Mag_x;
////	data_buf[7] = data.Mag_y;
////	data_buf[8] = data.Mag_z;
//
//	advance_print_general(data_buf, 9);
//}
#endif /* INC_PRINT_FUNC_HPP_ */
