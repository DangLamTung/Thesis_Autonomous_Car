/*
 * EKF.h
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "arm_math.h"
#include <math.h>
#include "mpu_data_type.hpp"
class EKF2 {
public:
//    Matrix x,y,A,B,P,Q,K,S,R,w;
	float32_t x_[7] = { 1, 0, 0, 0, 0, 0, 0 };
	float32_t Q_full[49]= {  0,0,0,0,0,0,0,
	        0,0,0,0,0,0, 0,
	         0,0,0,0,0,0,0,
	          0,0,0,0,0,0,0,
			  0,0, 0 ,0 ,0.001 ,0, 0,
	          0,0,0,0,0,0.001,0,
	          0,0,0,0,0,0,0.001};
	float32_t P_full[49]= {1, 0, 0, 0, 0, 0, 0,
		          0,1,0, 0, 0, 0, 0,
		          0, 0,1,0, 0, 0, 0,
		          0, 0, 0,1,0, 0, 0,
		          0, 0, 0, 0,1,0, 0,
		          0, 0, 0, 0, 0,1,0,
		          0, 0, 0, 0, 0, 0,1};
	float32_t R_full1[36]= {0.0001, 0, 0, 0, 0, 0,
						0, 0.0001, 0, 0, 0, 0,
						0, 0, 0.0001, 0, 0, 0,
						0, 0, 0, 0.0001, 0, 0,
						0, 0, 0, 0, 0.0001, 0,
						0, 0, 0, 0, 0, 0.0001};
	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 y;
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 AT;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 I;
	arm_matrix_instance_f32 w;

	arm_matrix_instance_f32 temp1;
	arm_matrix_instance_f32 temp2;
	arm_matrix_instance_f32 temp3;
	arm_matrix_instance_f32 temp4;
	arm_matrix_instance_f32 temp5;

	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 Hx;
	arm_matrix_instance_f32 Sinv;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 P_HT;
	arm_matrix_instance_f32 HPHT;

	arm_matrix_instance_f32 inovation;
	arm_matrix_instance_f32 Kino;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 IKH;
	arm_status status  = ARM_MATH_TEST_FAILURE;

	float32_t temp1_[7];
	float32_t temp2_[7];
	float32_t temp3_[49];
	float32_t temp4_[49];
	float32_t y_[6];
	float32_t A_[49];
	float32_t AT_[49];
	float32_t B_[21];
//	float32_t P_ [49];
	float32_t HT_[42];
	float32_t Kino_[49];
	float32_t IKH_[49];
	float32_t Hx_[6];
	float32_t PHT_[42];
	float32_t P_HT_[42];
	float32_t HPHT_[36];
	float32_t H_[42];
	float32_t K_ [42];
	float32_t KH_ [49];
	float32_t S_ [36];
	float32_t Sinv_ [36];
	float32_t ino_ [36];
	float32_t R_ [36];
	float32_t w_ [3];
	float32_t I_ [49] = {1, 0, 0, 0, 0, 0, 0,
	          0,1,0, 0, 0, 0, 0,
	          0, 0,1,0, 0, 0, 0,
	          0, 0, 0,1,0, 0, 0,
	          0, 0, 0, 0,1,0, 0,
	          0, 0, 0, 0, 0,1,0,
	          0, 0, 0, 0, 0, 0,1};
public:
	void loadEKF(){
		 arm_mat_init_f32(&x,  7,1, x_);
		  arm_mat_init_f32(&P, 7, 7, P_full);
		  arm_mat_init_f32(&Q, 7, 7, Q_full);
		  arm_mat_init_f32(&R, 6, 6, R_full1);
		  arm_mat_init_f32(&temp1, 7, 1, temp1_);
		  arm_mat_init_f32(&temp2, 7, 1, temp2_);
		  arm_mat_init_f32(&temp3, 7, 7, temp3_);
		  for(uint8_t i = 0; i<49;i++){
			  A_[i] = 0;
			  AT_[i] = 0;
			  B_[i] = 0;
			  Kino_[i] = 0;
			  KH_[i] = 0;
			  IKH_[i] = 0;
		  }

		  for(uint8_t i = 0; i<42;i++){
			  H_[i] = 0;
			  K_[i] = 0;
			  PHT_[i] = 0;
			  HT_[i] = 0;
		  }

		  for(uint8_t i = 0; i<36;i++){
			  HPHT_[i] = 0;
			  S_[i] = 0;
			  Sinv_[i] = 0;
		  }
	}
	EKF2();
	void updateEKF(IMU_data data_imu, MAG_data mag, float dt){
//          quaternion state ;
          float recipNorm;
          if(!isnan(data_imu.Acc_x) &&!isnan(data_imu.Acc_y)&&!isnan(data_imu.Acc_z)&&!isnan(data_imu.Gyro_x)&&!isnan(data_imu.Gyro_y)&&!isnan(data_imu.Gyro_z)){
        	  w_[0] = Gyro_x*DEC2RAD;
        	 	  w_[1] = Gyro_y*DEC2RAD;
        	 	  w_[2] = Gyro_z*DEC2RAD;

        	 	  y_[0] = data_imu.Acc_x;
        	 	  y_[1] = data_imu.Acc_y;
        	 	  y_[2] = data_imu.Acc_z;

        	 	  y_[3] = mag.Mag_x;
        	 	  y_[4] = mag.Mag_y;
        	 	  y_[5] = mag.Mag_z;


        	 	  float32_t x0,x1,x2,x3,x0_,x1_,x2_,x3_,x02,x12,x22,x32,x02_,x12_,x22_,x32_;
        	 	  x0 = x_[0]*0.5*dt;
        	 	  x1 = x_[1]*0.5*dt;
        	 	  x2 = x_[2]*0.5*dt;
        	 	  x3 = x_[3]*0.5*dt;

        	 	  x0_ = -x0;
        	 	  x1_ = -x1;
        	 	  x2_ = -x2;
        	 	  x3_ = -x3;

        	 	  A_[0] = 1;
        	 	           A_[8] = 1;
        	 	           A_[16] = 1;
        	 	           A_[24] = 1;
        	 	           A_[32] = 1;
        	 	           A_[40] = 1;
        	 	           A_[48] = 1;

        	 	           A_[4] = x1;
        	 	           A_[5] = x2;
        	 	           A_[6] = x3;

        	 	           A_[11] = x0_;
        	 	           A_[12] = x3;
        	 	           A_[13] = x2_;

        	 	           A_[18] = x3_;
        	 	           A_[19] = x0_;
        	 	           A_[20] = x1;

        	 	           A_[25] = x2;
        	 	           A_[26] = x1_;
        	 	           A_[27] = x0_;

        	 	           B_[0] = x1_;
        	 	           B_[1] = x2_;
        	 	           B_[2] = x3_;

        	 	           B_[3] = x0;
        	 	           B_[4] = x3_;
        	 	           B_[5] = x2;

        	 	           B_[6] = x3;
        	 	           B_[7] = x0;
        	 	           B_[8] = x1_;

        	 	           B_[9] = x2_;
        	 	           B_[10] = x1;
        	 	           B_[11] = x0;
        	 	  arm_mat_init_f32(&A,7,7,A_);
        	 	  arm_mat_init_f32(&B,7,3,B_);
        	 	  arm_mat_init_f32(&w,3,1,w_);
        	 	  arm_mat_init_f32(&temp2,7,1,temp2_);
        	 	  arm_mat_init_f32(&AT,7,7,AT_);
        	 	  arm_mat_init_f32(&temp4,7,7,temp4_);

        	 	  status = arm_mat_mult_f32 (&A, &x, &temp1);
        	 	  status = arm_mat_mult_f32 (&B, &w, &temp2);

        	 	  status = arm_mat_add_f32 (&temp1, &temp2,&x);
        	 	  status = arm_mat_mult_f32 (&A, &P, &temp3);

        	 	  status = arm_mat_trans_f32 (&A, &AT);
        	 	  status = arm_mat_mult_f32 (&temp3, &AT, &temp4);
        	 //
        	 	  status = arm_mat_add_f32 (&temp4, &Q, &P);

        	 	  x02 = x_[0]*(-2);
        	 	  x12 = x_[1]*(-2);
        	 	  x22 = x_[2]*(-2);
        	 	  x32 = x_[3]*(-2);

        	 	  x02_ = -x02;
        	 	  x12_ = -x12;
        	 	  x22_ = -x22;
        	 	  x32_ = -x32;

        	 	  H_[0] = x22_;
        	 	  H_[1] = x32;
        	 	  H_[2] = x02_;
        	 	  H_[3] = x12;

        	 	  H_[7] = x12;
        	 	  H_[8] = x02;
        	 	  H_[9] = x32;
        	 	  H_[10] = x22;

        	 	  H_[14] = x02;
        	 	  H_[15] = x12_;
        	 	  H_[16] = x22_;
        	 	  H_[17] = x32;


        	 	          H_[21] = x32;
        	 	          H_[22] = x22;
        	 	          H_[23] = x12;
        	 	          H_[24] = x02;

        	 	          H_[28] = x02;
        	 	          H_[29] = x12_;
        	 	          H_[30] = x22;
        	 	          H_[31] = x32_;

        	 	          H_[35] = x12_;
        	 	          H_[36] = x02_;
        	 	          H_[37] = x32;
        	 	          H_[38] = x22;

        	 	          arm_mat_init_f32(&HT,7,6,HT_);
        	 	    	  arm_mat_init_f32(&H,6,7,H_);
        	 	    	  arm_mat_init_f32(&HPHT,6,6,HPHT_);
        	 	    	  arm_mat_init_f32(&PHT,6,7,PHT_);
        	 	    	  arm_mat_init_f32(&P_HT,7,6,P_HT_);
        	 	    	  arm_mat_init_f32(&S,6,6,S_);
        	 	    	  arm_mat_init_f32(&Sinv,6,6,Sinv_);
        	 	    	  arm_mat_init_f32(&K,7,6,K_);
        	 	    	  arm_mat_init_f32(&Hx,7,6,Hx_);
        	 	    	  arm_mat_init_f32(&Kino,7,7,Kino_);
        	 	    	  arm_mat_init_f32(&inovation,6,1,ino_);
        	 	    	  arm_mat_init_f32(&KH,7,7,KH_);
        	 	    	  arm_mat_init_f32(&I,7,7,I_);
        	 	    	  arm_mat_init_f32(&IKH,7,7,IKH_);
        	 //	    	  arm_mat_init_f32(&temp2,7,1,temp2_);



        	 	    	  status = arm_mat_trans_f32 (&H, &HT);
        	 	    	  status = arm_mat_mult_f32 (&H, &P, &PHT);


        	 	    	  status = arm_mat_mult_f32 (&PHT, &HT, &HPHT);
        	 	    	  status = arm_mat_add_f32 (&HPHT, &R, &S);
        	 	    	  status = arm_mat_inverse_f32(&S,&Sinv);
        	 //
        	 	    	  status = arm_mat_mult_f32 (&P, &HT, &P_HT);
        	 	    	  status = arm_mat_mult_f32 (&P_HT, &Sinv, &K);

        	 //
        	 	    	  status = arm_mat_mult_f32 (&H, &x, &Hx);
        	 	    	  status = arm_mat_sub_f32(&y,&Hx, &inovation);
        	 	    	  status = arm_mat_sub_f32(&K,&inovation, &Kino);

        	 	    	  status = arm_mat_add_f32(&x,&Kino, &x);
        	 	    	  status = arm_mat_add_f32(&K,&H, &KH);
        	 	    	  status = arm_mat_sub_f32(&I,&KH, &IKH);
        	 	    	  status = arm_mat_mult_f32 (&IKH, &P, &P);

               	       float temp = (float) (x_[0] * x_[0]  + x_[1] * x_[1] + x_[2] * x_[2] + x_[3] * x_[3]);
               	       temp = sqrt(temp);
               	       float recipNorm;
               	    	  recipNorm = 1/(temp);
               	    	  x_[0] *= recipNorm;
               	    	  x_[1] *= recipNorm;
               	    	  x_[2] *= recipNorm;
               	    	  x_[3] *= recipNorm;

               	    	  angle = getAngle1(x_[0],x_[1],x_[2],x_[3]);

//        x.print();


//         y_.print(huart3);
//         B.print(huart3);
//     	 P.print();
//         H.print(huart3);
//         P.print(huart3);
//         S.print(huart3);
          }

	}
	quaternion getQuaternion(){
		quaternion q;
		q.q0 = x_[0];
		q.q1 = x_[1];
		q.q2 = x_[2];
		q.q3 = x_[3];
		return q;
	}
	EULER_angle getAngle1(float q0,float q1, float q2,float q3){
		float r,p,y;
		EULER_angle angle_e;


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

	EULER_angle angle;
	virtual ~EKF2();
};
EKF2::EKF2(){

}
EKF2::~EKF2() {
	// TODO Auto-generated destructor stub
	 // free(this->data);
}

#endif /* INC_EKF_H_ */
