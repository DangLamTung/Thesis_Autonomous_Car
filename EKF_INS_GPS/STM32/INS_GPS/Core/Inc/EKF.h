/*
 * EKF.h
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_
#include "Matrix.hpp"
//#include "MatrixBig.hpp"
#include <math.h>
#include "mpu_data_type.hpp"
class EKF {
private:
//    Matrix x,y,A,B,P,Q,K,S,R,w;
	Matrix x = Matrix(7,1);
	Matrix y = Matrix(6,1);
	Matrix A = Matrix(7,7);
	Matrix B = Matrix(7,3);
	Matrix P = Matrix(7,7);
	Matrix Q = Matrix(7,7);
	Matrix H = Matrix(6,7);
	Matrix K = Matrix(7,6);
	Matrix S = Matrix(6,6);
	Matrix R = Matrix(6,6);
	Matrix I = diag_mat(7, 7);
	Matrix w = Matrix(3,1);
public:
	void loadEKF(double* x_, double *P_, double* Q_, double*R_){
	      this->x.get_value(x_);
	      this->P.get_value(P_);
	      this->Q.get_value(Q_);
	      this->R.get_value(R_);
	}
	EKF();
	void updateEKF(IMU_data data_imu, MAG_data mag, float dt){
//          quaternion state ;
          float recipNorm;
          if(!isnan(data_imu.Acc_x) &&!isnan(data_imu.Acc_y)&&!isnan(data_imu.Acc_z)&&!isnan(data_imu.Gyro_x)&&!isnan(data_imu.Gyro_y)&&!isnan(data_imu.Gyro_z)){
          w.data[0] = data_imu.Gyro_x*DEC2RAD;
          w.data[1] = data_imu.Gyro_y*DEC2RAD;
          w.data[2] = data_imu.Gyro_z*DEC2RAD;

          y.data[0] = data_imu.Acc_x;
          y.data[1] = data_imu.Acc_y;
          y.data[2] = data_imu.Acc_z;

          y.data[3] = mag.Mag_x;
          y.data[4] = mag.Mag_y;
          y.data[5] = mag.Mag_z;

          double x0,x1,x2,x3,x0_,x1_,x2_,x3_,x02,x12,x22,x32,x02_,x12_,x22_,x32_;
          x0 = this->x.data[0]*0.5*dt;
          x1 = this->x.data[1]*0.5*dt;
          x2 = this->x.data[2]*0.5*dt;
          x3 = this->x.data[3]*0.5*dt;

          x0_ = -x0;
          x1_ = -x1;
          x2_ = -x2;
          x3_ = -x3;


         A.data[0] = 1;
         A.data[8] = 1;
         A.data[16] = 1;
         A.data[24] = 1;
         A.data[32] = 1;
         A.data[40] = 1;
         A.data[48] = 1;

         A.data[4] = x1;
         A.data[5] = x2;
         A.data[6] = x3;

         A.data[11] = x0_;
         A.data[12] = x3;
         A.data[13] = x2_;

         A.data[18] = x3_;
         A.data[19] = x0_;
         A.data[20] = x1;

         A.data[25] = x2;
         A.data[26] = x1_;
         A.data[27] = x0_;

         B.data[0] = x1_;
         B.data[1] = x2_;
         B.data[2] = x3_;

         B.data[3] = x0;
         B.data[4] = x3_;
         B.data[5] = x2;

         B.data[6] = x3;
         B.data[7] = x0;
         B.data[8] = x1_;

         B.data[9] = x2_;
         B.data[10] = x1;
         B.data[11] = x0;



         Matrix re1 = mul_mat(A,x);
         Matrix re2 = mul_mat(B,w);
         x = add_mat(re1,re2);
//         x.print();
         Matrix temp1 = mul_mat(A,P);
         Matrix temp2 = mul_mat(temp1, transpose(A));
         P = add_mat(temp2,Q);
//         P.print();

         x02 = this->x.data[0]*(-2);
         x12 = this->x.data[1]*(-2);
         x22 = this->x.data[2]*(-2);
         x32 = this->x.data[3]*(-2);

         x02_ = -x02;
         x12_ = -x12;
         x22_ = -x22;
         x32_ = -x32;

         H.data[0] = x22_;
         H.data[1] = x32;
         H.data[2] = x02_;
         H.data[3] = x12;

         H.data[7] = x12;
         H.data[8] = x02;
         H.data[9] = x32;
         H.data[10] = x22;

         H.data[14] = x02;
         H.data[15] = x12_;
         H.data[16] = x22_;
         H.data[17] = x32;


         H.data[21] = x32;
         H.data[22] = x22;
         H.data[23] = x12;
         H.data[24] = x02;

         H.data[28] = x02;
         H.data[29] = x12_;
         H.data[30] = x22;
         H.data[31] = x32_;

         H.data[35] = x12_;
         H.data[36] = x02_;
         H.data[37] = x32;
         H.data[38] = x22;
//         A.print();
//         P.print();
//         H.print();

//         Matrix temp3 = ;
         Matrix temp4 = mul_mat(mul_mat(H,P),transpose(H));


//         Matrix x = transpose(H);
//         x.print();

         Matrix S_ = add_mat(temp4,R);

         S = inverse(S_);

         if(S.inv){
//        	 S.print();
         K = mul_mat(mul_mat(P,transpose(H)),S);

         Matrix inovation = sub_mat(y,mul_mat(H,x));
//         inovation.print();
         Matrix l = mul_mat(K,inovation);
//         l.print(huart3);
         x = add_mat(x,mul_mat(K,inovation));
         Matrix c = mul_mat(K,H);
//         c.print();
         P = mul_mat(sub_mat(I,mul_mat(K,H)),P);
//         mul_mat(B,w);
//         P.print();
        float temp = (float) (x.data[0] * x.data[0]  + x.data[1] * x.data[1] + x.data[2] * x.data[2] + x.data[3] * x.data[3]);
        temp = sqrt(temp);
     	recipNorm = 1/(temp);
     	x.data[0] *= recipNorm;
     	x.data[1] *= recipNorm;
     	x.data[2] *= recipNorm;
     	x.data[3] *= recipNorm;
//        x.print();
         }
         else{

         }
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
		q.q0 = x.data[0];
		q.q1 = x.data[1];
		q.q2 = x.data[2];
		q.q3 = x.data[3];
		return q;
	}
	EULER_angle getAngle(){
		float q0,q1,q2,q3,r,p,y;
		EULER_angle angle_e;
		q0 = x.data[0];
		q1 = x.data[1];
		q2 = x.data[2];
		q3 = x.data[3];

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
	virtual ~EKF();
};
EKF::EKF(){

}
EKF::~EKF() {
	// TODO Auto-generated destructor stub
	 // free(this->data);
}

#endif /* INC_EKF_H_ */
