
#include "arm_math.h"
void calib_magnet(){
	arm_status status  = ARM_MATH_TEST_FAILURE;
    int size = 9;
	float x[20];
	float y[20];
	float z[20];

	float D[200];
	float DT_[200];
    float S_[100];

    float S11_[36];
    float S12_[24];
    float S21_[24];
    float S22_[16];
    float invS22_[16];
    float S12invS22_[24];
    float S12_invS22_S21_[36];
    float sub_mat_[36];
    float E_[36];
    float C_[36] = {
    		 0   , 0.5000  ,  0.5000   ,      0    ,     0   ,      0,
    		    0.5000     ,    0   , 0.5000  ,       0 ,        0 ,        0,
    		    0.5000  ,  0.5000    ,     0   ,      0   ,      0   ,      0,
    		         0    ,     0     ,    0  , -0.2500   ,      0     ,    0,
    		         0    ,     0    ,     0      ,   0  , -0.2500      ,   0,
    		         0      ,   0      ,   0    ,     0 ,        0 ,  -0.2500
    };
       for(uint8_t i = 0; i < size; i++){

			D[i*10] = x[i]*x[i];
			D[i*10+1] = x[i]*x[i];
			D[i*10+2] = x[i]*x[i];
			D[i*10+3] = x[i]*y[i];
			D[i*10+4] = y[i]*z[i];
			D[i*10+5] = z[i]*x[i];

			D[i*10+6] = x[i]*2;
			D[i*10+7] = x[i]*2;
			D[i*10+8] = x[i]*2;
			D[i*10+9] = 1;
    	}
       arm_matrix_instance_f32 S;
       arm_matrix_instance_f32 S11;
       arm_matrix_instance_f32 S12;
       arm_matrix_instance_f32 S21;
       arm_matrix_instance_f32 S22;
       arm_matrix_instance_f32 invS22;
       arm_matrix_instance_f32 S12invS22;
       arm_matrix_instance_f32 S12_invS22_S21;
       arm_matrix_instance_f32 Dm;
       arm_matrix_instance_f32 DT;
       arm_matrix_instance_f32 C;
       arm_matrix_instance_f32 sub_mat;
       arm_matrix_instance_f32 E;

       arm_mat_init_f32(&S,  10,10, S_);
       arm_mat_init_f32(&Dm, size, 10, D);
       arm_mat_init_f32(&DT, 10, size, DT_);

       for(uint8_t i = 0; i < 6; i++){
    	   for(uint8_t j = 0; j < 6; j++){
    		   S11_[i*6 + j] = S_[i*10 + j];
    	   }
       }

       for(uint8_t i = 0; i < 6; i++){
    	   for(uint8_t j = 6; j < 10; j++){
    		   S12_[i*6 + j] = S_[i*10 + j];
    	   }
       }

       for(uint8_t i = 6; i < 10; i++){
    	   for(uint8_t j = 0; j < 6; j++){
    		   S21_[i*4 + j] = S_[i*10 + j];
    	   }
       }

       for(uint8_t i = 6; i < 10; i++){
    	   for(uint8_t j = 6; j < 10; j++){
    		   S22_[i*4 + j] = S_[i*10 + j];
    	   }
       }

       arm_mat_init_f32(&S11, 6,6, S11_);
       arm_mat_init_f32(&S12, 6,4, S12_);
       arm_mat_init_f32(&S21, 4,6, S21_);
       arm_mat_init_f32(&S22, 4,4, S22_);
       arm_mat_init_f32(&invS22, 4,4, invS22_);
       arm_mat_init_f32(&S12invS22, 6,4, S12invS22_);
       arm_mat_init_f32(&S12_invS22_S21, 6,6, S12_invS22_S21_);
       arm_mat_init_f32(&C, 4,4, C_);
       arm_mat_init_f32(&sub_mat, 6,6, sub_mat_);
       arm_mat_init_f32(&E, 6,6, E_);

       status = arm_mat_trans_f32 (&Dm, &DT);
       status = arm_mat_mult_f32 (&DT, &Dm,&S);

       //inv(C)*(S11 - S12*inv(S22)*S21)
       status = arm_mat_inverse_f32 (&S22, &invS22);
       status = arm_mat_mult_f32 (&S12, &invS22,&S12invS22);
       status = arm_mat_mult_f32 (&S12invS22, &S21,&S12_invS22_S21);
       status = arm_mat_sub_f32 (&S11, &S12_invS22_S21,&sub_mat);
       status = arm_mat_sub_f32 (&C, &sub_mat,&E);

}
