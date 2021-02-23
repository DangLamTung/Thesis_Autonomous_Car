/*
 * LPF.hpp
 *
 *  Created on: Jun 2, 2020
 *      Author: tung
 */

#ifndef INC_LPF_HPP_
#define INC_LPF_HPP_

double LPF_100HZ[6] = {1,2,1, -1.199677568202184,0.515738756176753, 0.079015296993642};
double LPF_70HZ[6] = {1,2,1, -1.453201299565920,0.622636675464978, 0.042358843974765};
double LPF_50HZ[6] = {1,2,1, -1.618519638615533,0.710593476651197, 0.023018459508916};
double LPF_20HZ[6] = {1,2,1, -1.855059392365665,0.871321382947899, 0.004065497645559};
double LPF_10HZ[6] = {1,2,1, -1.929169817354214,0.933375551589350, 0.001051433558784};
double LPF_1HZ[6] = {1,2,1, -1.985588815403997,0.985691916279302, 2.577521882616267e-05};




double LPF_10HZ_fs100_chev2[6] = {1,-1.302658194482643,1,1.987004126245633,0.987088037460222, 1.203301077400287e-04};
double LPF_10HZ_fs100[6] = {1,2,1, -1.199677568202184,0.515738756176753, 0.079015296993642};
class LPF{
	double w, w1, w2;
	double b0,b1,b2;
	double a1, a2, g;
	double y1;
public:
     LPF();
     void load(double coff[6]){
    	this->b0 = coff[0];
        this->b1 = coff[1];
    	this->b2 = coff[2];

    	this->a1 = coff[3];
    	this->a2 = coff[4];

    	this->g = coff[5];
     }
     double update(double Input){
    	 w = Input - a1*w1 - a2*w2;
    	 y1 = (w*b0 + w1*b1 + w2*b2)*g;
    	 w2 = w1;
    	 w1 = w;
    	 return y1;
     }
     double get(){
     	 return y1;
      }
};
LPF::LPF(){

}


#endif /* INC_LPF_HPP_ */
