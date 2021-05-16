function [e] = Quat2Euler(q)
sinr_cosp = 2 * (q(1) * q(2) + q(3) * q(4));
cosr_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
e(1) = atan2(sinr_cosp, cosr_cosp);

sinp = 2 * (q(1) * q(3) - q(4) * q(2));
    if (abs(sinp) >= 1)
%         e(2) = copysign(M_PI / 2, sinp); 
    else
        e(2) = asin(sinp);
    end
  
siny_cosp = 2 * (q(1) * q(4) + q(2) * q(3));
cosy_cosp = 1 - 2 * (q(3) * q(3) + q(4) * q(4));
e(3) = atan2(siny_cosp, cosy_cosp);