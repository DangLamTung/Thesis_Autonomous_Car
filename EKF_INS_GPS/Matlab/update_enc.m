  function [dx,x,P] = update_enc(dx,x,y_enc,P)
          % 
        H_r = [zeros(3) eye(3) zeros(3,13)];
            q0 = x(7);
            q1 = x(8);
            q2 = x(9);
            q3 = x(10);
            R = [q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2); ...
                 2*(q1*q2 + q0*q3), q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1); ...
                 2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 + q3^2];
        Q_r = 0.5*[[-q1 -q2 -q3];
               [q0 q3 -q2];
               [-q3 q0 q1];
               [q2 -q1 q0]];
        X_r = [[eye(6) zeros(6,3) zeros(6,9)];
               [zeros(4,6) Q_r zeros(4,9)];
               [zeros(9,6)  eye(9,12)]];
          H = H_r*X_r;
          K = P*transpose(H)/(H*P*transpose(H) +eye(3)*0.005);
          s = y_enc - transpose(H_r*transpose(x));
          dx = K*transpose(s);%????
          P = (eye(18) - K*H)*P;
          
          x(1:3) = x(1:3) + transpose(dx(1:3));
          x(4:6) = x(4:6) + transpose(dx(4:6));
          
          rotvec = Euler2Quat(dx(7),dx(8),dx(9));
          x(7:10) = QuatMult(rotvec,x(7:10));
          norm = sqrt(x(7)^2 + x(8)^2 + x(9)^2 + x(10)^2);
          x(7) = x(7)/norm;
          x(8) = x(8)/norm;
          x(9) = x(9)/norm;
          x(10) = x(10)/norm;
          x(11:16) = x(11:16) + transpose(dx(10:15));
          
          G = eye(18);
          G(7:9,7:9) = eye(3) - 0.5*Skew_Mat(dx(7:9));
          P = G*P*transpose(G);
          
          
          dx = zeros(15,1);
          
      end