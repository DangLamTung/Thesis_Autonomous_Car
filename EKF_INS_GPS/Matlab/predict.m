function [dx, x, P,gt] = predict(gt,dx,x,t,P,Q,u)

            g = [0 0 9.81];
            q0 = x(7);
            q1 = x(8);
            q2 = x(9);
            q3 = x(10);
            R = [q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2); ...
                 2*(q1*q2 + q0*q3), q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1); ...
                 2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 + q3^2];
             % Rotation matrix
             %update nominal state
            R*transpose( u(4:6) - x(11:13) );
            x(1:3) = x(1:3) + x(4:6)*t  + transpose(0.5*R*transpose(u(4:6) - x(11:13)+ g )*t^2);
            x(4:6) = x(4:6) + transpose(R*transpose((u(4:6)- x(11:13) + g))*t);

            % s_trans = [[-x(2 + 6),-x(3+6),-x(4+6)];
            %            [x(1+6),-x(4+6),x(3+6)];
            %            [x(4+6), x(1+6), -x(2+6)];
            %            [-x(3+6), x(2+6),x(1+6)]];
            %        
            % x(7:10) = x(7:10) + transpose(s_trans*0.5*t*transpose(u));
            rotvec = Euler2Quat((u(1)-x(14))*t,(u(2)-x(15))*t,(u(3)-x(16))*t);
            x(7:10) = QuatMult(x(7:10),rotvec);


            
            gt(1:3) = gt(1:3) + gt(4:6)*t  + transpose(0.5*R*transpose(u(4:6) + g )*t^2);
            gt(4:6) = gt(4:6) + transpose(R*transpose((u(4:6) + g))*t);
            gt(7:10) = QuatMult(gt(7:10),rotvec);
            
            
            %error state estimation matrix
            F_x = [[eye(3) eye(3)*t zeros(3)     zeros(3)        zeros(3)            zeros(3)];
                   [zeros(3) eye(3) -R*Skew_Mat(u(4:6) - x(11:13))*t -R*t zeros(3)  eye(3)*t ];
                  [zeros(3) zeros(3) eye(3) zeros(3) -R*t  zeros(3)  ];
                  [zeros(3) zeros(3) zeros(3) eye(3) zeros(3) zeros(3) ]
                  [zeros(3) zeros(3) zeros(3) zeros(3) eye(3) zeros(3)]  
                  [zeros(3) zeros(3) zeros(3) zeros(3) zeros(3) eye(3)] ];

            F_i = [[zeros(3,12)];
                    [eye(12)];
                    [zeros(3,12)]];
            %estimate error state this step is somewhat not needed
%             dx = F_x*dx;
            %update the predicted covariance matrix P 
            P = F_x*P*transpose(F_x) + F_i*Q*transpose(F_i);
            
       norm = sqrt(x(7)^2 + x(8)^2 + x(9)^2 + x(10)^2);
       x(7) = x(7)/norm;
       x(8) = x(8)/norm;
       x(9) = x(9)/norm;
       x(10) = x(10)/norm;
       
      end