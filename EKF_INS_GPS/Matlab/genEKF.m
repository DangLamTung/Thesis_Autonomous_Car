syms x0 x1 x2 x3 x4 x5 x6 Gx Gy Gz Ax Ay Az Mx My Mz t
syms Mn Me Md Rax Ray Raz Rmx Rmy Rmz


x_pre = [x0;x1;x2;x3;x4;x5;x6]
w = [ Gx, Gy, Gz];
a = [Ax, Ay, Az];
y_full =  [Ax, Ay, Az, Mx,My,Mz];
nStates = 7


     
%       y = [  mag(k,1),mag(k,2), mag(k,3)];
s_trans = [[-x1,-x2,-x3];
           [x0,-x3,x2];
           [x3, x0, -x1];
           [-x2, x1,x0]];
s_up = -s_trans*0.5*t;
     
%      
 A = [[eye(4), s_up];
     [zeros(3,4), eye(3)]];
% 
 B = [s_trans*0.5*t; zeros(3)];
% %predict route
% 
 x = A*x_pre + B*transpose(w);
%  
for rowIndex = 1:nStates
    for colIndex = 1:nStates
        eval(['syms OP_l_',num2str(rowIndex),'_c_',num2str(colIndex), '_r_ real']);
        eval(['P(',num2str(rowIndex),',',num2str(colIndex), ') = OP_l_',num2str(rowIndex),'_c_',num2str(colIndex),'_r_;']);
    end
end
% 
 PP = A*P*transpose(A) ; 
[PP,SPP]=OptimiseAlgebra(PP,'SPP');

g = [0; 0; 1]
m = [0; -1; 0]

Tbn = Quat2Tbn([x1,x2,x3,x4]);

a_mea = Tbn*g
H_a = jacobian(a_mea,x_pre) % measurement Jacobian
[H_a,Ha_b]=OptimiseAlgebra(H_a,'Ha_b');

H_ax = H_a(1,:)
H_ay = H_a(2,:)
H_az = H_a(3,:)

h_mea = Tbn*m
H_m = jacobian(h_mea,x_pre) % measurement Jacobian
[H_m,Hm_b]=OptimiseAlgebra(H_m,'Hm_b');
% Ca = [[-x3 x4 -x1 x2];
%       [x2 x1 x4 x3];
%       [x1 -x2 -x3 x4]];
% Cm = [[x4 x3 x2 x1];
%      [x1 -x2 x3 -x4];
%      [-x2 -x1 x4 x3]];
%  H_acc =[-2.*Ca zeros(3)];
      
% H_mag = [-2.*Cm zeros(3)]
K_ax = P*transpose(H_a(1,:))/(H_a(1,:)*P*transpose(H_a(1,:)) + Rax )
K_ay = P*transpose(H_a(2,:))/(H_a(2,:)*P*transpose(H_a(2,:)) + Ray)
K_az = P*transpose(H_a(3,:))/(H_a(3,:)*P*transpose(H_a(3,:)) + Raz)

K_mx = P*transpose(H_m(1,:))/(H_m(1,:)*P*transpose(H_m(1,:)) + Rmx )
K_my = P*transpose(H_m(2,:))/(H_m(2,:)*P*transpose(H_m(2,:)) + Rmy)
K_mz = P*transpose(H_m(3,:))/(H_m(3,:)*P*transpose(H_m(3,:)) + Rmz)

H  = [H_a;H_m];
% clear    H_a H_m
K= [K_ax,K_ay,K_az,K_mx,K_my,K_mz];
% clear   K_ax K_ay K_az K_mx K_my K_mz
[K,SK]=OptimiseAlgebra(K,'SK');

nStates = length(P);
fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
save(fileName);
% SaveScriptCode(nStates);
saveEKF(7)
ConvertToM(nStates);
ConvertToC(nStates);
% 
% % 
% magMeas = transpose(Tbn)*[Mx;My;Mz]; % predicted measurement
% H_MAG = jacobian(magMeas,x_pre) % measurement Jacobian

% H_VP  = [H_VN;H_VE;H_VD;H_PN;H_PE;H_PD];
% clear    H_VN H_VE H_VD H_PN H_PE H_PD
% K_VP = [K_VN,K_VE,K_VD,K_PN,K_PE,K_PD];
% clear   K_VN K_VE K_VD K_PN K_PE K_PD
% [K_VP,SK_VP]=OptimiseAlgebra(K_VP,'SK_VP');
% [H_MAG,SH_MAG]=OptimiseAlgebra(H_MAG,'SH_MAG');

% K_MX = (P*transpose(H_MAG))/(H_MAG*P*transpose(H_MAG)); % Kalman gain vector
% [K_MX,SK_MX]=OptimiseAlgebra(K_MX,'SK_MX');
%       [K,SK_VP]=OptimiseAlgebra(K,'SK_VP');
% %       y;

%     m = [ transpose(y) H*x];
%      x = x_pre + K_ax*s_ax;
%      x = x_pre + K_ay*s_ay;
%      x = x_pre + K_az*s_az;
% x
     
%      P = (eye(7) - K_acc*H_acc)*P ;
%        norm = sqrt(x(1)^2 + x(2)^2 + x(3)^2 + x(4)^2);
%        x(1) = x(1)/norm;
%        x(2) = x(2)/norm;
%        x(3) = x(3)/norm;
%        x(4) = x(4)/norm;