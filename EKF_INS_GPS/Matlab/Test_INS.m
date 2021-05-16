x = [0 0 0 0 0 0 0 0]
P = eye(18)*0.001
% Q = eye(9)
t = 0.01

g = [0 0 -9.81]

G_Noise = 0.015;
GB_Noise = 2e-05;
A_Noise = 0.019;
AB_Noise = 0.0001;

Q = eye(12);
Q(1:3,1:3) = eye(3)*A_Noise*t^2;
Q(4:6,4:6) = eye(3)*G_Noise*t^2;
Q(7:9,7:9) = eye(3)*AB_Noise*t;
Q(10:12,10:12) = eye(3)*GB_Noise*t;

position = [0,0,0]
velocity = [0,0,0]


orientation = Euler2Quat(0,0,pi/4);
q = [1,0,0,0]

bias_a = [0.0,0.0,0]
bias_g = [0.005,0,0]


x = [position,velocity,orientation,bias_a,bias_g,g];
dx = zeros(15,1)
% u = [0 0 0 0 1 9.81]

q_l = [[q(1) -q(2) -q(3) -q(4)]
       [q(2) q(1) -q(4) q(3)]
       [q(3) q(4) q(1) -q(2)]
       [q(4) -q(3) q(2) q(1)]];

q_r = [[q(1) -q(2) -q(3) -q(4)]
       [q(2) q(1) q(4) -q(3)]
       [q(3) -q(4) q(1) q(2)]
       [q(4) q(3) -q(2) q(1)]];
q_w = Euler2Quat(pi/4,pi/3,pi/6);
q = q_l*transpose(q_w);

u = [0.005,0.0,0,0,1,1,9.82];
i = 1;
gt = x
N = 5000
orientation = zeros(3,N);
pos_1 = zeros(3,N);
pos_2 = zeros(3,N);

fs = 1
time = (0:(N-1)).'/fs;
while i < N
  [dx, x, P,gt] = predict(gt,dx,x,t,P,Q,u);
Quat2Euler(x(7:10))*180/pi;
pos_1(:,i) = x(1:3);
x(1:3);
Quat2Euler([x(7) x(8) x(9) x(10)])*180/pi
 
 vel(:,i) = x(4:6);
  y_gps = [gt(1:3)];
  [dx,x,P] = update(dx,x,y_gps,P);
  pos_2(:,i) = x(1:3);
  x(14:16);
  x;
  i = i+1;
end
% gt

figure;
subplot(3,1,1);
hold on;grid on;
plot(time,pos_2(1,:),'b');

legend( 'estimated');
title('Roll');
% 
subplot(3,1,2);
hold on;grid on;

plot(time,pos_2(2,:),'b');
legend('estimated');
title('Pitch');

 subplot(3,1,3);
hold on;grid on;

plot(time,pos_2(3,:),'b');
legend('estimated');
title('Pitch');