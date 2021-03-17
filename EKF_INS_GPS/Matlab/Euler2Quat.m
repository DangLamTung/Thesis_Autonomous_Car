function [q] = Euler2Quat(roll,pitch,yaw)
cy = cos(yaw * 0.5);
sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5);
sp = sin(pitch * 0.5);
cr = cos(roll * 0.5);
sr = sin(roll * 0.5);


q(1) = cr * cp * cy + sr * sp * sy;
q(2) = sr * cp * cy - cr * sp * sy;
q(3) = cr * sp * cy + sr * cp * sy;
q(4) = cr * cp * sy - sr * sp * cy;   