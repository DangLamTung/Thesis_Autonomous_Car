%% Create Ground Vehicle Trajectory
% The |waypointTrajectory| object calculates pose based on specified
% sampling rate, waypoints, times of arrival, and orientation. Specify the
% parameters of a circular trajectory for the ground vehicle.
imuFs = 100;
gpsFs = 1;
% Trajectory parameters
r = 8.42; % (m)
speed = 2.50; % (m/s)
center = [0, 0]; % (m)
initialYaw = 90; % (degrees)
numRevs = 2;

% Define angles theta and corresponding times of arrival t.
revTime = 2*pi*r / speed;
theta = (0:pi/2:2*pi*numRevs).';
t = linspace(0, revTime*numRevs, numel(theta)).';

% Define position.2
x = 3*r .* cos(theta) + center(1);
y = r .* sin(theta) + center(2);
z = zeros(size(x));
position = [x, y, z];

% Define orientation.
yaw = theta + deg2rad(initialYaw);
yaw = mod(yaw, 2*pi);
pitch = zeros(size(yaw));
roll = zeros(size(yaw));
orientation = quaternion([yaw, pitch, roll], 'euler', ...
    'ZYX', 'frame');
% Generate trajectory.
groundTruth = waypointTrajectory('SampleRate', imuFs, ...
    'Waypoints', position, ...
    'TimeOfArrival', t, ...
    'Orientation', orientation);

% Initialize the random number generator used to simulate sensor noise.
rng('default');

totalSimTime = 30; % seconds

% Log data for final metric computation.
numsamples = floor(min(t(end), totalSimTime) * gpsFs);
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

gpsOrientation = zeros(numsamples,3);
gpsOrientation_gt = zeros(numsamples,3);
est_pos = zeros(numsamples,3);
idx = 0;

imuSamplesPerGPS = (imuFs/gpsFs);
localOrigin = [42.2825 -71.343 53.0352];
%% IMU Sensors
% Typically, ground vehicles use a 6-axis IMU sensor for pose estimation. 
% To model an IMU sensor, define an IMU sensor model containing an 
% accelerometer and gyroscope. In a real-world application, the two sensors 
% could come from a single integrated circuit or separate ones. The 
% property values set here are typical for low-cost MEMS sensors.

imu = imuSensor('accel-gyro', ...
    'ReferenceFrame', 'ENU', 'SampleRate', imuFs);

% % Accelerometer
% imu.Accelerometer.MeasurementRange =  19.6133;
% imu.Accelerometer.Resolution = 0.0023928;
% imu.Accelerometer.NoiseDensity = 0.0012356;
% 
% % Gyroscope
% imu.Gyroscope.MeasurementRange = deg2rad(250);
% imu.Gyroscope.Resolution = deg2rad(0.0625);
% imu.Gyroscope.NoiseDensity = deg2rad(0.025);


gps = gpsSensor('UpdateRate', gpsFs, 'ReferenceFrame', 'ENU');
gps.ReferenceLocation = localOrigin;
gps.DecayFactor = 0.5;                % Random walk noise parameter 
gps.HorizontalPositionAccuracy = 1.0;   
gps.VerticalPositionAccuracy =  1.0;
gps.VelocityAccuracy = 0.1;

a_v = 0.3;
b_v = 0;


a = 0.1;
b = 0;
y = a.*randn(1000,1) + b;

idx_gps = 1

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


velocity = [0,0,0];


orientation = Euler2Quat(0,0,pi/2);
q = [1,0,0,0]

bias_a = [0.0,0.0,0]
bias_g = [0.000,0,0]

[position, orientation1, velocity, trueAcc, trueAngVel] = groundTruth();

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
gt = x
pos_accel = [0 0 0];
vel_accel = [0 0 0];
g = [0 0 9.81];







%*****************************************************************************


% hold on
for sampleIdx = 1:numsamples
    % Predict loop at IMU update frequency.
    for i = 1:imuSamplesPerGPS
        if ~isDone(groundTruth)
            idx = idx + 1;
            
            % Simulate the IMU data from the current pose.
            [truePosition(idx,:), trueOrientation(idx,:), ...
                trueVel, trueAcc, trueAngVel] = groundTruth();
            
            [accelData, gyroData] = imu(trueAcc, trueAngVel, ...
                trueOrientation(idx,:));
            u = [ gyroData(1),gyroData(2),gyroData(3),accelData];
            [dx, x, P,gt] = predict(gt,dx,x,t,P,Q,u);
            [dx,x,P] = update_enc(dx,x,trueVel + 1.*randn(1,3) + b_v,P);
            est_pos(idx,:) = x(1:3);
%              est_pos(idx,:) = pos_accel;
        end
    end
    if ~isDone(groundTruth)
        % This next step happens at the GPS sample rate.
        % Simulate the GPS output based on the current pose.
        gpsOrientation_gt(idx_gps,:) = truePosition(idx,:);
        gpsOrientation(idx_gps,:) = truePosition(idx,:) + a.*randn(1,3) + b;
        [dx,x,P] = update(dx,x,gpsOrientation(idx_gps,:),P);
        idx_gps = idx_gps+1;
        % Update the filter states based on the GPS data.
%         fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
    end
end
% 
% 
figure(1)
xlim([-15,15])
ylim([-15,15])
hold on
% plot(truePosition(:,1),truePosition(:,2))
i = 1
j = 1
k = 1
for i = 1:3000
  
     plot(truePosition(1:i,1),truePosition(1:i,2),'-r')
     xlim([-30,30])
     ylim([-30,30])
     hold on
     
     
     plot(est_pos(1:i,1),est_pos(1:i,2))
     
    if j == 100
       scatter(gpsOrientation(1:k,1),gpsOrientation(1:k,2))
       hold on 
       k = k+1
       j = 0
    end
     drawnow
     i = i+1;
     j = j+1;
     xlim([-30,30])
     ylim([-30,30])
     drawnow
%     
%      scatter(gpsOrientation(1:i,1),gpsOrientation(1:i,2))
%      drawnow  
end


figure;
for i = 1:n
    plot(truePosition(1:i,1),truePosition(1:i,2),'-r');

    drawnow;
end

% scatter(gpsOrientation(:,1),gpsOrientation(:,2))
% hold on
% scatter(est_pos(:,1),est_pos(:,2))
% hold on            


% % Generate trajectory.
% groundTruth = waypointTrajectory('SampleRate', imuFs, ...
%     'Waypoints', position, ...
%     'TimeOfArrival', t, ...
%     'Orientation', orientation);
% 
% % Initialize the random number generator used to simulate sensor noise.
% rng('default');