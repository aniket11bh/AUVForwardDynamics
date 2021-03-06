clc
clear all;
close all;

%% setup
addpath('datapoints');
addpath('sensorModels');
addpath('helperFunctions');
addpath('stateEstimation');

%% load simulation parameters & data
A = csvread('GoodSensordata30min.csv',1);
vel_bf = A(:,2:4)'; % m/s
omega_bf = A(:,5:7)'*pi/180; % rad/s
position_in = A(:,8:10)'; % m
euler_angle = A(:,11:13)'*pi/180; % rad
accel_bf = A(:,20:22)'; % m/s2
omega_bf_dot = A(:,23:25)'*pi/180; % rad/s2

% time parameters
t = A(:,1);
tinc = t(2) - t(1);

% Input to state estimation
a = [t, vel_bf', omega_bf', position_in', euler_angle', A(:,14:19), accel_bf', omega_bf_dot'];

%% load auv parameters as global params. [req. tinc]
AUVsensors;
Estimator_type = 'ekf';

% Load correct kalman filter params
if strcmp(Estimator_type, 'ikf')
    IKFparams;
    X_true = [position_in;
              euler_angle;
              vel_bf;
              zeros(3,size(position_in, 2));
              zeros(3,size(position_in, 2))
              ];

    % Estimated states [ pos, euler_angles, velocity, accel_bias, gyro_bias] %
    X_est = zeros(size(X_true));
    P_est = zeros(size(X_true,2),15,15);

elseif strcmp(Estimator_type, 'ekf')
    EKFparams;
    X_true = [position_in;
              euler_angle;
              vel_bf];
          
    % Estimated states [ pos, euler_angles, velocity] %
    X_est = zeros(size(X_true));
    P_est = zeros(size(X_true,2),9,9);

else
    fprintf('Incorrect estimator type');
end

%% Loop
for i = 1:length(t)
   
   [X_est(:,i),P_est(i,:,:)] = stateEstimation(a(i,:), Estimator_type, tinc);
   
end


eStates = (X_true - X_est)';
timeVector = t;

% Save robot localization data
names = {'t', 'a_x', 'a_y', 'a_z', 's_axx', 's_ayy', 's_azz' , ...
                'g_x', 'g_y', 'g_w', 's_gxx', 's_gyy', 's_gzz' , ...
                'v_x',  'v_y', 'v_z', 's_vxx', 's_vyy', 's_vzz' , ...
                'd_x',  'd_y', 'd_z', 's_dxx', 's_dyy', 's_dzz'};

T = cell2table( num2cell(data_robotLocalization),'VariableNames',names);
writetable(T,'./datapoints/robotLocalization.csv');

plotStateEstimData(timeVector, X_est, P_est, A, Estimator_type);