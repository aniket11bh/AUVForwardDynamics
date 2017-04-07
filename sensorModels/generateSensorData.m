% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


clc
clear all
close all
%% setup
addpath('datapoints');
addpath('sensorModels');
addpath('helperFunctions');
addpath('stateEstimation');

%% load auv parameters as global params.
% s = load('datapoints.mat');
% n = fieldnames(s);
% for k = 1:length(n)
%     eval(sprintf('global %s; %s=s.%s;',n{k},n{k},n{k}));
% end
% clear s;
AUVsensors;

%% load simulation parameters
A = csvread('GoodSensordata200Sec.csv',1);
vel_bf = A(:,2:4)'; % m/s
omega_bf = A(:,5:7)'*pi/180; % rad/s
position_in = A(:,8:10)'; % m
euler_angle = A(:,11:13)'*pi/180; % rad
accel_bf = A(:,20:22)'; % m/s2
omega_bf_dot = A(:,23:25)'; % rad/s2
euler_rates = zeros(size(accel_bf));

% time parameters
t = A(:,1);
tinc = t(2) - t(1);

IMUaccel_bf_meas = zeros(size(accel_bf)); % [accel_meas expressed in sensor frame] of contact point at IMU
Gyro_omega_bf_meas = zeros(size(omega_bf)); % [omega_meas expressed in sensor frame] of contact point at IMU
euler_rates_cal = zeros(size(omega_bf)); % rad/s [calculated euler angular rates]
euler_angles_cal = zeros(size(omega_bf)); % rad  [calculated euler angles]
omega_bf_dot_cal = zeros(size(omega_bf)); % rad/s2 [calculated angular acceleration]

accel_bias = zeros(size(accel_bf));
omega_bias = zeros(size(omega_bf));

g = [0 0 1]'*gravity;

accel_cg_bf_meas = zeros(size(accel_bf)); % [accel of c.g calculated as expressed in sensor frame w.r.t inertial frame]
vel_bf_meas = zeros(size(vel_bf)); % [ velocity of c.g calculated using sensor data]
position_in_meas = zeros(size(position_in)); % Position of c.g in inertial frame of ref.

ac_bias = [0 0 0]';
omeg_bias = [0 0 0]';

X_est = zeros(9, size(accel_bf,1));

%% Initial conditions
vel_bf_meas(:,1) = vel_bf(:,1);
X_est(7:9,1) = vel_bf(:,1);

%% Loop
for i = 1:length(t)
    
   % measured accel and omega in sensor frame
%    [IMUaccel_bf_meas(:,i), accel_bias(:,i)] = accelerometer_model(DCM(euler_angle(:,i)), omega_bf(:,i), omega_bf_dot(:,i), accel_bf(:,i), ac_bias, tinc);
%    [Gyro_omega_bf_meas(:,i), omega_bias(:,i)] = gyro_model(omega_bf(:,i), omeg_bias, tinc);
%    accel_bias = accel_bias(:,i); % prev. values of bias
%    omega_bias = omega_bias(:,i);

%    U = [IMUaccel_bf_meas(:,i); Gyro_omega_bf_meas(:,i)];

   % Transform the measured values to body frame
%    IMUaccel_bf_meas(:,i) = (IMU_to_body * IMUaccel_bf_meas(:,i)); % Acceleration of point where IMU is placed
%    Gyro_omega_bf_meas(:,i) = (IMU_to_body * Gyro_omega_bf_meas(:,i));

   % Calculate angular accl.
%    if i > 1
%        omega_bf_dot_cal(:,i) = (Gyro_omega_bf_meas(:,i) - Gyro_omega_bf_meas(:,i-1))/tinc;
%    end
%    omega_bf_dot_cal(3,i) = omega_bf_dot(3,i);

   % Calculate the acceleration of c.g in body frame
   % TODO : use measured wb and estimate wb_dot and then use them here.
%    accel_cg_bf_meas(:,i) = IMUaccel_bf_meas(:,i) + (DCM(euler_angles_cal(:,i))*g) ...
%                            - cross(Gyro_omega_bf_meas(:,i),cross(Gyro_omega_bf_meas(:,i),d_IMU)) ...
%                            - cross(omega_bf_dot(:,i),d_IMU);

   % Calculate velocity [bf]
%    if(i > 1)
%        vel_bf_meas(:,i) = vel_bf_meas(:,i-1) + accel_cg_bf_meas(:,i-1)*tinc;
%    end

   % Calculate euler rates
   
%    if(i > 1)
% %        euler_to_bodyRates(euler_angles_cal(:,i-1),-1);
%        euler_rates_cal(:,i) = (euler_to_bodyRates(euler_angles_cal(:,i-1),-1)*omega_bf(:,i)); % -1 means for body fixed rates to euler rates
%        euler_rates(:,i) = (euler_to_bodyRates(euler_angle(:,i-1),-1)*Gyro_omega_bf_meas(:,i)); % -1 means for body fixed rates to euler rates
%    end
   
   % Calculate euler angles
%    if(i > 1)
%        euler_angles_cal(:,i) =  euler_angles_cal(:,i-1) + euler_rates_cal(:,i-1)*tinc;
%    end

   % Calculate position [inertial frame]
%    if(i > 1)
%       position_in_meas(:,i) = position_in_meas(:,i-1) + ( DCM(euler_angle(:,i-1))'*vel_bf_meas(:,i-1))*tinc;
%    end

    if (i > 1)
        [IMUaccel_bf_meas(:,i-1), accel_bias(:,i-1)] = accelerometer_model(DCM(euler_angle(:,i-1)), omega_bf(:,i-1), omega_bf_dot(:,i-1), accel_bf(:,i-1), ac_bias, tinc);
        [Gyro_omega_bf_meas(:,i-1), omega_bias(:,i-1)] = gyro_model(omega_bf(:,i-1), omeg_bias, tinc);
        accel_bias = accel_bias(:,i-1); % prev. values of bias
        omega_bias = omega_bias(:,i-1);
        U = [IMUaccel_bf_meas(:,i-1); Gyro_omega_bf_meas(:,i-1)];
        X_est(:,i) = propagateNavState(X_est(:,i-1), U, tinc);
    end
end

%{
figure
plot(t,accel_bf);
hold on;
plot(t,accel_cg_bf_meas);
legend('a_{x true}','a_{y true}','a_{z true}','a_{x measured}','a_{y measured}','a_{z measured}');
xlabel('time (sec)');
ylabel('linear acc (m/s^2)');
hold off;

figure
plot(t,omega_bf*180/3.14);
hold on;
plot(t,Gyro_omega_bf_meas*180/3.14);
legend(' p_{true}','q_{true}','r_{true}','p_{measured}','q_{measured}','r_{measured}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
hold off;

figure
plot(t,euler_rates*180/3.14);
hold on;
plot(t,euler_rates_cal*180/3.14);
I = legend('$\dot{\phi}_{true}$','$\dot{\theta}_{true}$','$\dot{\psi}_{true}$'...
          ,'$\dot{\phi}_{meas}$','$\dot{\theta}_{meas}$','$\dot{\psi}_{meas}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('Euler rates (deg/s)');
hold off;

figure
plot(t,omega_bf_dot*180/3.14);
hold on;
plot(t,omega_bf_dot_cal*180/pi);
I = legend('$\dot{p}_{true}$','$\dot{q}_{true}$','$\dot{r}_{true}$' ...
          ,'$\dot{p}_{meas}$','$\dot{q}_{meas}$','$\dot{r}_{meas}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('Angular accel. (deg/s^2)');
hold off;


figure
plot(t,vel_bf);
hold on;
plot(t,vel_bf_meas);
legend('v_{x true}','v_{y true}','v_{z true}','v_{x measured}','v_{y measured}','v_{z measured}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

figure
plot(t,position_in);
hold on;
plot(t,position_in_meas);
legend('x_{true}','y_{true}','z_{true}','x_{calculated}','y_{calculated}','z_{calculated}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;

% With yaw
%{
figure
plot(t,euler_angle*180/3.14);
hold on;
plot(t,euler_angles_cal*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\psi_{true}$','$\phi$','$\theta$','$\psi$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;
%}

% Without yaw
figure
plot(t,euler_angle(1:2,:)*180/3.14);
hold on;
plot(t,euler_angles_cal(1:2,:)*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\phi$','$\theta$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;

figure
plot3(position_in(1,:), position_in(2,:), position_in(3,:));
hold on
plot3(position_in_meas(1,:), position_in_meas(2,:), position_in_meas(3,:));
hold off;
%}


figure
plot(t,position_in);
hold on;
plot(t,X_est(1:3,:));
legend('x_{true}','y_{true}','z_{true}','x_{est}','y_{est}','z_{est}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;

% with yaw
% figure
% plot(t,euler_angle*180/3.14);
% hold on;
% plot(t,X_est(4:6,:)*180/3.14);
% I = legend('$\phi_{true}$','$\theta_{true}$','$\psi_{true}$','$\phi_{est}$','$\theta_{est}$','$\psi_{est}$');
% set(I,'interpreter','latex');
% xlabel('time (sec)');
% ylabel('degrees');
% hold off;


% Without yaw
figure
plot(t,euler_angle(1:2,:)*180/3.14);
hold on;
plot(t,X_est(4:5,:)*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\phi_{est}$','$\theta_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;

figure
plot(t,vel_bf);
hold on;
plot(t,X_est(7:9,:));
legend('v_{x true}','v_{y true}','v_{z true}','v_{x est}','v_{y est}','v_{z est}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;
%}
%{
figure
plot(t,euler_rates_cal);
hold on;
plot(t,euler_angles_cal);
I = legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','$\phi$','$\theta$','$\psi$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('deg and deg/s');
hold off;

figure
plot(t,accel_bias_vec);
legend('b_{ax}','b_{ay}','b_{az}');
xlabel('time (sec)');
ylabel('Linear accel. (m/s^2)');

figure
plot(t,omega_bias_vec);
legend('b{\omega x}','b{\omega y}','b{\omega z}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
%}

% Measured parameters 
% figure
% plot(t,accel_bf);
% hold on;
% plot(t,accel_cg_bf_meas);
% legend('a_{x true}','a_{y true}','a_{z true}','a_{x measured}','a_{y measured}','a_{z measured}');
% xlabel('time (sec)');
% ylabel('linear acc (m/s^2)');
% hold off;
