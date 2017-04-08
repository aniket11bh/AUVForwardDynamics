% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


function [X_est, P_est] = stateEstimation(A, tinc)
% Estimated states [ pos, euler_angles, velocity] %

%% Get these params from dynamics
vel_bf = A(2:4)'; % m/s
omega_bf = A(5:7)'; % rad/s
position_in = A(8:10)'; % m
euler_angle = A(11:13)'; % rad
accel_bf = A(20:22)'; % m/s2
omega_bf_dot = A(23:25)'; % rad/s2

t = A(1);

format;
fprintf('t : %2.2f\n',t)
%% ekf parameters
persistent ekf;
persistent x_est
persistent p_est;
global P Q R R_Psensor;
global DVL PSENSOR;
global density gravity Patm;

if isempty(ekf)
    init_state_guess = [position_in;
                        euler_angle;
                        vel_bf];
    P_est = P;
    ekf = extendedKalmanFilter(@propagateNavState,... % State transition function
                               @getSensorData,... % Measurement function
                               init_state_guess);
    ekf.MeasurementNoise = R;
    ekf.ProcessNoise = Q;
    
end

 % Initialize states & error states
 if isempty(x_est) || isempty(p_est)
    x_est = [position_in 
             euler_angle 
             vel_bf
            ];
    p_est = P;
 end

   % Measured IMU data as INPUT, U in prediction step
   U = getEKFinputs(euler_angle, omega_bf, omega_bf_dot, accel_bf, tinc);

   % Get measurement data of DVL and Pressure sensor
   yDVL = dvl_model(vel_bf, omega_bf);
   yPsensor = pSensor_model(position_in(3));
   yPsensor = [x_est(1:2); pressureToDepth(yPsensor)];
   
        if DVL
            
            % DVL update available only at 1 HZ
            if(rem(t,1) == 0)
                % Correction step
                [x_est, p_est] = correct(ekf, yDVL, 1);
            end
            
        end
        
        if PSENSOR
            
            % pressure sensor update at 10 HZ, Correction step
            ekf.MeasurementNoise = R_Psensor;
            [x_est, p_est] = correct(ekf, yPsensor, 2);
            ekf.MeasurementNoise = R;
        end
        
        
        % Prediction step
        [x_est, p_est] = predict(ekf, U, tinc);
    X_est = x_est;
    P_est = p_est;        
end