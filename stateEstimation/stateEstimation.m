% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


function [X_est, P_est] = stateEstimation(A, type ,tinc)
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

persistent ekf;
persistent x_est;
persistent p_est;
persistent del_X;
persistent bias;
global P Q R R_Psensor;
global density gravity Patm;
global DVL PSENSOR;

% Check type %
if type == 'ekf'

    % EKF
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

    % Get measurement data of DVL
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

elseif type == 'ikf'
   % Error state kalman filter

   % Initialize states & error states
    if isempty(x_est) || isempty(p_est) || isempty(del_X)
        x_est = [position_in
                 euler_angle
                 vel_bf
                 zeros(3,1)
                 zeros(3,1)
                ];
        p_est = P;
        del_X = zeros(15,1);
    end

    % Measured IMU data as INPUT, U in prediction step
    [U, bias] = getIKFinputs(euler_angle, omega_bf, omega_bf_dot, accel_bf, tinc);
    x_est(4:6) = euler_angle;
    x_est = propagateNavState(x_est, U, tinc);
    [del_X, p_est ] = predictErrorState(x_est, del_X, p_est, U, tinc);

    % If update available
    yDVL = dvl_model(vel_bf, omega_bf);

    if DVL
         % DVL update available only at 1 HZ
%          if(rem(t,1) == 0)
             [del_y, H, R] = dvlErrorState(yDVL, x_est, U, tinc);
             [del_X, p_est] = correctErrorState(del_y, H, R, del_X, p_est);
             x_est = x_est + del_X;
%              del_X = zeros(15,1);
%          end
    end

    X_est = x_est;
    P_est = p_est;

else
    fprintf('Unknown estimator type');
end

end
