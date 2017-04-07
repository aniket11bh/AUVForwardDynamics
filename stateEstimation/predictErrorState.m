function [ delx_pred, cov_pred ] = predictErrorState( X, delx_prev, cov_prev, U, tinc )
%PREDICT_ERROR_STATE Propagates and Predicts Error State from IMU Input
% This is the prediction step of Indirect Kalman Fiter. It predicts the New
% Error State from Old Error State and IMU Process Noise

global IMU_to_body;
global gravity;
global R_i2t;
global earth_rate;
global accel_corr_time;
global gyro_corr_time;
global Q;

% Computation of Required Parameters %

g_t = [0 0 1]'*gravity; % gravity in tangent frame
earth_rate_t = R_i2t*earth_rate; % earth_rate in tangent frame.
wb = IMU_to_body*(U(4:6) - X(13:15)); % Body rate - gyrobias

R_t2b = DCM(X(4:6));
R_b2t = R_t2b';

% F Matrix %
Z = zeros(3,3);

F12 = -skew(R_b2t*X(7:9));
F22 = -skew(earth_rate_t);
F32 = R_t2b*( skew(g_t)  + earth_rate_t*(R_b2t*X(7:9))' ...
             -dot(earth_rate_t , R_b2t*X(7:9))*eye(3) );
F33 = -skew(wb) - R_t2b*skew(earth_rate_t)*R_b2t;
F21 = Z; % Assumed to be zero because very small
F31 = Z; % Assumed to be zero because very small

F = [  Z  F12  R_b2t      Z          Z;
      F21 F22      Z      Z     -R_b2t;
      F31 F32    F33 -eye(3) -skew(X(7:9))
    ];

F = [ F
      zeros(6,9) -inv(diag([accel_corr_time, accel_corr_time, accel_corr_time, ...
                             gyro_corr_time, gyro_corr_time, gyro_corr_time]))
     ];

% G Matrix %
G = zeros(15,12);
G(4:6,4:6) = -R_b2t;
G(7:9,1:3) = -eye(3,3);
G(7:9,4:6) = -skew(X(7:9));
G(10:15,7:12) = eye(6,6);

% Q matrix is global %

% % Computation of Propagation Parameters % %

% If F has dimension axa
% Dimension of gamma : (a+a)x(a+a)
gamma = expm([-F               G*Q*G'
               zeros(size(F,2))  F' ]*tinc);
           
%  Discrete time parameters %
phi = gamma( (length(delx_prev)+1:2*length(delx_prev)) , (length(delx_prev)+1:2*length(delx_prev)) )';
Q_d = phi*gamma( (1:length(delx_prev)) , (length(delx_prev)+1:2*length(delx_prev)) );

% Propagating State Here %
delx_pred = phi*delx_prev;
cov_pred = phi*cov_prev*phi'+ Q_d;

end
