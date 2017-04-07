function [ x_dot ] = NavStateDot( X , U, tinc)
% NAV_STATE_DOT calculates estimated value x_dot = f(x_cap,u) and returns x_dot_cap
%
% Inputs :
%   X : [x, y, z, phi, theta, psi, v_bx, v_by, v_bz, ba_x, ba_y, ba_z, bg_x, bg_y, bg_z]
%           Note that, x, y, z are expressed in tangent frame [m],
%                      v_bx, v_by, v_bz expressed in body fixed frame. [m/s]
%                      bias are in gyro and accel. frames respectively.
% 
%   U is IMUInput : [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
%                   are accelerometer and gyro data in m/s2 and rad/s in their resp. frames

global IMU_to_body;
global gravity;
global d_IMU;
global R_i2t;
global earth_rate;
persistent prevWb;
persistent wb_dot;


g_t = [0 0 1]'*gravity; % gravity in tangent frame
R_t2b = DCM(X(4:6));
x_dot = zeros(9,1);

earth_rate_t = R_i2t*earth_rate; % earth_rate in tangent frame.
G_b = R_t2b* (g_t - cross(earth_rate_t, cross(earth_rate_t, X(1:3) ) ) );

wb = IMU_to_body*(U(4:6) - X(13:15)); % Body rate - gyrobias
u = U(1:3) - X(11:13); % Meas Acceleration - accelbias

if isempty(prevWb)
    prevWb = wb;
    wb_dot = [0 0 0]';
end

if(~isequal(wb, prevWb))
    wb_dot = (wb - prevWb)/tinc;
end

prevWb = wb;

% dot{r_{b/t}^{t}} %
x_dot(1:3) = R_t2b' * X(7:9);

% theta %
x_dot(4:6) = euler_to_bodyRates(X(4:6), -1) * (wb  - R_t2b*earth_rate_t);

% dot{v_{b/t}^{b}} %
x_dot(7:9) = IMU_to_body*u + G_b ...
             - R_t2b*cross(earth_rate_t, R_t2b'*X(7:9)) ...
             - cross(wb, X(7:9));
         
% bias %
x_dot(10:15) = 0; 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compensate for acc. due to IMU at diff. position than bf origin.
% omega_matrix_tangentf = R_t2b'*skew(IMU_to_body*(Y(4:6)-X(13:15)))*R_t2b;
% omegadot_matrix_tangentf = R_t2b'*skew(alpha)*R_t2b; % req. alpha in body frame
% acc_residual_IMU2bf = - R_t2b*omega_matrix_tangentf*omega_matrix_tangentf*R_t2b*pos_IMU ...
%                       - R_t2b*omegadot_matrix_tangentf*R_t2b*pos_IMU;

% end
