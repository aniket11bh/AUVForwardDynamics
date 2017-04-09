function [ newState ] = propagateNavState( prevState, Input, Estimator_type, tinc )
% PROPAGATE_NAV_STATE, calculates navigation states X(t_(k+1)) using data x , xdot of prev. time t
%
% Inputs :
%   prevState : X(t_k) [9x1] for EKF [15x1] for IKF or EKF_RL
%   Input : for 'ekf' or 'ikf' : 
%               [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
% 
%   Estimator_type : 'ekf', 'ikf' or 'ekfRL'
%   tinc : timestep [sec]

if strcmp(Estimator_type, 'ekf')
    % EKF
    predState = prevState + NavStateDotWithoutBias(prevState, Input, tinc)*tinc;
    corrState = prevState + NavStateDotWithoutBias(predState, Input, tinc)*tinc;
    newState = (predState + corrState)/2; 

elseif strcmp(Estimator_type, 'ikf')
    % IKF
    predState = prevState + NavStateDot(prevState, Input, tinc)*tinc;
    corrState = prevState + NavStateDot(predState, Input, tinc)*tinc;
    newState = (predState + corrState)/2;

elseif strcmp(Estimator_type, 'ekfRL')
    %EKF_RL
    predState = prevState + NavStateDotEKF_RL(prevState, Input, tinc)*tinc;
    corrState = prevState + NavStateDotEKF_RL(predState, Input, tinc)*tinc;
    newState = (predState + corrState)/2;
    
else
    % Error
    disp('Error invalid state vector size');

end

end
