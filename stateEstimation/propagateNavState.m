function [ newState ] = propagateNavState( prevState, IMUInput, tinc )
% PROPAGATE_NAV_STATE, calculates navigation states X(t_(k+1)) using data x , xdot of prev. time t
%
% Inputs :
%   prevState : X(t_k) [9x1] for EKF [15x1] for IKF
%   IMUInput : [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
%   tinc : timestep [sec]

if length(prevState) == 9
    % EKF
    predState = prevState + NavStateDotWithoutBias(prevState, IMUInput, tinc)*tinc;
    corrState = prevState + NavStateDotWithoutBias(predState, IMUInput, tinc)*tinc;
    newState = (predState + corrState)/2; 

elseif length(prevState) == 15
    % IKF
    predState = prevState + NavStateDot(prevState, IMUInput, tinc)*tinc;
    corrState = prevState + NavStateDot(predState, IMUInput, tinc)*tinc;
    newState = (predState + corrState)/2;

else
    % Error
    disp('Error invalid state vector size');

end

end
