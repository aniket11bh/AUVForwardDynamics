function [ del_y, H, R ] = dvlErrorState(yDVL, X, U, tinc)
% DVL_ERROR_STATE 
%   This Model computes Measurement Residual, Measurement Matrix and
%   Measurement Noise Covariance from the sensor input and present
%   navigation state for DVL

global d_DVL;
global dvl_noise_density;
global earth_rate;
global R_i2t;
global DVL_to_body;
global sigma_g;

%if isempty(wb)
%    wb = U(4:6);
%end

R_t2b = DCM(X(4:6));
earth_rate_t = R_i2t*earth_rate; % earth_rate in tangent frame.
sigma_v = dvl_noise_density*(1/sqrt(tinc));

%wb = wb + (euler_to_bodyRates(X(4:6), 1)*X(4:6) + R_t2b*earth_rate_t)*tinc;
wb = U(4:6)-X(13:15)-R_t2b*earth_rate_t;

yDVL_cap = DVL_to_body'*(X(7:9) + cross(wb, d_DVL));

% Computing the Output of the Error state Model %
del_y = yDVL - yDVL_cap;
H = [zeros(3,6) eye(3,3) zeros(3,3) skew(d_DVL)];
R = skew(d_DVL)*sigma_g*sigma_g*eye(3,3)*skew(d_DVL)' + sigma_v*sigma_v*eye(3,3);

end

