function [ Y ] = getSensorData(X, sensorId)
%GETSENSORDATA outputs DVL and pressure sensor data.
% sensorId : 1, for DVL , X : 3x1
%            2, for Pressure sensor , X : 3x1


% p_meas = Patm + density*gravity*depth

if(sensorId == 1)
    H = zeros(9,3);
    H(7:9,:) = eye(3,3);
    Y = H'*X;
elseif (sensorId == 2)
    H = zeros(9,3);
    H(1:3,:) = eye(3,3);
    Y = H'*X;
end


end

