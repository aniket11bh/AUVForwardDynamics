% Use this after AUVsensors
global sigma_a sigma_g sigma_apg sigma_dvl P Q R R_Psensor tinc DVL PSENSOR data_robotLocalization

sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_dvl = dvl_noise_density*(1/sqrt(tinc));

sigma_apg = sigma_a + sigma_g; %sqrt(sigma_a*sigma_a + sigma_g*sigma_g); % TODO : Not confirm
P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05]);
Q = diag([0,0,0, sigma_g, sigma_g, sigma_g, sigma_apg, sigma_apg, sigma_apg]);
R = diag([0.05, 0.00087, 0.05]);
R_Psensor = diag([999999999 999999999 0.1]);

DVL = 1;
PSENSOR = 1;

% Store data of sensors for robot localization pkg in following format
% data_robotLocalization = ['t',  ...
%                                              'a_x', 'a_y', 'a_z', 's_axx', 's_ayy', 's_azz' , ...
%                                              'g_x', 'g_y', 'g_w', 's_gxx', 's_gyy', 's_gzz' , ...
%                                              'v_x',  'v_y', 'v_z', 's_vxx', 's_vyy', 's_vzz' , ...
%                                              'd_x',  'd_y', 'd_z', 's_dxx', 's_dyy', 's_dzz' ,];
% Now,  's_axx' =  's_ayy' =  's_azz'  = sigma_a
% Now,  's_gxx' =  's_gyy' =  's_gzz'  =  sigma_g
%              's_dzz' = R_Psensor(3)