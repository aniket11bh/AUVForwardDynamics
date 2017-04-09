% Use this after AUVsensors
global sigma_a sigma_g sigma_apg P Q R R_Psensor tinc DVL PSENSOR

sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_apg = sigma_a + sigma_g; %sqrt(sigma_a*sigma_a + sigma_g*sigma_g); % TODO : Not confirm

P = diag(0.05*ones(1,15));
Q = diag(0.05*ones(1,15));
R = diag([0.05, 0.00087, 0.05]);
R_Psensor = diag([999999999 999999999 0.1]);

DVL = 1;
PSENSOR = 0;