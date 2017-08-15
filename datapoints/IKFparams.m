% Use this after AUVsensors

global sigma_a sigma_g sigma_aGM sigma_gGM P Q R tinc DVL PSENSOR

sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_aGM = sqrt(tinc/accel_corr_time)*accelerometer_bias_instability;
sigma_gGM = sqrt(tinc/accel_corr_time)*gyroscope_bias_instability;

P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05, zeros(1,6) ]);
Q = diag([sigma_a^2*ones(1,3) sigma_g^2*ones(1,3) sigma_aGM^2*ones(1,3) sigma_gGM^2*ones(1,3)]);
R = diag([0.05, 0.00087, 0.05]);

DVL = 1;
PSENSOR = 0;