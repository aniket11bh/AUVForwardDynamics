function [depth] = pressureToDepth(pressure)

global density; % density of water
global gravity;
global Patm;

depth = (pressure - Patm)/(density*gravity);

end