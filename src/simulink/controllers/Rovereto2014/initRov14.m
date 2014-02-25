% Add path to MATLAB
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink/controllers'))
cd '/home/daniele/src/codyco/src/simulink/controllers/Rovereto2014'
% Controller period
Ts = 0.01;

% Desired positions for postural task
qDes = zeros(25,1);


% Controller gains
k = [ 1 1 0
      1 1 0 ];
  
% Robot's weight
m = 25;
