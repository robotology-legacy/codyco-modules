% Add path to MATLAB
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink/controllers'))

% Controller period
Ts = 0.01;

% Controller gains
k = [ 1 1 0
      1 1 0 ];
  
% Robot's weight
m = 25;
