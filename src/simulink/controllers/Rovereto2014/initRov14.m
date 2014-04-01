
 % Add path to MATLAB
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01;
 
% Controller gains in P I D order
k = [ 20   5   15
      0.1  0   0.1
      10   0   1 ];
  
% 
fake = eye(25);
