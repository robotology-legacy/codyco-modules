 
 % Add path to MATLAB
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 
 
% Controller gains in P I D order
k  = [  10    0.0   2 ];
ko = [ 100 100 100      200 200 200 200 200     20 20 20 20 20 20; 
         1   1   1        1   1   1   1   1      1  1  1  1  1  1]; 
kSat = [500 500 500 3000 3000 3000];  

% 
fake = eye(25);
