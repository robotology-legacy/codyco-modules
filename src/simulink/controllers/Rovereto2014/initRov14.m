% Add path to MATLAB
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink/controllers'))
cd '/home/daniele/src/codyco/src/simulink/controllers/Rovereto2014'
% Controller period
Ts = 0.01;

% Desired positions for postural task
qDes = [0.0012;         5.3057e-04;    -1.4774e-05;     -0.8198;        1.4814;      
        3.5122e-04;     0.7836;        -7.4526e-05;     -0.5181;        0.5185;
        2.0348e-04;     0.7834;        -4.3691e-05;      6.9712e-04;    0.0010;
        -1.8968e-05;	5.6388e-04;    -0.0011;          9.8418e-04;    4.2296e-04;	
        3.4129e-04;    -1.1662e-05;     3.0456e-04;     -4.1897e-04;	3.2851e-04];


% Controller gains
k = [ 0.1 0.1 0
      0.1 0.1 0 ];
  
% Robot's weight
m = 25;
