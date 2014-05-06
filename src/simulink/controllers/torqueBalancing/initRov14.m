 clear all
 % Add path to MATLAB 
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 
 
% Controller gains in P I D order
kCom  = [  40    0.5   0 ];
kImpTorso = [1 1 1]*10; 
kImpArms  = [1 1 1 1 1]*10;
kImpLegs  = [80 80 80 150 1 1]; 
kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];

kDamTorso = [1 1 1]; 
kDamArms  = [1 1 1 1 1];
kDamLegs  = [1 1 1 1 1 1]; 
kDam      = [kDamTorso,kDamArms,kDamArms,kDamLegs,kDamLegs]*0.5;

kw = 3;

% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

% 
% fake = eye(25);
