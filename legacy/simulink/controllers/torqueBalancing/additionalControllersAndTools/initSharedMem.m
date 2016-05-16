nj = 25;   %number of joints
nc = 12;   %number of contraints

% Add path to MATLAB
% addpath(genpath('/home/daniele/MATLAB'))
% addpath(genpath('/home/daniele/src/codyco/build'))
% addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 
 

% Controller gains in P I D order
weights = [0.2512      0.3335       0.428      0.9506      0.9604      0.9831      0.9877      0.9974      0.9514      0.9601      0.9829      0.9879      0.9974      0.4056       0.213      0.2676      0.1629      0.0545      0.0108      0.4054       0.213      0.2674      0.1628      0.0545      0.0108];

k_pst_torso = [1 1 1]*500;
k_pst_arm   = [1 1 1 1 1]*500;
k_pst_leg   = [1 1 1 2 1.5 1.5]*200;

k_com = [ 25    0   0];
k_pst = [ diag([k_pst_torso k_pst_arm k_pst_arm k_pst_leg k_pst_leg])    0*eye(25)   0*eye(25)];   
k_pos = [ 10    0   0];
  
kw = 10;

% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

