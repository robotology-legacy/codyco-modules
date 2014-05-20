nj = 25;   %number of joints
nc = 12;   %number of contraints

% Add path to MATLAB
% addpath(genpath('/home/daniele/MATLAB'))
% addpath(genpath('/home/daniele/src/codyco/build'))
% addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 

k_pst_torso = [200 100 100];
k_pst_arm   = [100 100 100 100 100];
k_pst_leg   = [100 100 100 100 100 100];

k_f_linear  = [.10 .10 .10];
k_f_rotatio = [.07 .07 .07];

k_f   = diag([k_f_linear k_f_rotatio k_f_linear k_f_rotatio]);
k_pst = [ diag([k_pst_torso k_pst_arm k_pst_arm k_pst_leg k_pst_leg])    0*eye(25)   0*eye(25)];   

% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

