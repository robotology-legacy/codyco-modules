nj = 25;   %number of joints
nc = 6;    %number of unonctrolled contraints 
nf = 6;    %number of controlled contraints

% Add path to MATLAB
% addpath(genpath('/home/daniele/MATLAB'))
% addpath(genpath('/home/daniele/src/codyco/build'))
% addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 

k_pst_torso = [0 0 0];
k_pst_arm   = [0 0 0 0 0];
k_pst_leg   = [0 0 0 0 0 0];

k_f_linear  = [0 0 0];
k_f_rotatio = [0 0 0];

k_f   = diag([k_f_linear k_f_rotatio]);
k_pst = [ diag([k_pst_torso k_pst_arm k_pst_arm k_pst_leg k_pst_leg])    0*eye(25)   0*eye(25)];   

% Rotation of the gazebo FT sensor
R   = [1 0 0; 0 -1 0;0 0 -1];
Rf  = [R, zeros(3,3); zeros(3,3), R];
