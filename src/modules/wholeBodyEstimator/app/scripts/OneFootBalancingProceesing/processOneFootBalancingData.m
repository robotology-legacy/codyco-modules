genpath('plotFunctions')
close all
right_foot_FT_offset = [296.9213  -80.3759  45.6567  -0.1511  -2.914  -1.0459];
%% COP FOR ONE FOOT BALANCING WITHOUT FLOATING BASE ESTIMATE
% Import Force/Torque data
% From time = 14secs >> datum 1389
[timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/right_foot/analog/data.log',1389, 3600);
% Making time start from 0
timeFT = timeFT - timeFT(1);
% Filter and transform force measurements to world reference frame 
[f_x, f_y, f_z, u_x, u_y, u_z] = filterAndTransformFTmeas(right_foot_FT_offset, f_x, f_y, f_z, u_x, u_y, u_z);
% Compute COP
[time_cop, cop_x, cop_y] = computeCOP(timeFT, f_x, f_y, f_z, u_x, u_y, u_z);
% COP on Foot
drawFoot = 1;
plotCOP(time_cop, cop_x, cop_y, drawFoot, 'Without Floating Base Estimate');
% FT Plots
plotFTMeas( time_cop, f_x, f_y, f_z, 'No Floating Base Estimate' );
% Left leg status
figure
[time_left_leg, j0, j1, j2, j3, ~, ~] = importfileLeftLegStatus('../../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/left_leg/state/data.log',1389,3600);
time_left_leg = time_left_leg - time_left_leg(1);
plot(time_left_leg, j1);
legend('j1');

%% COP FOR ONE FOOT BALANCING WITH FLOATING BASE ESTIMATE
% From time = 24secs >> datum 2380 
[timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithQuaternionEKF/dumper/icub/right_foot/analog/data.log',2380, 4018);
% -- Plotting --
% Making time start from 0
timeFT = timeFT - timeFT(1);
% Filter and transform force measurements to world reference frame 
[f_x, f_y, f_z, u_x, u_y, u_z] = filterAndTransformFTmeas(right_foot_FT_offset, f_x, f_y, f_z, u_x, u_y, u_z);
% Computing COP
[time_cop, cop_x, cop_y] = computeCOP(timeFT, f_x, f_y, f_z, u_x, u_y, u_z);
% CoP on Foot
drawFoot = 1;
plotCOP(time_cop, cop_x, cop_y, drawFoot, 'With Floating Base Estimate');
% FT Plots
plotFTMeas( time_cop, f_x, f_y, f_z, 'With Floating Base Estimate' );
% Left leg status
figure
[time_left_leg, j0, j1, j2, ~, ~, ~] = importfileLeftLegStatus('../../../data/oneFootBalancingDataWithQuaternionEKF/dumper/icub/left_leg/state/data.log',2380,4081);
time_left_leg = time_left_leg - time_left_leg(1);
plot(time_left_leg, j1);
legend('j1');

%%
% [timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/right_foot/analog/data.log',1, 4018);
