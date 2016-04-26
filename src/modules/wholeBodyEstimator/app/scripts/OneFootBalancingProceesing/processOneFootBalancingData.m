genpath('plotFunctions')
close all
% Flag to save figures
save_figures = true
right_foot_FT_offset = [296.9213  -80.3759  45.6567  -0.1511  -2.914  -1.0459];
% Flag to use true origin (under the skin)
true_origin = 0;
%% COP FOR ONE FOOT BALANCING WITHOUT FLOATING BASE ESTIMATE
% Import Force/Torque data
% From time = 14secs >> datum 1389
% [timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/right_foot/analog/data.log',1389, 3600);
[timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/right_foot/analog/data.log',1389, 3372);
% Making time start from 0
timeFT = timeFT - timeFT(1);
% Filter and transform force measurements to world reference frame 
[f_x, f_y, f_z, u_x, u_y, u_z] = filterAndTransformFTmeas(right_foot_FT_offset, f_x, f_y, f_z, u_x, u_y, u_z);
% Compute COP
[time_cop, cop_x, cop_y] = computeCOP(timeFT, f_x, f_y, f_z, u_x, u_y, u_z, true_origin);
% COP on Foot
drawFoot = 1;
plotCOP(time_cop, cop_x, cop_y, drawFoot, 'Without Floating Base Estimate');
savefig('cop_without_base_estimate.fig');
% FT Plots
plotFTMeas( time_cop, f_x, f_y, f_z, 'No Floating Base Estimate' );
savefig('forces_without_base_estimate.fig');
% Left leg status
h_joint = figure;
[time_left_leg, j0, j1, j2, j3, ~, ~] = importfileLeftLegStatus('../../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/left_leg/state/data.log',1389,3372);
time_left_leg = time_left_leg - time_left_leg(1);
plot(time_left_leg, j1, 'LineWidth', 2);
savefig(h_joint,'joint_angle_without_estimate.fig');
% Put relevant plots in a single one for comparison.
drawSinglePlot('forces_without_base_estimate.fig', ...
                'cop_without_base_estimate.fig',...
                'joint_angle_without_estimate.fig',...
                'No Floating Base Estimate')

%% COP FOR ONE FOOT BALANCING WITH FLOATING BASE ESTIMATE
% From time = 24secs >> datum 2380 
[timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithQuaternionEKF/dumper/icub/right_foot/analog/data.log',2380, 4018);
% [timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../../data/oneFootBalancingDataWithQuaternionEKF/dumper/icub/right_foot/analog/data.log',1, 4018);
% -- Plotting --
% Making time start from 0
timeFT = timeFT - timeFT(1);
% Filter and transform force measurements to world reference frame 
[f_x, f_y, f_z, u_x, u_y, u_z] = filterAndTransformFTmeas(right_foot_FT_offset, f_x, f_y, f_z, u_x, u_y, u_z);
% Computing COP
[time_cop, cop_x, cop_y] = computeCOP(timeFT, f_x, f_y, f_z, u_x, u_y, u_z, true_origin);
% CoP on Foot
drawFoot = 1;
plotCOP(time_cop, cop_x, cop_y, drawFoot, 'With Floating Base Estimate');
savefig('cop_with_base_estimate.fig');
% FT Plots
plotFTMeas( time_cop, f_x, f_y, f_z, 'With Floating Base Estimate' );
savefig('forces_with_base_estimate.fig');
% Left leg status
figure
[time_left_leg, j0, j1, j2, ~, ~, ~] = importfileLeftLegStatus('../../../data/oneFootBalancingDataWithQuaternionEKF/dumper/icub/left_leg/state/data.log',2380,4018);
time_left_leg = time_left_leg - time_left_leg(1);
plot(time_left_leg, j1, 'LineWidth', 2);
% legend('j1');
savefig('joint_angle_with_estimate.fig');

drawSinglePlot('forces_with_base_estimate.fig', ...
                'cop_with_base_estimate.fig',...
                'joint_angle_with_estimate.fig',...
                'With Floating Base Estimate')

%%
% [timeFT,f_x,f_y,f_z,u_x,u_y,u_z] = importfile('../../data/oneFootBalancingDataWithoutQuaternionEKF/dumper_ft/icub/right_foot/analog/data.log',1, 4018);
