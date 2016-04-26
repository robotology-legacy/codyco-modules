function [ output_args ] = drawSinglePlot( forces, cop, joint_angle, super_title)
%DRAWSINGLEPLOT Summary of this function goes here
%   Detailed explanation goes here

% Load saved figures
h_forces = hgload(forces);
h_cop = hgload(cop);
h_joint_angle = hgload(joint_angle);

% To define the highlighting area limits
t_init = 6.5;
t_end  = 8.0;
EdgeColorArea = [158/255 29/255 25/255];
FaceColorArea = [255/255 0/255 0/255];
alphaArea = 0.1;

% Subplots
figure;

h(1) = subplot(4,1,1);
% Paste figures on the subplots
axes_h_forces = allchild(get(h_forces,'children'));
% Only the z force, 4 for y, 6 for x
axes_h_forces_x = axes_h_forces{2};
copyobj(axes_h_forces_x,h(1)); 
axis(h(1),'tight');
lim = axis(h(1));
level = lim(4);
hold on;
grid on;
area(h(1),[t_init;t_end], [lim(3);lim(3)] , level, 'FaceColor', FaceColorArea, 'FaceAlpha', alphaArea, 'EdgeColor', EdgeColorArea);
ylabel(h(1),'F_z [N]');
title(h(1), super_title);

h(2) = subplot(4,1,2);
% Copy COP x
axes_h_cop = allchild(get(h_cop,'children'));
axes_h_cop_x = axes_h_cop{2};
copyobj(axes_h_cop_x,h(2));
axis(h(2),'tight');
lim = axis(h(2));
level = lim(4);
hold on;
grid on;
area(h(2),[t_init;t_end], [lim(3);lim(3)] , level, 'FaceColor', FaceColorArea, 'FaceAlpha', alphaArea, 'EdgeColor', EdgeColorArea);
ylabel(h(2),'CoP_X [m]');

h(3) = subplot(4,1,3);
% Copy COP y
axes_h_cop_y = axes_h_cop{1};
copyobj(axes_h_cop_y,h(3));
axis(h(3),'tight');
lim = axis(h(3));
level = lim(4);
hold on;
grid on;
area(h(3),[t_init;t_end], [lim(3);lim(3)] , level, 'FaceColor', FaceColorArea, 'FaceAlpha', alphaArea, 'EdgeColor', EdgeColorArea);
ylabel(h(3),'CoP_Y [m]');

h(4) = subplot(4,1,4);
% Copy left leg joint 1 angle
axes_h_joint_angle = allchild(get(h_joint_angle,'children'));
axes_h_joint_angle_j1 = axes_h_joint_angle;
copyobj(axes_h_joint_angle_j1,h(4));
axis(h(4),'tight');
lim = axis(h(4));
level = lim(4);
hold on;
grid on;
area(h(4),[t_init;t_end], [lim(3);lim(3)] , level, 'FaceColor', FaceColorArea, 'FaceAlpha', alphaArea, 'EdgeColor', EdgeColorArea);
ylabel(h(4),'Left leg J_1 [Deg]')
xlabel(h(4), 'Time [secs]');

% Close all figures
close(h_forces)
close(h_cop)
close(h_joint_angle)


end

