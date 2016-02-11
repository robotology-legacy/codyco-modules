function [time_cop, cop_x, cop_y] = computeCOP(time, f_x, f_y, f_z, u_x, u_y, u_z)
%COMPUTECOP Computes the COP given the input data
%   Detailed explanation goes here
% Filtering
% f_x = sgolayfilt(f_x - offset(1), 3, 21);
% f_y = sgolayfilt(f_y - offset(2), 3, 21);
% f_z = sgolayfilt(f_z - offset(3), 3, 21);
% u_x = sgolayfilt(u_x - offset(4), 3, 21);
% u_y = sgolayfilt(u_y - offset(5), 3, 21);
% u_z = sgolayfilt(u_z - offset(6), 3, 21);

% f_x = sgolayfilt(f_x, 3, 21);
% f_y = sgolayfilt(f_y, 3, 21);
% f_z = sgolayfilt(f_z, 3, 21);
% u_x = sgolayfilt(u_x, 3, 21);
% u_y = sgolayfilt(u_y, 3, 21);
% u_z = sgolayfilt(u_z, 3, 21);
% 

foot_wrench = [f_x, f_y, f_z, u_x, u_y, u_z];
% Transform measurements to the world reference frame orientation
w_R_ft = [1 0 0; 0 0 -1; 0 -1 0];
for i=1:size(foot_wrench,1)
    foot_wrench(i,:) = (blkdiag(w_R_ft,w_R_ft) * foot_wrench(i,:)')';
end

%CoP = [- mu_y / f_z ; + mu_x / f_z]
cop_x = -foot_wrench(:,5)./foot_wrench(:,3);
cop_y =  foot_wrench(:,4)./foot_wrench(:,3);

time_cop = time;
end