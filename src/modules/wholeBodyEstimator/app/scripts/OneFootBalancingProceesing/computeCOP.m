function [time_cop, cop_x, cop_y] = computeCOP(time, f_x, f_y, f_z, u_x, u_y, u_z, true_origin)
%COMPUTECOP Computes the COP given the input data
%   Detailed explanation goes here

if (nargin < 8 && nargin > 6)
    true_origin = 0;
end

foot_wrench = [f_x, f_y, f_z, u_x, u_y, u_z];

%CoP = [- mu_y / f_z ; + mu_x / f_z]
h = 0.01;
if (~true_origin)
    cop_x =  -foot_wrench(:,5)./(foot_wrench(:,3)); 
    cop_y =  +foot_wrench(:,4)./(foot_wrench(:,3));
else
    cop_x =  -foot_wrench(:,5)./(10.*foot_wrench(:,3)) - h*foot_wrench(:,1)./(foot_wrench(:,3));
    cop_y =  +foot_wrench(:,4)./(10.*foot_wrench(:,3)) - h*foot_wrench(:,2)./(foot_wrench(:,3));
end
    

time_cop = time;
end