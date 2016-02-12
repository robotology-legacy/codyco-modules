function [ filt_f_x, filt_f_y, filt_f_z, filt_u_x, filt_u_y, filt_u_z ] = filterAndTransformFTmeas( offset, f_x, f_y, f_z, u_x, u_y, u_z )
%FILTERANDTRANSFORMMEAS Summary of this function goes here
%   Detailed explanation goes here
f_x = sgolayfilt(f_x - offset(1), 3, 21);
f_y = sgolayfilt(f_y - offset(2), 3, 21);
f_z = sgolayfilt(f_z - offset(3), 3, 21);
u_x = sgolayfilt(u_x - offset(4), 3, 21);
u_y = sgolayfilt(u_y - offset(5), 3, 21);
u_z = sgolayfilt(u_z - offset(6), 3, 21);


foot_wrench = [f_x, f_y, f_z, u_x, u_y, u_z];
% Transform measurements to the world reference frame orientation, x
% forward from the robot and y to the left of the robot
w_R_ft = [1 0 0; 
          0 0 -1; 
          0 -1 0];
% for i=1:size(foot_wrench,1)
%     foot_wrench(i,:) = (blkdiag(w_R_ft,w_R_ft) * foot_wrench(i,:)')';
% end

filt_f_x =  foot_wrench(:,1);
filt_f_y = -foot_wrench(:,2);
filt_f_z = -foot_wrench(:,3);
filt_u_x =  foot_wrench(:,4);
filt_u_y = -foot_wrench(:,5);
filt_u_z = -foot_wrench(:,6);

end

