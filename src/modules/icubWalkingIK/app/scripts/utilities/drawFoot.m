%.........::: FUNCTION: drawFoot :::................................................
% 
% Suspends the program until keypress and informs the user to do so
%

function [] = drawFoot(pos, zmp_tol, foot_width, foot_length)
  pos_x = pos(1,1);
  pos_y = pos(1,2);
  footSketch = [ pos_x-foot_length*0.5, pos_y-foot_width*0.5;
		pos_x-foot_length*0.5, pos_y+foot_width*0.5;
		pos_x+foot_length*0.5, pos_y+foot_width*0.5;
		pos_x+foot_length*0.5, pos_y-foot_width*0.5;
		pos_x-foot_length*0.5, pos_y-foot_width*0.5];
  plot(footSketch(:,1), footSketch(:,2),'k--');
  zmpTolSketch = [ pos_x-zmp_tol/2, pos_y-zmp_tol/2;
                   pos_x-zmp_tol/2, pos_y+zmp_tol/2; 
		           pos_x+zmp_tol/2, pos_y+zmp_tol/2;
		           pos_x+zmp_tol/2, pos_y-zmp_tol/2;
		           pos_x-zmp_tol/2, pos_y-zmp_tol/2];
  plot(zmpTolSketch(:,1), zmpTolSketch(:,2),'k--');

end
