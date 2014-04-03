

% Controller period
Ts = 0.01; 
 
% Controller gains in P I D order
k = [ 20   5   15
      0.1  0   0.1
      10   0   1 ];
  
% 
fake = eye(25);
