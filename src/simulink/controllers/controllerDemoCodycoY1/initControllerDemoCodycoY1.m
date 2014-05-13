clear all

% Controller period
Ts = 0.01; 
 
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and the Derivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - kH(1)*(xcom - xcomDes) - kH(2)*IntErrorCoM - kH(3)*(xDcom - xDcomDes)  
%
% The fourth element is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -kH(4)*hw  

kH  = [  50    0   0 2];

kImpTorso = [30 20 20]; 
kImpArms  = [10 10 20 10 1];
kImpLegs  = [50 50 10 30 2 10];  


kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];

transitionLengths = [7 10 2];

              
qFinHands     = [ -30 20 0 50  0;
                  -30 20 0 50  0]*pi/180;

if (min(min(qFinHands)) < -3.14 || max(max(qFinHands)) > 3.14)
   error('qFinMovingPart is not valid. Some of its elements do not belong to [-3.14,3.14]'); 
end
              
% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

% The variable states (vector) identifies the state of the demo as sent from lua.
% First element:   standing
% Second element:  seeking contacts, both hands are moving
% Third element:   contact on the left hand is occurred
% Fourth element:  contact on the right hand is occurred

states = [1 2 4 8 16];

desiredHandForces    = [ 0 0 10 0 0 10]';
threasholdHandForces = [ 0 0 2  0 0 2]';


%% GAINS FOR Gazebo
% kCom  = [  40    0.2   0 ];
% kImpTorso = [3 2 4]*8; 
% kImpArms  = [2 2 2 2 1]*8;
% kImpLegs  = [35 10 0.1 40 2 0.5]; 

%% GAINS FOR iCubGenova01
% kw = 4;
% kCom  = [  40    0.2   0 ];
% kImpTorso = [3 2 4]*8; 
% kImpArms  = [2 2 2 2 1]*8;
% kImpLegs  = [35 10 0.1 40 2 0.5]; 
% 

%% Right left 
% A = 0.02;
% f = 0.15; up to 0.17
% kH  = [  70    2   0 4];
% kImpTorso = [3 2 3]*4; 
% kImpArms  = [2 2 2 2 1]*5;
% kImpLegs  = [35 50 0.1 30 2 10];  

%% GAINS FOR iCubGenova03
%
% Demo: right left
%
% kCom  = [  40    1   0 ];
% kImpTorso = [1 1 4]*15; 
% kImpArms  = [2 2 2 2 1]*15;
% kImpLegs  = [200 150 250 500 10 1]; 
% kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];
%
% 
% Demo: Constanst CoM
%
% kCom  = [  40    0   0 ];
% kImpTorso = [1 1 2]*15; 
% kImpArms  = [1 1 1 1 1]*15;
% kImpLegs  = [200 150 250 450 10 1]; 