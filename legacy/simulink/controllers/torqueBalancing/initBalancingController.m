clear all

% Controller period
Ts = 0.01; 
 
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and the Derivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - Gains(1)*(xcom - xcomDes) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - xDcomDes)  
%
% The fourth element is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -Gains(4)*hw  

Gains  = [  70    0   0  4];

% Impadances acting in the null space of the desired contact forces 

kImpTorso = [12 8 12]; 
kImpArms  = [10 10 10 10 5];
kImpLegs  = [35 50 0.1 30 2 10]; 

Impedances  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];


              
% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

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