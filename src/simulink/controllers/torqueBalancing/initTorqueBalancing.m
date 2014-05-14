 clear all

% Controller period
Ts = 0.01; 
 
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and dDerivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - kCom(1)*(xcom - xcomDes) - kCom(2)*IntErrorCoM - kCom(3)*(xDcom - xDcomDes)  
%
% kw is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -kw*hwDotDes  
kCom  = [  70    2   0];
kw    = 4;

% Impedence acting in the null space of HDDDes 

kImpTorso = [3 2 3]*4; 
kImpArms  = [2 2 2 2 1]*5;
kImpLegs  = [35 50 0.1 30 2 10]; 


% kCom  = [  60    0   0];
% kw    = 2;
% 
% % kH  = [  60    0   0 2];
% 
% % Impedence acting in the null space of HDDDes 
% 
% kImpTorso = [40 15 15]; 
% kImpArms  = [5 5 5 5 5];
% kImpLegs  = [35 50 0.1 30 2 10]; 



kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];


DT = 20;

idOfMovingParts = [];  % idOfMovingParts is a vector of integers, each of which is
                     % interpreted as: 1 = torso, 2 = left arm, 3 = right arm, 4 = left leg, 5 = right leg
qFinMovingPart     = [ -15   15   15   20  5   5;
                       -60   60    0   20  0   0]*pi/180;

% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];

if (min(min(qFinMovingPart)) < -3.14 || max(max(qFinMovingPart)) > 3.14)
   error('qFinMovingPart is not valid. Some of its elements do not belong to [-3.14,3.14]'); 
end
qFin = -ones(25,1)*5;

for i = 1 : length(idOfMovingParts)
    if  idOfMovingParts(i) == 1
        qFin(1:3) = qFinMovingPart(i,1:3);
    end
    if  idOfMovingParts(i) == 2 
        qFin(4:8) = qFinMovingPart(i,1:5);
    end
    if  idOfMovingParts(i) == 3 
        qFin(9:13) = qFinMovingPart(i,1:5);
    end
    if  idOfMovingParts(i) == 4 
        qFin(14:19) = qFinMovingPart(i,1:6);
    end
    if  idOfMovingParts(i) == 5 
        qFin(20:25) = qFinMovingPart(i,1:6);
    end
end


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
% kCom  = [  70    2   0 ];
% kImpTorso = [3 2 3]*4; 
% kImpArms  = [2 2 2 2 1]*8;
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