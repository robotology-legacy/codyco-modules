 clear all
 % Add path to MATLAB 
addpath(genpath('/home/daniele/MATLAB'))
addpath(genpath('/home/daniele/src/codyco/build'))
addpath(genpath('/home/daniele/src/codyco/src/simulink'))

% Controller period
Ts = 0.01; 
 
% Controller gains in P I D order
kCom  = [  40    0.5   0 ];
kImpTorso = [1 1 1]*13; 
kImpArms  = [1 1 1 1 1]*10;
kImpLegs  = [80 80 80 150 1 1]; 
kImp  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];

kDamTorso = [1 1 1]; 
kDamArms  = [1 1 1 1 1];
kDamLegs  = [1 1 1 1 1 1]; 
kDam      = [kDamTorso,kDamArms,kDamArms,kDamLegs,kDamLegs]*0.5;

kw = 3;
DT = 30;

idOfMovingParts = 2;  % idOfMovingParts is a vector of integers, each of which is
                     % interpreted as: 1 = torso, 2 = left arm, 3 = right arm, 4 = left leg, 5 = right leg
qFinMovingPart     = [  0    0   0   0  0   0;
                        0    0   0   0  0   0;
                       -1   -1  -1  -1 -1  -1];

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