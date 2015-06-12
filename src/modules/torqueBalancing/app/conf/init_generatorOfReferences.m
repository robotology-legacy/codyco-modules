noOscillationTime        = 10;

robotName = 'icub';
localName = 'refGen4TorqueBalancing';

outputPortCoM       = ['/', localName, '/comDes:o'];
outputPortPostural  = ['/', localName, '/qDes:o'];
balancingPort       = 'myTest';


% Controller period
Ts                = 0.01;
simulationTime    = inf;

ROBOT_DOF = 23;
directionOfOscillation  = [0;1;0];
referenceParamsCoM      = [0.04  0.6];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz
referenceParamsPost     = [20*pi/180  0.05];           
mask                    = zeros(ROBOT_DOF ,1);
mask(7)                 = 1;                    