clear;

% Create iDynTree object
robot = iDynTree.DynamicsComputations();
%load model from urdf file
modelFile = '/Users/makaveli/Projects/src/codyco-superbuild/build/install/share/codyco/robots/iCubGenova02/model.urdf';
robot.loadRobotModelFromFile(modelFile);


%desired joint configuration
qDes = zeros(23,1);
comDes = zeros(3,1);
feetInSupport = [1, 1];
[qOptim, error] = jointReferencesGenerator(robot, qDes, comDes, feetInSupport);