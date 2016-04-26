addpath(genpath('utilities'));
inputFile = [ getenv('INSTALLED_ROBOT_DIRS') '/test_ik_pg.csv' ];
outputDir = getenv('INSTALLED_ROBOT_DIRS');

% The value of ts should coincide with the one in walkingParams.txt
ts = 0.010;
generateFilesForWalkPlayer(inputFile,ts,outputDir);

mainGenerateFilesForTorqueBalancing;
