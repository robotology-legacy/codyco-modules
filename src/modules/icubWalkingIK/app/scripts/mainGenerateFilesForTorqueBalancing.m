addpath(genpath('utilities'));
% ik_joints_file = [getenv('INSTALLED_ROBOT_DIRS') '/test_ik_pg.csv' ];
% outputDir      = [getenv('INSTALLED_ROBOT_DIRS') ];
% com_file       = [getenv('INSTALLED_ROBOT_DIRS') '/com_l_sole.csv'];
% 
% filepath = getenv('INSTALLED_ROBOT_DIRS');
% % !!!!Temporary!!!! Local solution for Tariq's experiments. Working only on
% % OS X if compiling in Debug mode. MUST BE CHANGED!!
% % filepath = [getenv('CODYCO_SUPERBUILD_DIR') '/main/codyco-modules/bin/Debug']; 
% filepath = [getenv('INSTALLED_ROBOT_DIRS')];
% 
% l_foot_traj    = [filepath '/l_foot_traj.csv'];
% r_foot_traj    = [filepath '/r_foot_traj.csv'];
% params         = parseParams('walkingParams.txt');
% 
% % The value of ts should coincide with the one in walkingParams.txt
% ts = params.ts;


%% COM data
% - CoM: 	9-dimensional vector (posizione, velocita e accelerazione del CoM).
comDataMat = csvread('/Users/makaveli/Projects/src/codyco-superbuild/main/codyco-modules/src/modules/walkPlayer/app/sequences/iCubGenova01/WalkTrajectories/com_l_sole.csv');
% At this point we have | time | com_x | com_y | zmp_x | zmp_y | We only
% need the first two columns and height of the COM
comDataMat = [comDataMat(:,1:2),  0.505 *ones(length(comDataMat),1),  zeros(length(comDataMat),6)];
formatSpecCom = '%8.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n';

writeToTxt(['~/Desktop' '/torqueBalancing_comTraj'],comDataMat,formatSpecCom);


% 
% 
% % Generate trajectories for joint torque controller
% % - CoM: 	9-dimensional vector (posizione, velocit??? e accelerazione del CoM).
% % - posturale: Dof-sized vector
% % - Constraints: vettore di due elementi: 1 vuol dire che il piede in contatto, 0 no. Quindi per esempio: two feet => 1,1, left foot support: 1,0, etc
% generateFilesForTorqueBalancing(com_file, ik_joints_file, l_foot_traj, r_foot_traj, params, ts, outputDir);
