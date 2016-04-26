function generateFilesForTorqueBalancing( com_file, ik_joints_file, l_foot_traj, r_foot_traj, params, ts , outputDir )
% GENERATEFILESFORTORQUEBALANCING Summary of this function goes here
%   Detailed explanation goes here

%% COM data
% - CoM: 	9-dimensional vector (posizione, velocita e accelerazione del CoM).
comDataMat = csvread(com_file);
% At this point we have | time | com_x | com_y | zmp_x | zmp_y | We only
% need the first two columns and height of the COM
comDataMat = [comDataMat(:,1:2),  params.z_c*ones(length(comDataMat),1),  zeros(length(comDataMat),6)];

%% - posturale: Dof-sized vector
% Initial left arm configuration
initLeftArmConfig = deg2rad([-35.8      30.0     0.0      50]);
% Initial right arms configuration
initRightArmConfig = deg2rad([-35.8      30.0     0.0      50]);

posturalTraj = csvread(ik_joints_file);
torsoJoints = posturalTraj(:,end-2:end);
leftLegJoints = posturalTraj(:,1:6);
rightLegJoints = posturalTraj(:,7:12);
leftArmJoints = repmat(initLeftArmConfig, length(posturalTraj), 1);
rightArmsJoints = repmat(initRightArmConfig, length(posturalTraj), 1);
posturalTraj = [torsoJoints, leftArmJoints, rightArmsJoints, leftLegJoints, rightLegJoints];
%% - Constraints: vettore di due elementi: 1 vuol dire che il piede e in contatto, 0 no. Quindi per esempio: two feet => 1,1, left foot support: 1,0, etc
l_foot_traj_z = csvread(l_foot_traj);
l_foot_traj_z = l_foot_traj_z(:,3);

r_foot_traj_z = csvread(r_foot_traj);
r_foot_traj_z = r_foot_traj_z(:,3);

constraints = [l_foot_traj_z|l_foot_traj_z  r_foot_traj_z|r_foot_traj_z]; 
constraints = ~constraints;
% idx=find(sum(constraints,2)==0);
% constraints(idx,1)=1;
% constraints(idx,2)=1;

%% - Writing to file
formatSpecCom = '%8.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n';
formatSpecPostural = '%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n';
formatSpecConstraints = '%i %i\n';
% Rotating comDataMat x and y coordinates to coincide with
% torqueBalancing's inertial reference frame.
writeToTxt([outputDir '/torqueBalancing_comTraj'],comDataMat,formatSpecCom);
writeToTxt([outputDir '/torqueBalancing_posturalTraj'],posturalTraj,formatSpecPostural);
writeToTxt([outputDir '/torqueBalancing_constraints'],constraints,formatSpecConstraints);

end