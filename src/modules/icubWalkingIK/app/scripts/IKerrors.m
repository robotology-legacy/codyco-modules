%% Specify data sources
% iCubWlakingIK writes CSV files in the following directory
dir_source = getenv('INSTALLED_ROBOT_DIRS');
% COM planned and interpolated trajectories
dir_interp_traj = [getenv('CODYCO_SUPERBUILD_DIR')  '/main/codyco-modules/bin/Debug'];
% Load generated planned trajectories
load([dir_interp_traj '/com_traj.csv']);
% Load real ROOT trajectories from inverse kinematics
load([dir_source '/real_com_traj.csv']);

%% COM Tracking
figure(1);
plot(com_traj,'--','LineWidth',3), hold on;
plot(real_com_traj,'LineWidth',3);
legend('x_r','y_r','z_r','x_{real}','y_{real}','z_{real}');
axis tight;

% Informtive legends


%% Left and right foot.
figure(2);
load([dir_interp_traj '/l_foot_traj.csv']);
load([dir_interp_traj '/r_foot_traj.csv']);
load([dir_source '/real_left_foot.csv']);
load([dir_source '/real_right_foot.csv']);
plot(real_left_foot,'-','LineWidth',3), hold on;
plot(l_foot_traj,'--','LineWidth',1);
legend('x_r','y_r','z_r','x_{real}','y_{real}','z_{real}');
axis tight;
