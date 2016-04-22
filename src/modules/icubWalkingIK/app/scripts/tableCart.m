%% .............::: SHEET 07 MORMS'14 (WS 14/15) :::.............

%% Table cart mparameters
function tableCart(fileName)
close all;
clc;
addpath(genpath('utilities'));
params = parseParams('walkingParams.txt');
if (~isstruct(params))
    display('[ERROR] walkingParams.txt doesnt exist.');
end
display(['[WARNING] Using walkingParams.txt found in ' pwd]);

z_c = params.z_c;               % [m]
T_stride = params.T_stride;     % [s]
g = params.g;                   % [ms^(-2)]

n_strides = params.n_strides;
step_width = params.step_width;
step_length = params.step_length;
zmp_tol = params.zmp_tol;
T_0 = params.ts;
T_switch = params.T_switch;

foot_width = 0.07;
foot_length = 0.14;

%% ............ Build ZMP motion
% If you want to change T_0 (ts in walkingParams.txt) you should modify
% Tstride accordingly.
%% New way. n_samples should be computed and not specified
n_sub_smpl = T_stride/(2*T_0);
n_samples = floor(n_sub_smpl*2*(n_strides+1));
n_samples = n_samples + 1;
params.n_samples = n_samples;

%% Update computed samples into walkingParams.txt along with the other parameters. This is necessary because MUSCOD would eventually need it from config file
fid = fopen('walkingParams.txt','w');
formatSpec = '%s %f\n';
fnames = fieldnames(params);
values = struct2cell(params);
nrows = size(fieldnames(params),1);
S = cell(nrows,2);
for row = 1:nrows
    S(row,1) = fnames(row);
    S(row,2) = values(row);
    if (strcmp(fnames(row),'n_samples') || ...
            strcmp(fnames(row),'n_strides') || ...
            strcmp(fnames(row),'T_stride'))
        formatSpec = '%s %i\n';
    end
    fprintf(fid, formatSpec, S{row,:} );
end
fclose(fid);

%%
% n_sub_smpl = floor(n_samples / ((n_strides+1)*2));
% T_0 = T_stride/(n_sub_smpl*2);
n_cur_smpl = n_sub_smpl+1;
step_cur = 0;

% pre-allocate
pos_ZMP = zeros(n_samples, 3);
pos_ZMP(ceil(n_samples/2):n_samples,2) = (n_strides*2-1)*step_length;

%pos_ZMP(:,1) = linspace(0.0, n_samples*T_0,n_samples)';
%pos_ZMP(:,1) = 0.0:0.010:10;

switch_smpl = ceil(T_switch/T_0);
pos_ZMP(:,1) = 0.0:T_0:(T_0*(n_samples-1));
step_cur_x = 0;
step_cur_y = 0;
step_inc = step_length/(2*switch_smpl);
step_width_inc = step_width/(2*switch_smpl);

% Only for the first step
for n_smp = n_cur_smpl:(n_sub_smpl + switch_smpl)
    % Decreasing ramp
    pos_ZMP(n_cur_smpl, 3) = step_cur_y;
    %step_cur_x = step_cur_x + step_inc;
    step_cur_y = step_cur_y - step_width_inc;
    n_cur_smpl = n_cur_smpl + 1;
end

%% 
for n_stride = 1:n_strides
    % (Constant y) (constant x)
    for n_smp = 0:(n_sub_smpl - 2*switch_smpl - 1)
        pos_ZMP(n_cur_smpl, 2:3) = [ step_cur_x, step_cur_y];
        %step_cur_x = step_cur_x + step_inc;
        %step_cur_y = step_cur_y - step_width_inc;
        n_cur_smpl = n_cur_smpl + 1;
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
    % (increasing y) (increasing x)
    for n_smpl = 0:switch_smpl - 1
        pos_ZMP(n_cur_smpl,2:3) = [ step_cur_x, step_cur_y];
        step_cur_x = step_cur_x + step_inc;
        step_cur_y = step_cur_y + step_width_inc;
        n_cur_smpl = n_cur_smpl + 1;
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
    
    % (Increasing y) (increasing x)
    for n_smpl = 0:switch_smpl-1
        pos_ZMP(n_cur_smpl, 2:3) = [step_cur_x, step_cur_y];
        step_cur_x = step_cur_x + step_inc;
        step_cur_y = step_cur_y + step_width_inc;
        n_cur_smpl = n_cur_smpl + 1;
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
    
    % (Constant y) (constant x)
    for n_smp = 0:(n_sub_smpl - 2*switch_smpl - 1)
        pos_ZMP(n_cur_smpl, 2:3) = [ step_cur_x, step_cur_y];
        n_cur_smpl = n_cur_smpl + 1;
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
    
    % (Decreasing y) (Increasing x)
    for n_smp = 0:switch_smpl-1
        pos_ZMP(n_cur_smpl, 2:3) = [ step_cur_x, step_cur_y];
        if (n_stride < n_strides)
            step_cur_x = step_cur_x + step_inc;
        end
        step_cur_y = step_cur_y - step_width_inc;
        n_cur_smpl = n_cur_smpl + 1;
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
    if (n_stride < n_strides)
        % (Decreasing y) (Increasing x)
        for n_smp = 0:switch_smpl-1
            pos_ZMP(n_cur_smpl, 2:3) = [ step_cur_x, step_cur_y];
            step_cur_x = step_cur_x + step_inc;
            step_cur_y = step_cur_y - step_width_inc;
            n_cur_smpl = n_cur_smpl + 1;
        end
    end
    subplot(121);
    plot(pos_ZMP(:,1), pos_ZMP(:,2));
    subplot(122);
    plot(pos_ZMP(:,1), pos_ZMP(:,3));
end
%% ............ Prepare Least Square System

% pre-allocate
pos_CoM = zeros(n_samples+2,2);

pos_CoM(1,1:2) = [ 0.0, 0.0];
pos_CoM(n_samples+2,1:2) = [  pos_ZMP(n_samples,2), 0.0 ];

%% <<<<<<< implement code here

Axx = eye(n_samples,n_samples);
%Axx = zeros(n_samples,n_samples);
bxx = zeros(n_samples,1);
Ayy = eye(n_samples,n_samples);
%Ayy = zeros(n_samples,n_samples);
byy = zeros(n_samples,1);

%% As done by Kajita
a = -z_c/(g*T_0^2);
b = 2*z_c/(g*T_0^2) + 1;
c = -z_c/(g*T_0^2);

centralDiag = diag(b*ones(1,n_samples));
centralDiag(1,1) = a + b;
centralDiag(end,end) = b + c;
belowDiag = diag(a*ones(1,n_samples-1),-1);
aboveDiag = diag(c*ones(1,n_samples-1),1);

Axx = centralDiag + belowDiag + aboveDiag;

%% Creating Matrix Ayy
Ayy = Axx;
bxx = pos_ZMP(1:n_samples,2);
byy = pos_ZMP(1:n_samples,3);
%[AxxQ, QxxR] = qr(Axx);
%[AyyQ, QyyR] = qr(Ayy);

pos_CoM(2:n_samples+1,1) = Axx\bxx;
pos_CoM(2:n_samples+1,2) = Ayy\byy;
%% ............ open file if given
data_MUSCOD = [];

if (nargin == 1)
    p_pArgList = argv ();
    filename = p_pArgList{1};
    data_MUSCOD = dlmread(filename,'','');
end


plWindowTraj_ptr = figure(1);

subplot(1,2,1);
hold on;
plot(pos_ZMP(1:n_samples,1),pos_ZMP(1:n_samples,2),'b');
plot(pos_ZMP(1:n_samples,1),pos_CoM(1:n_samples,1),'r');
if (~isempty(data_MUSCOD))
    plot(data_MUSCOD(:,1),data_MUSCOD(:,2),'g');
    plot(data_MUSCOD(:,1),data_MUSCOD(:,8),'c');
end
grid on;
title('ZMP Trajectory X');
ylabel('X [m]');
xlabel('t [s]');
alimitsXX = axis();
alimitsXX(1) = alimitsXX(1)-(alimitsXX(2)-alimitsXX(1))*0.25;
alimitsXX(2) = alimitsXX(2)+(alimitsXX(2)-alimitsXX(1))*0.25;
alimitsXX(3) = alimitsXX(3)- (alimitsXX(4)-alimitsXX(3))*0.1;
alimitsXX(4) = alimitsXX(4)+(alimitsXX(4)-alimitsXX(3))*0.1;
axis(alimitsXX)
hold off;

subplot(1,2,2);

hold on;
plot(pos_ZMP(1:n_samples,1),pos_ZMP(1:n_samples,3),'b');
plot(pos_ZMP(1:n_samples,1),pos_CoM(1:n_samples,2),'r');
if (~isempty(data_MUSCOD))
    plot(data_MUSCOD(:,1),data_MUSCOD(:,5),'g');
    plot(data_MUSCOD(:,1),data_MUSCOD(:,9),'c');
end
grid on;
title('ZMP Trajectory Y');
ylabel('Y [m]');
xlabel('t [s]');
alimitsYY = axis();
alimitsYY(1) = alimitsYY(1)-(alimitsYY(2)-alimitsYY(1))*0.25;
alimitsYY(2) = alimitsYY(2)+(alimitsYY(2)-alimitsYY(1))*0.25;
alimitsYY(3) = alimitsYY(3)- (alimitsYY(4)-alimitsYY(3))*0.1;
alimitsYY(4) = alimitsYY(4)+(alimitsYY(4)-alimitsYY(3))*0.1;
axis(alimitsYY)
hold off;

plWindowStep_ptr = figure(2);

hold on;
plot(pos_ZMP(1:n_samples,2),pos_ZMP(1:n_samples,3),'b');
plot(pos_CoM(1:n_samples,1),pos_CoM(1:n_samples,2),'r');
if (~isempty(data_MUSCOD))
    plot(data_MUSCOD(:,2),data_MUSCOD(:,5),'g');
    plot(data_MUSCOD(:,8),data_MUSCOD(:,9),'c');
end
drawFoot([0.0, -step_width*0.5], zmp_tol, foot_width, foot_length);
drawFoot([0.0, +step_width*0.5], zmp_tol, foot_width, foot_length);

footCursor = 0;
for n_step = 1:n_stride
    drawFoot([footCursor, -step_width*0.5], zmp_tol, foot_width, foot_length);
    drawFoot([footCursor+step_length, +step_width*0.5], zmp_tol, foot_width, foot_length);
    footCursor = footCursor+step_length*2;
end

drawFoot([(n_strides*2-1)*step_length, -step_width*0.5], zmp_tol, foot_width, foot_length);
drawFoot([(n_strides*2-1)*step_length, +step_width*0.5], zmp_tol, foot_width, foot_length);

grid on;
alimitsYY = axis();
alimitsYY(1) = alimitsYY(1) - step_length*0.5;
alimitsYY(2) = alimitsYY(2) + step_length*0.5;
alimitsYY(3:4) = [-(-alimitsYY(1)+alimitsYY(2))*0.25, (-alimitsYY(1)+alimitsYY(2))*0.25];
axis(alimitsYY)
hold off;

%% what for user to wake up
%stop;

%% save the trajectory as image on disk
%print(plWindowTraj_ptr,'Trajectory.eps','-deps','-color','-S640,240','-FArial:12');
%print(plWindowStep_ptr,'Stepping.eps','-deps','-color', '-S320,160','-FArial:12');

%% clean up mess and bail out
%close(plWindowTraj_ptr);
%close(plWindowStep_ptr);

% Generate CSV file with the following format
% | time | com_x | com_y | zmp_x | zmp_y
% zmp from pos_ZMP
% com from pos_CoM
trajectories = [pos_ZMP(:,1) pos_CoM(1:n_samples,1) pos_CoM(1:n_samples,2) pos_ZMP(:,2) pos_ZMP(:,3)];
% csvwrite([getenv('ICUB_WALK_ROOT') '/WalkTrajectories/comTraj_iCubGenova01.csv'],trajectories);
csvwrite([getenv('INSTALLED_ROBOT_DIRS') '/comTraj_iCubGenova01.csv'],trajectories);
display('COM trajectory saved in: ');
display([getenv('INSTALLED_ROBOT_DIRS') '/comTraj_iCubGenova01.csv']);

%% Footholds
[r_foot_pattern_aug, l_foot_pattern_aug] = extractFootholds(pos_ZMP, params);
writeToCSV('r_foot_pattern_aug',r_foot_pattern_aug, [getenv('INSTALLED_ROBOT_DIRS') '/']);
writeToCSV('l_foot_pattern_aug',l_foot_pattern_aug, [getenv('INSTALLED_ROBOT_DIRS') '/']);
copyfile('walkingParams.txt',getenv('INSTALLED_ROBOT_DIRS'));
display('Walking parameters file installed as: ');
display([getenv('INSTALLED_ROBOT_DIRS') '/walkingParams.txt']);

end