function [ r_foot_pattern_aug, l_foot_pattern_aug ] = extractFootholds( zmpdata, params )
%EXTRACTFOOTHOLDS Summary of this function goes here
%   Detailed explanation goes here
t = zmpdata(:,1);
zmpx = zmpdata(:,2);
zmpy = zmpdata(:,3);

% Find peaks of zmpy
[pksy,timeIdx] = findpeaks(zmpy);
timeLocs = t(timeIdx);
% from previous peaks locations extrapolate peaks for zmpx
c = ismember(t,timeLocs);
indices = find(c);
pksx = zmpx(indices);
pks = [timeLocs, pksx, pksy, zeros(length(timeLocs),1)];

l_foot_pattern = pks;
% Add final footstep
% The time for the last foothold correspond to the last time-peak of the zmp
% trajectory.
l_first_foothold = [t(1)    zmpx(1,1)    params.step_width*0.5    0.0];
l_foot_pattern = [l_first_foothold; l_foot_pattern];

%% LEFT FOOT AUGMENTED PATTERN
virtualFootholds = [];
newFoothoold = [];
halfStepTime = (timeLocs(2)-timeLocs(1))/2;
virtualTimeDelta = halfStepTime/2;

% Create time vector for l_foot_pattern_aug (first step) 
footholdTimeStep = l_foot_pattern(3,1) - l_foot_pattern(2,1);
virtualTimeStep = footholdTimeStep/4;
virtualTime = 0;
% % First half step
virtualTime = [virtualTime   virtualTimeStep];
% virtualTime = [virtualTime 2*virtualTimeStep];
% All other steps but last
virtualTime = [virtualTime   l_foot_pattern(2,1):virtualTimeStep:l_foot_pattern(end,1) ];
% virtualTime = [virtualTime   virtualTime(end)+2*virtualTimeStep  t(end)];
virtualTime = [virtualTime   virtualTime(end) + virtualTimeStep   t(end)];

% virtualTime must simply go from 0 to 10 seconds in intervals of
% T_stride/4.
virtualTime = 0:params.T_stride/4:(params.n_samples-1)*params.ts;

% Create additional LEFT LEG steps for each time in virtualTime
virtualFootholds = [];
% First half step. Without time ...
initialMidStep = [(l_foot_pattern(2,2)-l_foot_pattern(1,2))/2,  l_foot_pattern(1,3),  params.step_height];
virtualFootholds = [virtualFootholds; initialMidStep];
% Other steps but last
for i=2:(length(l_foot_pattern)-1)
    virtualFootholds = [virtualFootholds;  l_foot_pattern(i,2:end)];
    virtualFootholds = [virtualFootholds;  l_foot_pattern(i,2:end)];
    midStep          = [(l_foot_pattern(i+1,2) - l_foot_pattern(i,2))/2+l_foot_pattern(i,2)  l_foot_pattern(i,3)  params.step_height];
    virtualFootholds = [virtualFootholds;  midStep];
end
% For the last halfstep
for i = 1:2
    virtualFootholds = [virtualFootholds;  l_foot_pattern(end,2:end)];
end

% At this point virtualFootholds does not contain time
aux_l_foot_pattern = l_foot_pattern(:,2:end);
% insert virtualtFootholds into aux_l_foot_pattern
indices = repmat([2:length(l_foot_pattern)-1]',1,3);
indices = reshape(indices',numel(indices),1);
% Add first and last halfstep indeces
indices = [1; indices];
indices = [indices; length(l_foot_pattern); length(l_foot_pattern)];

% Add virtualTime column
aux_l_foot_pattern = insertrows(aux_l_foot_pattern, virtualFootholds, indices);

% Add dummy null steps at the beginning and at the end just because the first footstep
% alhtough it's a half-step it has the duration of a full step (faster than
% a normal step).
aux_l_foot_pattern = [aux_l_foot_pattern(1,:);  ...
                      aux_l_foot_pattern(1,:);  ...
                      aux_l_foot_pattern];
                  
aux_l_foot_pattern = [aux_l_foot_pattern;        ...
                      aux_l_foot_pattern(end,:); ...
                      aux_l_foot_pattern(end,:)];

l_foot_pattern_aug = [];
if (length(aux_l_foot_pattern) == length(virtualTime))
    l_foot_pattern_aug = [virtualTime', aux_l_foot_pattern];
else
    error('Malformed matrices were found when creating l_foot_pattern_aug');
end

%% for the RIGHT FOOT
r_foot_pattern_aug = [];

% Find valleys
% -- This works only for straight walking trajectories
[pksy,timeIdx] = findpeaks(-zmpy);
timeLocs = t(timeIdx);
c = ismember(t,timeLocs);
indices = find(c);
pksx = zmpx(indices);
valleys = [timeLocs, pksx, -pksy, zeros(length(timeLocs),1)];

% Format:
% This includes not only the footsteps but also the virtual points in
% between. In this particular case, valleys correspond to the right foot.
% time   pox_x   pos_y   pos_z
r_foot_pattern = valleys;
% Add final footstep
r_last_foothold = [t(end)   (params.n_strides*2-1)*params.step_length   -params.step_width*0.5   0.0];
r_foot_pattern = [r_foot_pattern;  r_last_foothold];

virtualFootholds = [];
aux_r_foot_pattern = r_foot_pattern(:,2:end);
% Add two copies of the first trajectories at the beginning (as this foot
% has to wait for the first halfstep of the starting foot.

% Other steps but last
for i=2:(length(r_foot_pattern))
    virtualFootholds = [virtualFootholds;  r_foot_pattern(i-1,2:end)];
    virtualFootholds = [virtualFootholds;  r_foot_pattern(i-1,2:end)];
    midStep          = [(r_foot_pattern(i,2) - r_foot_pattern(i-1,2))/2+r_foot_pattern(i-1,2)  r_foot_pattern(i,3)  params.step_height];
    virtualFootholds = [virtualFootholds;  midStep];
end

% Add last foothold
% virtualFootholds = [virtualFootholds; r_foot_pattern(end,2:end)];
% At this point virtualFootholds does not contain time
aux_r_foot_pattern = r_foot_pattern(:,2:end);
% insert virtualtFootholds into aux_l_foot_pattern
indices = repmat([1:length(r_foot_pattern)-1]',1,3);
indices = reshape(indices',numel(indices),1);
                  
% Add virtualTime column
aux_r_foot_pattern = insertrows(aux_r_foot_pattern, virtualFootholds, indices);

% Add dummy null steps at the beginning and at the end just because the first footstep
% alhtough it's a half-step it has the duration of a full step (faster than
% a normal step).
aux_r_foot_pattern = [aux_r_foot_pattern(1,:);  ...
                      aux_r_foot_pattern(1,:);  ...
                      aux_r_foot_pattern];
aux_r_foot_pattern = [aux_r_foot_pattern; ...
                      aux_r_foot_pattern(end,:); ...
                      aux_r_foot_pattern(end,:)];
               

r_foot_pattern_aug = [];
if (length(aux_r_foot_pattern) == length(virtualTime))
    r_foot_pattern_aug = [virtualTime', aux_r_foot_pattern];
else
    error('Malformed matrices were found when creating l_foot_pattern_aug');
end

%% adjustments for slow transition
[l_foot_peaks,l_foot_peaks_locs] = findpeaks(l_foot_pattern_aug(:,4));
[r_foot_peaks,r_foot_peaks_locs] = findpeaks(r_foot_pattern_aug(:,4));

l_foot_pattern_aug(l_foot_peaks_locs-1,1) = l_foot_pattern_aug(l_foot_peaks_locs-1,1) + params.T_switch;
l_foot_pattern_aug(l_foot_peaks_locs+1,1) = l_foot_pattern_aug(l_foot_peaks_locs+1,1) - params.T_switch;

r_foot_pattern_aug(r_foot_peaks_locs-1,1) = r_foot_pattern_aug(r_foot_peaks_locs-1,1) + params.T_switch;
r_foot_pattern_aug(r_foot_peaks_locs+1,1) = r_foot_pattern_aug(r_foot_peaks_locs+1,1) - params.T_switch;

%% change signs of x and y for iCub
% l_foot_pattern_aug(:,2) = -l_foot_pattern_aug(:,2);
% l_foot_pattern_aug(:,3) = -l_foot_pattern_aug(:,3);

% r_foot_pattern_aug(:,2) = -r_foot_pattern_aug(:,2);
% r_foot_pattern_aug(:,3) = -r_foot_pattern_aug(:,3);

