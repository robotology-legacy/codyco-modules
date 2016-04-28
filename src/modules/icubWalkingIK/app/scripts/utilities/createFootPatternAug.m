function [ foot_pattern_aug ] = createFootPatternAug( foot_pattern, timeLocs is_first_step )
%CREATEFOOTPATTERNAUG Summary of this function goes here
%   CREATEFOOTPATTERNAUG(FOOT_PATTERN,IS_FIRST_STEP)

newFoothoold = [];
halfStepTime = (timeLocs(2)-timeLocs(1))/2;
virtualTimeDelta = halfStepTime/2;

% Create time vector for foot_pattern_aug (first step) 
footholdTimeStep = foot_pattern(3,1) - foot_pattern(2,1);
virtualTimeStep = footholdTimeStep/4;
virtualTime = 0;
% First half step
virtualTime = [virtualTime 2*virtualTimeStep];
% All other steps but last
virtualTime = [virtualTime   foot_pattern(2,1):virtualTimeStep:foot_pattern(end,1) ];
virtualTime = [virtualTime   virtualTime(end)+2*virtualTimeStep  t(end)];

% Create additional LEFT LEG steps for each time in virtualTime
virtualFootholds = [];
% First half step. Without time ...
initialMidStep = [(foot_pattern(2,2)-foot_pattern(1,2))/2,  foot_pattern(1,3),  params.step_height];
virtualFootholds = [virtualFootholds; initialMidStep];
% Other steps but last
for i=2:(length(foot_pattern)-1)
    virtualFootholds = [virtualFootholds;  foot_pattern(i,2:end)];
    virtualFootholds = [virtualFootholds;  foot_pattern(i,2:end)];
    midStep          = [(foot_pattern(i+1,2) - foot_pattern(i,2))/2+foot_pattern(i,2)  foot_pattern(i,3)  params.step_height];
    virtualFootholds = [virtualFootholds;  midStep];
end
% For the last halfstep
for i = 1:2
    virtualFootholds = [virtualFootholds;  foot_pattern(end,2:end)];
end

% At this point virtualFootholds does not contain time
aux_foot_pattern = foot_pattern(:,2:end);
% insert virtualtFootholds into aux_foot_pattern
indices = repmat([2:length(foot_pattern)-1]',1,3);
indices = reshape(indices',numel(indices),1);
% Add first and last halfstep indeces
indices = [1; indices];
indices = [indices; length(foot_pattern); length(foot_pattern)];
% Add virtualTime column
aux_foot_pattern = insertrows(aux_foot_pattern, virtualFootholds, indices);
foot_pattern_aug = [];
if (length(aux_foot_pattern) == length(virtualTime))
    foot_pattern_aug = [virtualTime', aux_foot_pattern];
else
    error('Malformed matrices were found when creating foot_pattern_aug');
end


end

