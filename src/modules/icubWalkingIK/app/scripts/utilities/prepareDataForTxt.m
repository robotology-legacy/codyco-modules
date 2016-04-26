function [ newMat ] = prepareDataForTxt( mat, ts, format )
%prepareDataForTxt Summary of this function goes here
%   Detailed explanation goes here

% Add time from 0 to ...
newMat = [ [0:ts:(length(mat)-1)*ts]' mat];

% Add numbers from 1 to length(mat)
newMat = [ [1:length(newMat)]'   newMat];

end

