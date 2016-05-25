function [pks, locs]=peakseek(x,minpeakdist,minpeakh)
% Alternative to the findpeaks function.  This thing runs much much faster.
% It really leaves findpeaks in the dust.  It also can handle ties between
% peaks.  Findpeaks just erases both in a tie.  Shame on findpeaks.
%
% x is a vector input (generally a timecourse)
% minpeakdist is the minimum desired distance between peaks (optional, defaults to 1)
% minpeakh is the minimum height of a peak (optional)
%
% (c) 2010
% Peter O'Connor
% peter<dot>ed<dot>oconnor .AT. gmail<dot>com
%
% Modified by Francesco Romano (iCub Facility - IIT)
% to make it work for ZMP-peak detection
% (Note: probably the current version do not work anymore as was originally inteded)

if size(x,2)==1, x=x'; end

% Find all maxima and ties
locs=find(x(2:end-1)>=x(1:end-2) & x(2:end-1)>=x(3:end))+1;

if nargin<2, minpeakdist=1; end % If no minpeakdist specified, default to 1.

if nargin>2 % If there's a minpeakheight
    locs(x(locs)<=minpeakh)=[];
end

if minpeakdist>1
    while 1

        del=diff(locs)<minpeakdist;

        if ~any(del), break; end

        % I want to keep the first element of the peak,
        % not the last one (like findpeak does)
        idx = find(del==0);
        idx = idx + 1;
        idx(idx > length(locs)) = [];
        idx = [1, idx];
        del(setdiff(1:length(locs), idx)) = 1;
        del(idx) = false;
        
        % honestly, I don't understand this code.
%         pks=x(locs);

%         [~, mins]=min([pks(del) ; pks([del false])]);
% 
%         deln=find(del);
% 
%         deln=[deln(mins==1) deln(mins==2)+1];

        locs(del)=[];

    end
end

if nargout>1,
    pks=x(locs);
end


end