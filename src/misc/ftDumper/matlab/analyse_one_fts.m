% [start_mean,end_mean, max_mean,min_mean] = analyse_one_fts(data,interv,name)
%
% Function for plotting the data dumped from the force/torque sensors of iCub
% It plots the 6 components (forces and moments) of the force/torque
% sensors in legs and feet. It computes the mean of the measurements across
% the acquired data (through a sliding window).
%
% Param:   data = data.log acquired thorugh dataDumper (matrix N x 8)
%          interv = dimension of sliding window
%          name = text to be displayed on the plot, the name of the sensor
% Returns: start_mean = mean of the measurements at the beginning
%          end_mean = mean of the measurements at the end of the dump
%          max_mean = maximum mean of the measurements
%          min_mean = minimum mean of the measurements
%
% Date: 27-06-2013
% Author: Serena Ivaldi (serena.ivaldi@isir.upmc.fr)
% License: GPL
% Coyright:CODYCO Consortium (www.codyco.eu)
%
function [start_mean,end_mean, max_mean,min_mean] = analyse_one_fts(data,interv,name)

N = length(data(:,1));
fts = data(:,3:8);

for i=1:6
    fts_cumsum(:,i) = cumsum(fts(:,i));
    
    fts_mean(:,i) = (fts_cumsum(interv+1:end,i)-fts_cumsum(1:end-interv,i))/interv;
    start_mean(i) = fts_mean(2,i);
    end_mean(i) = fts_mean(end,i);

    max_mean(i) = max(fts_mean(:,i));
    min_mean(i) = min(fts_mean(:,i));
end

figure;
subplot(2,3,1);
plot(fts(interv:end,1),'r'); hold; plot(fts_mean(:,1),'b'); legend('fx mis','fx mean');
subplot(2,3,2); 
plot(fts(interv:end,2),'r'); hold; plot(fts_mean(:,2),'b'); legend('fy mis','fy mean'); title(name);
subplot(2,3,3);
plot(fts(interv:end,3),'r'); hold; plot(fts_mean(:,3),'b'); legend('fz mis','fz mean');
subplot(2,3,4);
plot(fts(interv:end,4),'r'); hold; plot(fts_mean(:,4),'b'); legend('mx mis','mx mean');
subplot(2,3,5);
plot(fts(interv:end,5),'r'); hold; plot(fts_mean(:,5),'b'); legend('my mis','my mean');
subplot(2,3,6);
plot(fts(interv:end,6),'r'); hold; plot(fts_mean(:,6),'b'); legend('mz mis','mz mean');

    

