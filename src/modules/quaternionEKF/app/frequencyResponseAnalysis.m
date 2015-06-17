%%import data
rawData = importdata('./filterResponseDump/data.log');
timeUSec = rawData(:,2) - rawData(1,2);

%% time series characteristics
dt = mean(timeUSec(2:end) - timeUSec(1:end-1));
Fs = 1/dt;

N = length(timeUSec);
accl = rawData(:,3:5);
accl_x = accl(:,1);
accl_y = accl(:,2);
accl_z = accl(:,3);
% scaling data
accl = 5.9855e-04 * accl;

dF = Fs/N;
f = -Fs/2:dF:Fs/2-dF;

bin_vals = [0 : N-1];
fax_Hz = bin_vals*Fs/N;
N_2 = ceil(N/2);

%% plotting sensor data
figure(1);
plot(timeUSec,accl);axis tight;
xlabel('time (sec)');
ylabel('accl (m/sec^2)');
legend('x','y','z');
figure(2);
subplot(1,3,1);
accl_x_mags = abs(fft(accl(:,1)));

maxY = 0.35;

%% plotting single side frequency response
plot(fax_Hz(1:N_2), accl_x_mags(1:N_2) ./ max(accl_x_mags(1:N_2)),'b');
xlabel('Frequency (Hz)')
ylabel('Magnitude');
title('a_x');
axis tight
a = axis();
axis([a(1),a(2),0,maxY]); 

subplot(1,3,2);
accl_y_mags = abs(fft(accl(:,2)));

plot(fax_Hz(1:N_2), accl_y_mags(1:N_2) ./ max(accl_y_mags(1:N_2)),'g');
xlabel('Frequency (Hz)')
ylabel('Magnitude');
title('a_y');
axis tight
a = axis();
axis([a(1),a(2),0,maxY]); 

subplot(1,3,3);
accl_z_mags = abs(fft(accl(:,3)));

plot(fax_Hz(1:N_2), accl_z_mags(1:N_2)./ max(accl_z_mags(1:N_2)),'r')
xlabel('Frequency (Hz)')
ylabel('Magnitude');
title('a_z');
axis tight
a = axis();
axis([a(1),a(2),0,maxY]); 
