function [ ] = plotFTMeas( timeFT, f_x, f_y, f_z, name_experiment )
%PLOTFTMEAS Summary of this function goes here
%   Detailed explanation goes here

figure, hold on;
subplot(311), hold on
plot(timeFT, f_x, 'LineWidth', 1, ...
                       'Color', [0, 0.7, 1])
title(name_experiment);
legend('f_x'), axis tight

subplot(312), hold on
plot(timeFT, f_y, 'LineWidth', 1, ...
                       'Color', [0, 1, 0.7])
title(name_experiment);                   
legend('f_y'), axis tight

subplot(313), hold on
title('Force Z Axis')
plot(timeFT, f_z, 'LineWidth', 1, ...
                       'Color', [0.7, 0, 1])
title(name_experiment);
legend('f_z'), axis tight;                            

end

