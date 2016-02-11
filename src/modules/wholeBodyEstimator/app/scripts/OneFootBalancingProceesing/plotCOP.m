function [handle_cop_x, handle_cop_y ] = plotCOP( time_cop, cop_x, cop_y ,drawFoot, name_experiment)
%PLOTCOP Summary of this function goes here
%   Detailed explanation goes here

if (drawFoot)
    %% Draw right foot
    [V8_R, F_R, ~] = read_plot_foot_mesh(1, 0);
    %should order the points in some way
    V8_R(:,2) = -V8_R(:,2);
    
    figure;
    
    %Plot feet
    trisurf(F_R, V8_R(:,1),V8_R(:,2),V8_R(:,3),'FaceColor',[255/255,234/255,71/255],'facealpha', 0.7, 'EdgeColor', 'none');
    hold on,     title(name_experiment)
    % plot(V8_R(:,1),V8_R(:,2),'k-');
    % trisurf(F_L, V8_L(:,1),V8_L(:,2),V8_L(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
end
%% Overlay ZMP
hold on;
cop_x_downsampled = downsample(cop_x,5);
cop_y_downsampled = downsample(cop_y,5);
cop_z = (max(V8_R(:,3)) + 0.001)*ones(length(cop_x),1);
rainbow_color = colormap(cool(length(cop_x)));
% Uncomment the next line if you want to inver the colormap
% rainbow_color = (fliplr(rainbow_color'))';
for i=[1:length(cop_x)]
    plot3(cop_x(i), cop_y(i), cop_z(i), '.','LineWidth',3, 'Color', rainbow_color(i,:));
    % pause(0.005)
    % drawnow
end
xlabel('cop_x'), ylabel('cop_y');

%%
figure
subplot(211)
plot(time_cop, cop_x);
title(name_experiment)
xlabel('Time (sec)'), ylabel('zmp_x'), axis tight;
subplot(212)
title(name_experiment)
plot(time_cop, cop_y);
xlabel('Time (sec)'), ylabel('zmp_y'), axis tight;

end

