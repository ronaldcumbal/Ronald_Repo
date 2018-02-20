clear all;
close all;
clc;
% Number of participants
num_people = 13;
% Add csv Files
folder = '/home/ronald/pepper-navigation/catkin_pepper/src/Experiment';
for i = 1:1:num_people
    baseFileName = sprintf('/user%d/user%d_bag_robotPose.csv',i,i);
%    baseFileName = sprintf('/user%d/user%d_bag_partnerGlobalPose.csv',i,i);
    fullFileName = fullfile(folder, baseFileName);
    if exist(fullFileName, 'file')
        data = load(fullFileName);
        data_cell{i} = data;
    end
end

%% SMOOTHNESS %%
% Number of participants
start_num = 1; 
num_people = 13;
% Plot Poses:
for k = start_num:1:num_people
    x = [];
    y = [];
    f = [];
    for j = 1:1:size(data_cell{k},1)
        x(j) = data_cell{k}(j,2); % x
        y(j) = data_cell{k}(j,3); % y
    end
    % Fit a polynomial
    p = polyfit(x,y,2);
    f = polyval(p,x);
    % Sum of squared error
    se = ((y-f).^2);
    sse = sum(se);
    % Mean squared error
    mse = mean(se);
    % Root Mean squared error
    rmse(k) = sqrt(mse);
    % Plot trajectories and fit
    figure(k);
    plot(x,y,'.');hold on;
    plot(x,f);hold on;
    axis('equal'); axis([-inf inf -0.75 0.25]);
    title(['Walking Trajectory: Participant ' num2str(k)])
    x0=600; y0=500; width=1400; height=200;
    set(gcf,'units','points','position',[x0,y0,width,height])
    xlabel('X (m)');
    ylabel('Y (m)');
end
 %% Plot errors
    figure();
    bar(rmse);
    xlabel('Participant');
    ylabel('Error');
    title('Error in Curve Fitting (Smoothness)')
    x0=600; y0=100; width=500; height=150;
    set(gcf,'units','points','position',[x0,y0,width,height])
