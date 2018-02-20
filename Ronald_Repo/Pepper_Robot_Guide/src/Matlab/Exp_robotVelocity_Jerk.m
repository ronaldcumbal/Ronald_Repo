clear;
close all;
clc;
% Number of participants
num_people = 20;
% Add csv Files
folder = '/home/ronald/pepper-navigation/catkin_pepper/src/Experiment';
for i = 1:1:num_people
    baseFileName = sprintf('/user%d/user%d_bag_robotVel.csv',i,i);
    fullFileName = fullfile(folder, baseFileName);
    if exist(fullFileName, 'file')
        data = load(fullFileName);
        data_cell{i} = data;
    end
end


%% Magnitude Speed:
for k = 1:1:size(data_cell,2)
    for j = 1:1:size(data_cell{k},1)
        velx = data_cell{k}(j,2);
        vely = data_cell{k}(j,3);
        velz = data_cell{k}(j,4);
        data_cell{k}(j,8) = sqrt((velx^2)+(vely^2)+(velz^2));
    end
    % Plot 
%    figure(k);
%    figure(1);
%    points = size(data_cell{k},1);
%    plot(linspace(1,points,points)',data_cell{k}(:,8));hold on;
end

%% Acceleration:
for k = 1:1:size(data_cell,2)
        for j = 1:1:(size(data_cell{k},1)-2)
        tf = data_cell{k}(j+1,1);
        ti = data_cell{k}(j,1);
        vf = data_cell{k}(j+1,8);
        vi = data_cell{k}(j,8);
        dv = 1000000000*(vf-vi)/(tf-ti);
        if isnan(dv) 
            continue
        end
        a(j) = dv;
        data_cell{k}(j,9) = dv;
        end
    % All data
%    figure(k);
%    points = size(data_cell{k},1);
%    plot(linspace(1,points,points)',data_cell{k}(:,9));hold on; 
    % Average
    am(k)=mean(a);
    se=((a-am(k)).^2);
    sse = sum(se);
    sd(k)=sqrt(sse/size(se,2));
end
figure();
errorbar(am,sd,'-');
xlabel('Participant'); ylabel('Acceleration (m/s^2)');
title('Average Acceleration');

x0=600; y0=100; width=450; height=200;
set(gcf,'units','points','position',[x0,y0,width,height])

