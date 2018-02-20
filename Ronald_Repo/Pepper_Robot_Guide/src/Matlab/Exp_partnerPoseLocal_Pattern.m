clear all;
close all;
clc;
% Number of participants
num_people = 13;
% Add csv Files
folder = '/home/ronald/pepper-navigation/catkin_pepper/src/Experiment';
for i = 1:1:num_people
    baseFileName = sprintf('/user%d/user%d_bag_partnerLocalPose.csv',i,i);
    fullFileName = fullfile(folder, baseFileName);
    if exist(fullFileName, 'file')
        data = load(fullFileName);
        data_cell{i} = data;
    end
end

% Circles:
i = 0;
for k = 0:0.01:6.28
    i = i+1;
    rx(i) = cos(k);
    ry(i) = sin(k);
end
lx=linspace(-2,2,100); ly=zeros(size(lx));

%% Plot ALL recorded Poses:
for k = 1:1:size(data_cell,2)
    for j = 1:1:size(data_cell{k},1)
        x(j) = data_cell{k}(j,2); % x
        y(j) = data_cell{k}(j,3); % y
    end
%    figure(k);
    figure(1);
    plot(x(1:end),y(1:end),'.');hold on;
    % Graph Lines
    plot(lx,ly,'k--');hold on;
    plot(ly,lx,'k--');hold on;
    % Pepper
    plot([0],[0],'ok','MarkerSize',115/4,'MarkerFaceColor',[0.95 0.95 0.95]);hold on;
    % Proxemic Circles
    plot(0.45*rx,0.45*ry,'k-.');hold on; %Intimate Space
    plot(1.2*rx,1.2*ry,'k-.');hold on; %Personal Space
    plot(3.6*rx,3.6*ry,'k-.');hold on; %Social Space

    axis('equal'); axis([-1.25 0.5 -0.5 1.25]);
    view([-90 90])
    %grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Recorded Partner Poses');
end
x0=600; y0=100; width=300; height=300;
set(gcf,'units','points','position',[x0,y0,width,height])

%% Plot AVERAGE INITIAL Poses:
max = 5;
for k = 1:1:size(data_cell,2)
    for j = 1:1:size(data_cell{k},1)
        x(j) = data_cell{k}(j,2); % x
        y(j) = data_cell{k}(j,3); % y
    end
    figure(2);
    ax=mean(x(1:max));
    ay=mean(y(1:max));
    plot(ax,ay,'o');hold on;
    % Graph Lines
    plot(lx,ly,'k--');hold on;
    plot(ly,lx,'k--');hold on;
    % Pepper
    plot([0],[0],'ok','MarkerSize',115/4,'MarkerFaceColor',[0.95 0.95 0.95]);hold on;
    % Proxemic Circles
    plot(0.45*rx,0.45*ry,'k-.');hold on; %Intimate Space
    plot(1.2*rx,1.2*ry,'k-.');hold on; %Personal Space
    plot(3.6*rx,3.6*ry,'k-.');hold on; %Social Space

    axis('equal'); axis([-0.5 0.5 -0.5 1]);
    view([-90 90])
    %grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Average Initial Partner Pose');
end
x0=600; y0=100; width=360; height=220;
set(gcf,'units','points','position',[x0,y0,width,height])

%% Plot AVERAGE ALL Poses:
for k = 1:1:size(data_cell,2)
    for j = 1:1:size(data_cell{k},1)
        x(j) = data_cell{k}(j,2); % x
        y(j) = data_cell{k}(j,3); % y
    end
    figure(3);
    ax=mean(x);
    ay=mean(y);
    plot(ax,ay,'o');hold on;
    % Graph Lines
    plot(lx,ly,'k--');hold on;
    plot(ly,lx,'k--');hold on;
    % Pepper
    plot([0],[0],'ok','MarkerSize',115/4,'MarkerFaceColor',[0.95 0.95 0.95]);hold on;
    % Proxemic Circles
    plot(0.45*rx,0.45*ry,'k-.');hold on; %Intimate Space
    plot(1.2*rx,1.2*ry,'k-.');hold on; %Personal Space
    plot(3.6*rx,3.6*ry,'k-.');hold on; %Social Space

    axis('equal'); axis([-0.5 0.5 -0.5 1]);
    view([-90 90])
    %grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Average Partner Pose');
end
x0=600; y0=100; width=360; height=220;
set(gcf,'units','points','position',[x0,y0,width,height])

%% Plot Distance:
for k = 1:1:size(data_cell,2)
    for j = 1:1:size(data_cell{k},1)
        x = data_cell{k}(j,2); % x
        y = data_cell{k}(j,3); % y
        d(j) =sqrt(x.^2 + y.^2);
    end
    dm(k)=mean(d);
    se=((d-dm(k)).^2);
    sse = sum(se);
    sd(k)=sqrt(sse/size(se,2));
end
figure();
errorbar(dm,sd,'-');
xlabel('Participant'); ylabel('Distance (m)');
title('Average Partner Distance');
x0=600; y0=100; width=400; height=200;
set(gcf,'units','points','position',[x0,y0,width,height])