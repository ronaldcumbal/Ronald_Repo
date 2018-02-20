clear
clc
clf

%% Initial conditions
ds = 1.0;
detect_ang = 1.75;
% Goal
goalx = 2.0;
goaly = 0.0;
% Pedestrian
pedx = 3.5;
pedy = 2.5;
% Partner
parx = 0.1;
pary = -0.5;
% Pepper
pepperx = 0;
peppery = 0;

%% Plot functions
dmin=-0.8; dmax=0.8; 
points=25;
xp=linspace(dmin,dmax,points);
yp=linspace(dmin,dmax,points);
[Xp,Yp] = meshgrid(xp,yp);

for m=1:1:length(xp)
    for n=1:1:length(yp)
    axy(m,n)=atan2(xp(n),yp(m)); % (rad) Angle between agents or features
    dxy(m,n)=norm([yp(m) xp(n)]);
    end
end

for n=1:1:length(xp)
    for m=1:1:length(yp)
        fped(n,m) = pedForce(dxy(n,m),axy(n,m));
    end
end

for n=1:1:length(xp)
    for m=1:1:length(yp)
        fpar(n,m) = parForce(dxy(n,m),axy(n,m));
    end
end

for n=1:1:length(xp)
    for m=1:1:length(yp)
        fobs(n,m) = obsForce(dxy(n,m),axy(n,m));
    end
end

% figure(2);
% az=60; el= 35;
% subplot(1,2,1);surf(Xp,Yp,fped);view(az,el);
% ylabel('X');xlabel('Y');zlabel('Force');
% az=0; el=90;
% subplot(1,2,2);surf(Xp,Yp,fped);view(az,el);
% ylabel('X');xlabel('Y');zlabel('Force');colorbar;
% set(gcf,'NextPlot','add');axes;
% set(gcf, 'Position', [500, 500, 1200, 400])
% h = title('Pedestrian Force');
% set(gca,'Visible','off');
% set(h,'Visible','on'); 
% %%%%%%%%%
% figure(3);
% az=60; el= 35;
% subplot(1,2,1);surf(Xp,Yp,fpar);view(az,el);
% ylabel('X');xlabel('Y');zlabel('Force');
% az=0; el=90;
% subplot(1,2,2);surf(Xp,Yp,fpar);view(az,el);axis([-0.8 0.8 -0.8 0.8]);
% ylabel('X');xlabel('Y');zlabel('Force');colorbar;
% set(gcf,'NextPlot','add');axes;
% h = title('Partner Force');
% set(gcf, 'Position', [500, 500, 1200, 400])
% set(gca,'Visible','off');
% set(h,'Visible','on'); 

% %%%%%%%%%
% figure(4);
% az=60; el= 35;
% subplot(1,2,1);surf(Xp,Yp,fobs);view(az,el);
% ylabel('X');xlabel('Y');zlabel('Force');
% az=0; el=90;
% subplot(1,2,2);surf(Xp,Yp,fobs);view(az,el);
% ylabel('X');xlabel('Y');zlabel('Force');colorbar;
% set(gcf,'NextPlot','add');axes;
% set(gcf, 'Position', [500, 500, 1200, 400])
% h = title('Obstacle Force');
% set(gca,'Visible','off');
% set(h,'Visible','on'); 

% % Detection Zone
% detectZone_x = linspace(-4,0,10*points);
% detectZone_y = detectZone_x*tan(detect_ang);

figure(1);
% plot(detectZone_x,detectZone_y,'.k','MarkerSize',1);hold on;
% plot(detectZone_x,-detectZone_y,'.k','MarkerSize',1);hold on;
plot(pepperx,peppery,'ok','MarkerSize',115/5); hold on;
plot(goalx,goaly,'ob','MarkerSize',10); hold on;
plot(pedx,pedy,'or','MarkerSize',115/4); hold on;
plot(parx,pary,'og','MarkerSize',115/4); hold on;
xlabel('X');ylabel('Y');
axis([-4 4 -4 4]);
grid on;
k = 0;

% Predifined Trajectories
%goalTraj=   [2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00  2.00; 
%             0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00];
%goalTraj=   [2.00  1.90  1.80  1.70  1.60  1.50  1.40  1.30  1.20  1.10  1.00  2.00  1.90  1.80  1.70  1.60  1.50  1.40  1.30  1.20  1.10  1.00  2.00; 
%             0.00  0.00  0.00  0.05  0.08  0.10  0.13  0.15  0.18  0.20  0.23  0.25  0.28  0.28  0.30  0.28  0.25  0.20  0.16  0.13  0.10  0.05  0.00];
goalTraj=   [3.90  3.80  3.70  3.60  3.50  3.40  3.30  3.20  3.10  3.00  2.90  2.80  2.70  2.60  2.50  2.40  2.30  2.20  2.10  2.00  1.90  1.80  1.70; 
             0.00  0.00  0.00  0.05  0.08  0.10  0.13  0.15  0.18  0.20  0.23  0.25  0.28  0.28  0.30  0.28  0.25  0.20  0.16  0.13  0.10  0.05  0.00];
pedTraj=    [3.00  2.75  2.50  2.25  2.00  1.75  1.50  1.25  1.00  0.75  0.50  0.25  0.00 -0.25 -0.50 -0.75 -1.00 -1.25 -1.50 -1.75 -2.00 -2.25 -2.50; 
             0.30  0.30  0.30  0.35  0.43  0.50  0.60  0.70  0.75  0.80  0.80  0.80  0.75  0.70  0.65  0.60  0.55  0.50  0.45  0.40  0.35  0.30  0.30];
parTraj=    [0.00  0.02  0.03  0.02  0.01  0.00 -0.01  0.00  0.02  0.03  0.02  0.01  0.00 -0.01  0.00  0.01  0.00 -0.01  0.00  0.02  0.03 -0.01  0.00; 
            -0.59 -0.54 -0.51 -0.54 -0.58 -0.58 -0.61 -0.61 -0.58 -0.60 -0.60 -0.59 -0.61 -0.62 -0.61 -0.60 -0.59 -0.58 -0.59 -0.59 -0.58 -0.58 -0.59];
pepperTraj= [0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00; 
             0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00];
     
%% Begin Loop
while 1
    %% Locations
%     [x,y,button] = ginput(1);
    ginput(1);
    k = k+1;
%     if(button == 2)
%         % Goal
%         goalx = x;
%         goaly = y;
%     end
%     if(button == 1)
%         % Pedestrian
%         pedx = x;
%         pedy = y;
%     end
%     if(button == 3)
%         % Partner
%         parx = x;
%         pary = y;
%     end
    
    %% Predifined Trajectories
    goalx = goalTraj(1,k);
    goaly = goalTraj(2,k);
    pedx = pedTraj(1,k);
    pedy = pedTraj(2,k);
    parx = parTraj(1,k);
    pary = parTraj(2,k);
    pepperx = pepperTraj(1,k);
    peppery = pepperTraj(2,k);      
    
    %% Forces
    % Goal
    avrForce = 0.7;
    m = sqrt(goalx*goalx+goaly*goaly);
    Fgoalx = avrForce * goalx/m;  
    Fgoaly = avrForce * goaly/m;  

    % Pedestrian
    m = sqrt((pedx-pepperx)^2+(pedy-peppery)^2);
    a = atan2(pedy,pedx);
    f = pedForce(m,a); 
    if abs(a) > detect_ang
       fx = 0;
       fy = 0;
    end       
    Fpedx = f * pedx/m;
    Fpedy = (f * pedy/m)*abs((1/sin(a)))*1;

    % Partner 
    m = sqrt((parx-pepperx)^2+(pary-peppery)^2);
    a = atan2(pary,parx);
    f = parForce(m,a);
    Fparx = f * parx/m;  
    Fpary = f * pary/m;  

    % Total Force
    Ftotalx = Fgoalx + Fpedx + Fparx;
    Ftotaly = Fgoaly + Fpedy + Fpary;

    %% Plotting
    % Create vectors for plotting
    points=100;
    mag = 1.25;

    Ftotalx = mag*Ftotalx;
    Ftotaly = mag*Ftotaly;
    
    Ftx = linspace(0,mag*Ftotalx,points);
    Fty = linspace(0,mag*Ftotaly,points);

    Fgx = linspace(0,mag*Fgoalx,points);
    Fgy = linspace(0,mag*Fgoaly,points);

    Fpex = linspace(0,mag*Fpedx,points);
    Fpey = linspace(0,mag*Fpedy,points);

    Fpax = linspace(0,2.0*mag*Fparx,points);
    Fpay = linspace(0,2.0*mag*Fpary,points);

    hold off
    figure(1);
%     plot(detectZone_x,detectZone_y,'.k','MarkerSize',1);hold on;
%     plot(detectZone_x,-detectZone_y,'.k','MarkerSize',1);hold on;
    pep = plot(pepperx,peppery,'ok','MarkerSize',115/6, 'MarkerFaceColor',[0.5,0.5,0.5]); hold on;
%    to = plot(Ftotalx,Ftotaly,'*k','MarkerSize',5, 'MarkerFaceColor',[0.5,0.5,0.5]);hold on;
    goa = plot(goalx,goaly,'ok','MarkerSize',8, 'MarkerFaceColor',[1,1,0]);hold on;
    ped = plot(pedx,pedy,'ok','MarkerSize',115/5, 'MarkerFaceColor',[1,0,0]);hold on;
    par = plot(parx,pary,'ok','MarkerSize',115/5, 'MarkerFaceColor',[0,1,0]);hold on;
    Fto = plot(Ftx,Fty,'-b','LineWidth',2);
    Fgo = plot(Fgx,Fgy,'-y', 'LineWidth',2);
    Fpe = plot(Fpex,Fpey,'-r','LineWidth',2);
    Fpa = plot(Fpax,Fpay,'-g','LineWidth',2);
%    wall = plot( )
    title('Force Interaction');  
    lhandle = legend([Fto Fgo Fpe Fpa], ...
        'Total Force','Sub-goal Force','Pedestrian Force','Partner Force');
%        'Pepper','Sub-goal','Pedestrian','Partner',...
    lhandle.Orientation = 'horizontal';
    lhandle.Location = 'southoutside';
    axis('equal');
    axis([-2 4 -1.3 1.3]);
    set(gca,'xtick',[],'ytick',[])
   
end