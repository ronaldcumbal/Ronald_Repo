clear
clc
clf
%% Pedestrian Force
dmin=-4; dmax=4; ds=1.0;
points=100;
x=linspace(dmin,dmax,points);% (m) Distance between agents or features
y=linspace(dmin,dmax,points);% (m) Distance between agents or features
Ax=0.04; Bx=1.5; Cx=0.0; offset=0.0;
for n=1:1:length(x)
    pedx(n)=-Ax*(exp((abs(x(n))-Cx)/Bx))+offset; % Repulsive force
end;
Ay=0.01; By=1.0; Cy=0.0; offset=0.0;
for n=1:1:length(y)
    pedy(n)=-Ay*(exp((abs(y(n))-Cy)/By))+offset; % Repulsive force
end;

%% Partner Force
Ax=0.01;  Bx=1.0; Cx=1.5; 
for n=1:1:length(x)
    parx(n)=Ax*(exp((abs(x(n))+1.35-Cx)/Bx)); % Attractive force
end;
for n=1:1:length(y)
    if y(n)<ds
        Ay=0.01;  By=1.0; Cy=0.0; offset=0.0; 
        pary(n)=-Ay*(exp((Cy-abs(y(n)))/By))+offset; % Repulsive force
    else
        Ay=0.01;  By=1.0; Cy=0.0; offset=0.0;
        pary(n)=Ay*(exp((abs(y(n))-Cy)/By))+offset; % Attractive force
    end;
end;

%% Goal Force
goalx=0.7*ones(1,length(x));
goaly=0.0*ones(1,length(y));

%% Total Force
totalx = pedx + goalx;
totaly = pedy + goaly;

figure(1);plot(x,totalx,'.k',x,goalx,'--g',x,pedx,'-r',x,parx,'-b');
xlabel('X');ylabel('Force');legend('total','goal','ped','par');
axis([0 4 -1 1]);
figure(2);plot(y,totaly,'.k',y,goaly,'--g',y,pedy,'-r',y,pary,'-b');
xlabel('Y');ylabel('Force');legend('total','goal','ped','par');
axis([0 4 -1 1]);
%%
%vd=2; va=1; relax_time=2;
%for n=1:1:length(x)
%    for m=1:1:length(y)
%        goal(n,m)=vxy(n,m)/relax_time;
%    end;
%end;
%figure();surf(X,Y,goal);view(az,el);ylabel('Y');xlabel('X');zlabel('Force');colorbar;
