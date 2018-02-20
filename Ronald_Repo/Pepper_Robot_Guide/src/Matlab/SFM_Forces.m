clear
close all
%% Repulsive Forces
dmin=-3; dmax=-dmin; 
points=100;
x=linspace(dmin,dmax,points);% (m) Distance between agents or features
y=linspace(dmin,dmax,points);% (m) Distance between agents or features
% Plots
az=0; el=90; [X,Y] = meshgrid(x,y);
for m=1:1:length(x)
    for n=1:1:length(y)
    axy(m,n)=atan2(x(n),y(m)); % (rad) Angle between agents or features
    dxy(m,n)=norm([y(m) x(n)]);
    end
end

% Source [18]
Aq=1.25; Bq=0.1; dq=0.2; lq=0.5;
for n=1:1:length(x)
    for m=1:1:length(y)
        fpp18(n,m)=Aq*(exp((dq-dxy(n,m))/Bq))*(lq+(1-lq)*(1+cos(axy(n,m)))/2); % Repulsive force
    end;
end;
%figure();surf(X,Y,fpp18);
%view(az,el);ylabel('Y');xlabel('X');zlabel('Repulsive force');
%title('Magnitud of Person - Pedestrian Force [18]');colorbar;

% Source [10]
Aq=10; Bq=0.34; dq=0.16; lq=1;
for n=1:1:length(x)
    for m=1:1:length(y)
        fpp10(n,m)=Aq*(exp((dq-dxy(n,m))/Bq))*(lq+(1-lq)*(1+cos(axy(n,m)))/2); % Repulsive force
    end;
end;
%figure();surf(X,Y,fpp10);
%view(az,el);ylabel('Y');xlabel('X');zlabel('Repulsive force');
%title('Magnitud of Person - Pedestrian Force [10]');colorbar;

% Source: Robot Companion:
Aq=2.66; Bq=0.9; dq=0.4; lq=0.25;%59
for n=1:1:length(x)
    for m=1:1:length(y)
        fpr(n,m)=Aq*(exp((dq-dxy(n,m))/Bq))*(lq+(1-lq)*(1+cos(axy(n,m)))/2); % Repulsive force
    end;
end;
figure();surf(X,Y,fpr);
view(az,el);ylabel('Y');xlabel('X');zlabel('Repulsive force');
title('Magnitud of Robot - Pedestrian Force');colorbar;

%% Attractive Forces
dmin=-3; dmax=-dmin; 
points=100;
x=linspace(dmin,dmax,points);% (m) Distance between agents or features
y=linspace(dmin,dmax,points);% (m) Distance between agents or features
% Plots
az=0; el=90; [X,Y] = meshgrid(x,y);
for m=1:1:length(x)
    for n=1:1:length(y)
    axy(m,n)=atan2(x(n),y(m)); % (rad) Angle between agents or features
    dxy(m,n)=norm([y(m) x(n)]);
    end
end% Attractive Forces
vd=2;
va=1;
ds=1.2; %(m) Social Distance
% Source [18]
k=2;%%%%%%%%%%%%%%%%%%%%CAHNGE THIS
for n=1:1:length(x)
    for m=1:1:length(y)
        fa18(n,m)=abs(k*(vd-va)*(ds-dxy(n,m)));
    end;
end;
%figure();surf(X,Y,fa18);
%view(az,el);ylabel('Y');xlabel('X');zlabel('Attractive force');
%title('Magnitud of Person - Pedestrian Force');colorbar;

% Source [10]
k=4.9;%%%%%%%%%%%%%%%%%%%%CAHNGE THIS
for n=1:1:length(x)
    for m=1:1:length(y)
        fa10(n,m)=abs(k*(vd-va)*(ds-dxy(n,m)));
    end;
end;
%figure();surf(X,Y,fa10);
%view(az,el);ylabel('Y');xlabel('X');zlabel('Attractive force');
%title('Magnitud of Person - Pedestrian Force');colorbar;

% Source Robot-Person
k=1;%%%%%%%%%%%%%%%%%%%%CAHNGE THIS
for n=1:1:length(x)
    for m=1:1:length(y)
        fapr(n,m)=abs(k*(vd-va)*(ds-dxy(n,m)));
    end;
end;
figure();surf(X,Y,fapr);
view(az,el);ylabel('Y');xlabel('X');zlabel('Attractive force');
title('Magnitud of Robot - Guided Person Force');colorbar;