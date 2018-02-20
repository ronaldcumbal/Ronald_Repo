A = 0.5;  
B = 1.5; 
C = 0.0; 
off = 0.0;
points = 100;
d=linspace(0,1.0,points)';
zeroline=zeros(points);
threshold = 0.5;
my=linspace(-1,1.0,points)';
mx=threshold*ones(points);
for m=1:1:length(d)
    if d(m)>=threshold
        % Attractive
        A = 0.0008;  
        B = 0.145; 
        C = 0.0; 
        off = 0.0;
        lq=1; 
        f(m,1)=A*(exp((d(m)-C)/B));
    else
        % Repulsive
        A=1.0;
        B=0.12; 
        C=0.0; 
        off=0; 
        lq=1;
        f(m,1)=-A*(exp((C-d(m))/B))+off;
    end
end
figure(1)
grid();
plot(d,f,d,zeroline,mx,my);
axis([-inf inf -1.0 1.0]);
