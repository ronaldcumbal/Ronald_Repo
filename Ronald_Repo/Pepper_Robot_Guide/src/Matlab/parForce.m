function f = parForce(d,a)
if d>=0.6
    % Attractive
    A = 0.0008;  
    B = 0.145; 
    C = 0.0; 
    off = 0.0;
    lq=1; 
    f=A*(exp((d-C)/B))+off;
else
    % Repulsive
    A=0.6*1.5;
    B=0.12*1.5; 
    C=0.0; 
    off=0.0; 
    lq=1;
    f=-A*(exp((C-d)/B))+off;
end

% function [fx,fy] = parForce(xpar,ypar)
% Ax = 0.5;  
% Bx = 1.5; 
% Cx = 0.0; 
% offx = 0.0;
% fx=Ax*(exp((abs(xpar)-Cx)/Bx))+offx; % Attractive forceend
% 
% if abs(ypar)<=1.0
%     Ay=0.1;  
%     By=1.5; 
%     Cy=0.0; 
%     offy=0.0; 
%     fy=-Ay*(exp((Cy-abs(ypar))/By))+offy; % Repulsive force
% else
%     Ay=0.15;  
%     By=1.5; 
%     Cy=0.0; 
%     offy=0.0;
%     fy=Ay*(exp((abs(ypar)-Cy)/By))+offy; % Attractive force
% end
% end

