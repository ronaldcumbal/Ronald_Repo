function f = obsForce(d,a)
    A = 1.0;    % Magnitud
    B = 2.0;    % Distance
    C = 0.0; 
    off = 0.0;
    lq =0.0;
    %f = -A*(exp((C-d)/B))+off; % Repulsive force
    f = -A*(exp((C-d)/B))*(lq+(1-lq));
end
 
% function [fx,fy] = pedForce(xped,yped)
% Ax = 1.5; 
% Bx = 1.2; 
% Cx = 0.0; 
% offx = 0.0;
% fx = -Ax*(exp((Cx-abs(xped))/Bx))+offx; % Repulsive force
% Ay = 1.5; 
% By = 1.2; 
% Cy = 0.0; 
% offy = 0.0;
% fy = -Ay*(exp((Cy-abs(yped))/By))+offy; % Repulsive force
% end
