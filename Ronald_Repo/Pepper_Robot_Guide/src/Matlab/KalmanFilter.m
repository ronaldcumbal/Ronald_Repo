function [ x_cor,P_cor ] = KalmanFilter(x_old,P_old,z)
% KALMANTRACK 
% x = State vector of the system, containing the position and speed of
% the object in x,y-direction.
% x_old = state vector at previous step
% z = the positions measured at current step
% P = covariance matrix

%% Noise matrices:
R = eye(2); % measurement noise
Q = eye(4); % process noise

%% System matrices:
A = [1 0 1 0;
    0 1 0 1;
    0 0 1 0;
    0 0 0 1];

H = [1 0 0 0;
    0 1 0 0];

I = eye(4);
%% Prediction equations:
x_pred = A* x_old;
P_pred = A* P_old * A.' + Q;
%% Update equations/correction Step:
%S = H* P_pred *H.' + R
K     = P_pred * H.'/(H * P_pred * H.' + R);  % K =P_pred*H.'*S^(-1)
x_cor = x_pred + K * (z - H * x_pred);
P_cor = (I - K * H) * P_pred;

end