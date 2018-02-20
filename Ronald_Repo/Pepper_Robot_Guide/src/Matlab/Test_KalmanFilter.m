clear
clc
clf

%% Initial conditions
ds = 1.0;
detect_ang = 1.75;
% Goal
goalx = 3.0;
goaly = 0.0;
% Pedestrian
pedx = 3.5;
pedy = 2.5;
% Partner
parx = 0.0;
pary = -1.0;
% Pepper
pepperx = 0;
peppery = 0;

figure(1);
plot(pepperx,peppery,'ok','MarkerSize',115/5); hold on;
plot(goalx,goaly,'ob','MarkerSize',10); hold on;
plot(pedx,pedy,'or','MarkerSize',115/4); hold on;
plot(parx,pary,'og','MarkerSize',115/4); hold on;
xlabel('X');ylabel('Y');
axis([-4 4 -4 4]);
grid on;

%% Kalman filter 
% Noise matrices:
R = eye(2); % measurement noise
Q = eye(4); % process noise

% System matrices:
A = [1 0 1 0;
    0 1 0 1;
    0 0 1 0;
    0 0 0 1];
H = [1 0 0 0;
    0 1 0 0];
I = eye(4);

% Initial values
P_old = eye(4);
x_old=[pedx;pedy;0;0];

while 1
    %% Locations
    [x,y,button] = ginput(1);
    if(button == 2)
        % Goal
        goalx = x;
        goaly = y;
    end
    if(button == 1)
        % Pedestrian
        pedx = x;
        pedy = y;
    end
    if(button == 3)
        % Partner
        parx = x;
        pary = y;
    end
    
    %% Kalman Filter
    % Prediction equations:
    x_pred = A* x_old;
    P_pred = A* P_old * A.' + Q;

    % Update equations/correction Step:
    z = [pedx;pedy];

    %S = H* P_pred *H.' + R
    K = P_pred * H.'/(H * P_pred * H.' + R);  % K =P_pred*H.'*S^(-1)
    x_cor = x_pred + K * (z - H * x_pred);
    P_cor = (I - K * H) * P_pred;
    
    % Save new state as old state for next iteration:
    x_old = x_cor;
    
    %% Forwar prediction
    for m=1:1:10
        x_pred = A*x_cor;
        x_cor = x_pred;
        pedx_pred(m) = x_pred(1); 
        pedy_pred(m) = x_pred(2);
    end

    hold off
    figure(1);
    plot(pepperx,peppery,'ok','MarkerSize',115/5); hold on;
    plot(goalx,goaly,'ob','MarkerSize',10);hold on;
    plot(pedx,pedy,'or','MarkerSize',115/4);hold on;
    plot(pedx_pred,pedy_pred,'or','MarkerSize',5);hold on;
    plot(parx,pary,'og','MarkerSize',115/4);hold on;
    axis([-4 4 -4 4]);
    grid on;
end