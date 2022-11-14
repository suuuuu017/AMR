clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 100])
ylim([0 100])


%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

x(1) = 10;
y(1) = 10;

% Initial Orientation 
theta = 2*pi*rand;

robot = TriangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])

destX = 50;
destY = 40;

xp = x(1);
yp = y(1);

setpoint = 3;

i = 1;

Kp = 0.2;
Ki = 0.02;
Kd = 0.01;

dt = 0.1;
integral = 0;
error = 0;
previous_error = 0;


velData = [];
time = [];

xData = [];
yData = [];
vel = 0;
theta = 0;

estXData = [];
estYData = [];

mesXData = [];
mesYData = [];

mes1X = [];
mes1Y = [];
mes2X = [];
mes2Y = [];

kp = 1;

% Kalman filter constant
% F = [1 dt; 0 0];
F = [1 0; 0 0];
% F = [1 0; 0 1];
% B = [dt 0; 0 dt];

R1 = [normrnd(0,6) 0; 0 normrnd(0,6)];
R2 = [normrnd(0,4) 0; 0 normrnd(0,4)];
% R = [normrnd(0, 5); normrnd(0, 5)];
% R = [0; 0];

% Q = [normrnd(0,1)*dt 0; 0 normrnd(0,1)*dt*dt];
% Q = [0 0; 0 0];

% TODO: initial mearsurement?
H = [1 0; 0 1];
X = [10; 0];
Y = [10; 0];
I = [1 0; 0 1];
% Zx = H * X;
% Zy = H * Y;

estX = [10; 0];
estY = [10; 0];

B = [dt; 1];


pX = [0 0; 0 0];
pY = [0 0; 0 0];

dst = [];
linVel = [];

while (abs(xp - destX) > 0.3) || (abs(yp - destY) > 0.3)
    
    R = [normrnd(0, 5); normrnd(0, 5)];
    R1 = [normrnd(0, 4); normrnd(0, 4)];
    R2 = [normrnd(0, 6); normrnd(0, 6)];
    R = (R1 + R2) / 2;
    Q = [1 * normrnd(0,1)*dt^2 1 * normrnd(0,1)*dt^2 ;...
         1 * normrnd(0,1)*dt^2 normrnd(0,1)*dt^2];

    vd = sqrt((destX - xp)^2 + (destY - yp)^2);
    measured_value = vel;
    velData = [velData, vel];
    error = setpoint - measured_value;
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt ;
    output = Kp*error + Ki*integral + Kd*derivative;
    vel = 0.99*vel + output;
%     if vel > 5
%         vel = 5;
%     end
    previous_error = error;
    steer = kp*(atan2(destY - yp, destX - xp) - theta);
    steering = atan2(sin(steer), cos(steer));
%     if steering > pi/4
%         steering = pi/4;
%     end
    theta = theta + steering * dt;
    xp = xp + vel * cos(theta) * dt;
    xData = [xData, xp];
    yp = yp + vel * sin(theta) * dt;
    yData = [yData, yp];

    i = i + 1;
    time = [time, i];
    
    uX = vel * cos(theta) + normrnd(0,1) *dt*dt;
    uY = vel * sin(theta) + normrnd(0,1) *dt*dt;

    trueX = [xp; vel * cos(theta)];
    trueY = [yp; vel * sin(theta)];
    
    % Predict 
    estX = F * estX + B * uX;
    estY = F * estY + B * uY;

    pX = F * pX * F' + Q;
    pY = F * pY * F' + Q;

    % Update
%     mesX = H * estX + 
%     mesY = H * estY + 

    % ground truth + noise
    Zx = trueX + R;
    Zy = trueY + R;
%     Zx = (trueX + R1 + R2 + trueX)./2;
%     Zy = (trueY + R1 + R2 + trueY)./2;

    mesXData = [mesXData, Zx(1)];
    mesYData = [mesYData, Zy(1)];

    mes1X = [mes1X, trueX(1) + R1(1)];
    mes1Y = [mes1Y, trueY(1) + R1(1)];
    mes2X = [mes2X, trueX(1) + R2(1)];
    mes2Y = [mes2Y, trueY(1) + R2(1)];

    Yx = Zx - H * estX;
    Yy = Zy - H * estY;

    Sx = H * pX * H' + R;
    Sy = H * pY * H' + R;

    Kx = pX * H' * inv(Sx);
    Ky = pY * H' * inv(Sy);

    % Updated state and convariance
    estX = estX + Kx * Yx;
    estY = estY + Ky * Yy;

    estXData = [estXData, estX(1)];
    estYData = [estYData, estY(1)];

    dst = [dst, sqrt((destX - estX(1))^2 + (destY - estY(1))^2)];
    linVel = [linVel, sqrt(estX(2) ^ 2 + estY(2) ^ 2)];

    pX = (I - Kx * H) * pX;
    pY = (I - Ky * H) * pY;

    robot = TriangularRobot(xp,yp,theta(1));
%     plot(robot(:,1),robot(:,2),'-',xData,yData,'-', estXData, estYData, '.', ...
%         mesXData, mesYData, 'g.');
    plot(estXData, estYData, 'r-', ...
        mes1X, mes1Y, 'g.', mes2X, mes2Y, 'b.');
    pause(0.01)
    
end    

figure;
plot(time, velData);
figure;
% plot(time, xData);
plot(estXData);
figure;
plot(estYData);
figure;
plot(dst);
figure;
plot(linVel);
