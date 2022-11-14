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

dt = 0.3;
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

estX = 10;
estY = 10;

B = [dt; 1];


pX = [0 0; 0 0];
pY = [0 0; 0 0];

xSig = 10;
ySig = 10;


while (abs(xp - destX) > 0.3) || (abs(yp - destY) > 0.3)
    
    R = [normrnd(0, 5); normrnd(0, 5)];
    R1 = [normrnd(0, 4); normrnd(0, 4)];
    R2 = [normrnd(0, 6); normrnd(0, 6)];
    R = (R1 + R2) / 2;
    Q = [1 / 4 * normrnd(0,1)*dt^4 1/2 * normrnd(0,1)*dt^3 ;...
         1/2 * normrnd(0,1)*dt^3 normrnd(0,1)*dt^2];

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

    mesX = xp + R(1);
    mesY = yp + R(1);

    mesXData = [mesXData, mesX];
    mesYData = [mesYData, mesY];
    
    % Predict 
    estX = (estX * xSig + mesX * R(1)) / (xSig + R(1));
    xSig = (1 / (1/xSig + 1/R(1)));

    estY = (estY * ySig + mesY * R(1)) / (ySig + R(1));
    ySig = (1 / (1/ySig + 1/R(1)));

    estXData = [estXData, estX];
    estYData = [estYData, estY];


    robot = TriangularRobot(xp,yp,theta(1));
    plot(robot(:,1),robot(:,2),'-', estXData, estYData, '.', ...
        mesXData, mesYData, 'g.');
%     plot(robot(:,1),robot(:,2),'-', estXData, estYData, '.');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end    

figure;
plot(estXData, estYData, '-');
% figure;
% plot(time, xData);
