clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200])


%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

% x(1) = 40;
% y(1) = 20;

x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta = 2*pi*rand;

robot = TriangularRobot(x,y,theta);

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
figure;

a = 2;
b = 2;
c = -300;

destX = 0:200;
destY = -(a/b)*destX - c/b;
plot(destX, destY, 'g-'); 
xlim([0 200])
ylim([0 200])
xp = x(1);
yp = y(1);

i = 1;

Kp = 1;
Ki = 0.01;
Kd = 1;

dt = 0.1;
integral = 0;
error = 0;
previous_error = 0;

destIdx = 1;

velData = [];
time = [];

xData = [];
yData = [];
vel = 5;

dis = [];

setDis = 10;

step = 1000;

kt = 0.2;
kh = 10;

for i = 1: step
%     vd = 100;
    error = atan2(-1*a, b) - theta;
    controlH = kh*(atan2(sin(error), cos(error)));

    if (a*xp + b*yp + c) / sqrt(a^2 + b^2) >= 0
        controlT = -kt * ((a*xp + b*yp + c) / sqrt(a^2 + b^2) - 10);
    else
        controlT = -kt * ((a*xp + b*yp + c) / sqrt(a^2 + b^2) + 10);
    end
%     controlT = -kt * (abs((a*xp + b*yp + c) / sqrt(a^2 + b^2)) - 10) ;
    steering = controlT + controlH;
    if steering > pi/4
        steering = pi/4;
    end
%     error = measured_value - setDis;
%     integral = integral + error*dt;
%     derivative = (error - previous_error)/dt ;
%     output = Kp*error + Ki*integral + Kd*derivative;
%     vel = output + 0.99*vel;
%     velData = [velData, vel];
%     previous_error = error;

    dis = [dis, abs((a*xp + b*yp + c) / sqrt(a^2 + b^2))];
    theta = theta + steering * dt;
    xp = xp + vel * cos(theta) * dt;
    xData = [xData, xp];
    yp = yp + vel * sin(theta) * dt;
    yData = [yData, yp];

%     i = i + 1;
%     time = [time, i];
    

    robot = TriangularRobot(xp,yp,theta);
    plot(robot(:,1),robot(:,2),'-',xData,yData,'-', destX, destY, 'g-');
%     hold on;
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
            
end
figure;
plot(dis);
% figure;
% plot(destX, destY, '-')
% figure;
% plot(time, xData);
