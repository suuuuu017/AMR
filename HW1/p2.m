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

x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta(1) = 0;
%TODO: change the theta to random angle
% theta(1) = 2*pi*rand;

% Build Robot Model
robot = TriangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
% hold on;
xlim([0 200])
ylim([0 200])
    
x(2) = x(1) + 20;
y(2) = y(1) + 30;
theta(2) = theta(1) + pi/6;

robot = TriangularRobot(x(2),y(2),theta(2));
plot(robot(:,1),robot(:,2),'-',x,y,'-');
% hold on;
xlim([0 200])
ylim([0 200])
%----------------position calculated from 1 ------------------

xg = 10;
yg = 120;

vel = 5;

nstep = 100;


dt = 0.05;
xp = 100;
yp = 120;
ang = 0;

xData = [];
yData = [];
time = [];

velData = [];

vd = 0;
%TODO: change the bound to 0.3, add constant
while (abs(xp - xg) > 0.01) || (abs(yp - yg) > 0.01)
    
    velData = [velData, vd];
    vd = sqrt((xg - xp)^2 + (yg - yp)^2);
    if vd > 5
        vd = 5;
    end
    
    error = atan2(yg - yp, xg - xp) - ang;
    errorPri = atan2(sin(error), cos(error));
    if errorPri > pi/4
        errorPri = pi/4;
    end
    ang = ang + errorPri * dt;
    xp = xp + vd * cos(ang) * dt;
    xData = [xData, xp];
    yp = yp + vd * sin(ang) * dt;
    yData = [yData, yp];

    i = i + 1;
    time = [time, i];

    robot = TriangularRobot(xp,yp,ang);
    plot(robot(:,1),robot(:,2),'-',xData,yData,'-');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end    

figure;

plot(velData)
title('vel');
% figure;

% plot(x);
% title('x position');
% figure;
% 
% plot(y);
% title('y position');
