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

xg = 150;
yg = 150;

vel = 5;

nstep = 100;

% xStep = linspace(x(2), xg, nstep);
% yStep = linspace(y(2), yg, nstep);

velx = (xg - x(2)) / nstep;
vely = (yg - y(2)) / nstep;

dt = 0.1;

for i = 2:nstep + 3
    if i == 2
        x(i + 1) = x(i);
        y(i + 1) = y(i);
        velxd(i) = 0;
        velyd(i) = 0;
    elseif i == nstep + 3
        x(i + 1) = x(i);
        y(i + 1) = y(i);
        velxd(i) = 0;
        velyd(i) = 0;
    else
        x(i+1) = x(i) + velx;
        y(i+1) = y(i) + vely;
        velxd(i) = velx;
        velyd(i) = vely;
    end
    
    theta(i+1) = theta(i);
    robot = TriangularRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)

end

figure;

plot(velxd)
title('x velocity graph');
figure;

plot(velyd)
title('y velocity graph');
figure;

plot(x);
title('x position');
figure;

plot(y);
title('y position');
