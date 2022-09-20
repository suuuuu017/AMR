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

theta(1) = 2*pi*rand;

% Build Robot Model
robot = TriangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
hold on;
xlim([0 200])
ylim([0 200])
    
x(2) = x(1) + 20;
y(2) = y(1) + 30;
theta(2) = theta(1) + pi/6;

robot = TriangularRobot(x(2),y(2),theta(2));
plot(robot(:,1),robot(:,2),'-',x,y,'-');
hold off;
xlim([0 200])
ylim([0 200])
    

