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

x(1) = 0;
y(1) = 0;

% Initial Orientation 
theta(1) = 0;

robot = TriangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])

destX = linspace(20, 20, 200);
destY = linspace(20, 120, 200);

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
vel = 0;

dis = [];

setDis = 10;

for destIdx = 1:200
%     vd = 100;
    theta = atan2(destY(destIdx) - yp, destX(destIdx) - xp);
    measured_value = sqrt((destX(destIdx) - xp)^2 + (destY(destIdx) - yp)^2);
    error = measured_value - setDis;
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt ;
    output = Kp*error + Ki*integral + Kd*derivative;
    vel = output + 0.99*vel;
%     velData = [velData, vel];
    previous_error = error;

    dis = [dis, sqrt((destX(destIdx) - xp)^2 + (destY(destIdx) - yp)^2)];
    
    xp = xp + vel * cos(theta) * dt;
    xData = [xData, xp];
    yp = yp + vel * sin(theta) * dt;
    yData = [yData, yp];

%     i = i + 1;
%     time = [time, i];
    

    robot = TriangularRobot(xp,yp,theta(1));
    plot(robot(:,1),robot(:,2),'-',xData,yData,'-');
    
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
