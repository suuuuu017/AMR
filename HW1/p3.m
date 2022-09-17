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

x(1) = 10;
y(1) = 10;

% Initial Orientation 
theta(1) = 0;

robot = TriangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])

destX = [20, 50, 190];
destY = [20, 70, 180];

xp = x(1);
yp = y(1);

setpoint = 3;

i = 1;

Kp = 1.5;
Ki = 0.1;
Kd = 0.001;

dt = 0.1;
integral = 0;
error = 0;
previous_error = 0;

destIdx = 2;

velData = [];
time = [];

xData = [];
vel = 0;

for destIdx = 1:3
    while (xp < destX(destIdx) - 0.1) && (yp < destY(destIdx) - 0.1)
        vd = sqrt((destX(destIdx) - xp)^2 + (destY(destIdx) - yp)^2);
        measured_value = vel;
    
        error = setpoint - measured_value;
        integral = integral + error*dt;
        derivative = (error - previous_error)/dt ;
        output = Kp*error + Ki*integral + Kd*derivative;
        vel = 0.99*vel + output;
        velData = [velData, vel];
        previous_error = error;
    
        theta = atan2(destY(destIdx) - yp, destX(destIdx) - xp);
        xp = xp + vel * cos(theta) * dt;
        xData = [xData, xp];
        yp = yp + vel * sin(theta) * dt;
    
        i = i + 1;
        time = [time, i];
    
        robot = TriangularRobot(xp,yp,theta(1));
        plot(robot(:,1),robot(:,2),'-',xp,yp,'-');
        xlim([0 200])
        ylim([0 200])
        pause(0.01)
        
    end    
end
plot(time, velData);
figure;
plot(time, xData);
