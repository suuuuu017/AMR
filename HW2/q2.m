clear
close all
clc

% Workspace Size
xlim([0 100])
ylim([0 100])

goal = [80, 20];

c1 = 1;

x1 = linspace(0, 100, 101);
y1 = x1';
z1 = 0.5 * c1 * ((x1 - goal(1)).^2 + (y1 - goal(2)).^2);
% surf(x1,y1,z1);
% figure;
% contour(x1, y1, z1, 100);
% figure;

c2 = 20000;

disTh = 30;

oCenter = [40, 60];

x2 = linspace(0, 100, 101);
y2 = x2';

z2 = (0.5 * c2 * (1 ./ (DistanceCal(x2, y2, oCenter(1), oCenter(2))) - 1/disTh).^2) .* (DistanceCal(x2, y2, oCenter(1), oCenter(2)) <= disTh);
% surf(x2,y2,z2);
% figure;
% contour(x2, y2, z2, 100);
% figure;

z = z1 + z2;
% surf(x1, y1, z);
% figure;
% contour(x1, y1, z, 100);

start = [10, 80];

dt = 0.1;

x = start(1);
y = start(2);
ang = 0;
xg = goal(1);
yg = goal(2);
xo = oCenter(1);
yo = oCenter(2);

xData = [];
yData = [];

while (abs(x - xg) > 0.1) || (abs(y - yg) > 0.1)
    [xatt,yatt] = FattCal(x, y, xg, yg, c1);
    [xrep,yrep] = FrepCal(x, y, xo, yo, c2, disTh);

    % TODO: why is xatt + xrep
    xdiv = xatt + xrep;
    ydiv = yatt + yrep;

    % max 5
    theta = atan2(ydiv, xdiv);
    if theta > pi / 4
        theta = pi / 4;
    end
    vel = sqrt(ydiv ^ 2 + xdiv ^ 2);
    if vel > 5
        vel = 5;
    end

    xData = [xData, x];
    yData = [yData, y];
    error = theta - ang;
    error = atan2(sin(error), cos(error));
    ang = ang + error * dt;
    x = x + vel * cos(ang) * dt;
    y = y + vel * sin(ang) * dt;

    robot = TriangularRobot(x,y,ang);
    plot(robot(:,1),robot(:,2),'-',xData,yData,'-');
    xlim([0 100])
    ylim([0 100])
    pause(0.01)

end

plot(xData,yData,'-');
xlim([0 100])
ylim([0 100])



