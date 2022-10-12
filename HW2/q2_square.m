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
surf(x1,y1,z1);
figure;
contour(x1, y1, z1, 100);
figure;

c2 = 120000;

disTh = 30;

oX = [30, 50];
oY = [50, 70];
oXmin = oX(1);
oXmax = oX(2);
oYmin = oY(1);
oYmax = oY(2);
maxval = 5000;

x2 = linspace(0, 100, 101);
y2 = x2';

z2 = (0.5 * c2 * (1 ./ (BoxDistanceCal(x2, y2, oXmin, oXmax, oYmin, oYmax)) - 1/disTh).^2) .* ((BoxDistanceCal(x2, y2, oXmin, oXmax, oYmin, oYmax)) <= disTh);
z2(z2>maxval) = maxval;
surf(x2,y2,z2);
figure;
contour(x2, y2, z2, 500);
figure;

z = z1 + z2;
surf(x1, y1, z);
figure;
contour(x1, y1, z, 100);
figure;

start = [10, 80];

dt = 0.1;
ang = 0;

x = start(1);
y = start(2);
xg = goal(1);
yg = goal(2);
% xo = oCenter(1);
% yo = oCenter(2);


xData = [];
yData = [];

while (abs(x - xg) > 1) || (abs(y - yg) > 1)
    [xatt,yatt] = FattCal(x, y, xg, yg, c1);
    [xrep,yrep] = FrepCalBox(x, y, oXmin, oXmax, oYmin, oYmax, c2, disTh);

    xdiv = xatt + xrep;
    ydiv = yatt + yrep;

    % max 5
    theta = atan2(ydiv, xdiv);
    vel = sqrt(ydiv ^ 2 + xdiv ^ 2);
    if vel > 5
        vel = 5;
    end

    xData = [xData, x];
    yData = [yData, y];

    error = theta - ang;
    errorPri = atan2(sin(error), cos(error));
    if errorPri > pi / 4
        errorPri = pi / 4;
    end
    if errorPri < -1 * pi / 4
        errorPri = -1 * pi / 4;
    end
    ang = ang + errorPri * dt;
    x = x + vel * cos(ang) * dt;
    y = y + vel * sin(ang) * dt;

    robot = TriangularRobot(x,y,ang);
    plot(robot(:,1),robot(:,2),'-',xData,yData,'-');
    xlim([0 100])
    ylim([0 100])

end

plot(xData,yData,'-', [30,50,50,30,30], [50,50,70,70,50], 'r-');
xlim([0 100])
ylim([0 100])