clear
close all
clc

% Workspace Size
xlim([0 100])
ylim([0 100])

%goal = [80, 20];
goal = [80, 20];


c1 = 1;
x1 = linspace(0, 100, 101);
y1 = x1';
z1 = 0.5 * c1 * ((x1 - goal(1)).^2 + (y1 - goal(2)).^2);
surf(x1,y1,z1);
figure;
contour(x1, y1, z1, 100);
figure;

c2 = 110000;
% c2 = 100000;
maxval = 3000;

disTh = 60;

oCenter = [40, 60];

x2 = linspace(0, 100, 101);
y2 = x2';

z2 = (0.5 * c2 * (1 ./ (DistanceCal(x2, y2, oCenter(1), oCenter(2))) ...
    - 1/disTh).^2) .* (DistanceCal(x2, y2, oCenter(1), oCenter(2)) <= disTh);
z2(z2>maxval) = maxval;
surf(x2,y2,z2);
figure;
contour(x2, y2, z2, 70);
figure;

z = z1 + z2;
surf(x1, y1, z);
figure;
contour(x1, y1, z, 100);
figure;

start = [10, 80];

dt = 0.1;

x = start(1);
y = start(2);
xg = goal(1);
yg = goal(2);
xo = oCenter(1);
yo = oCenter(2);

xData = [];
yData = [];
theta = 0;

while (abs(x - xg) > 1 || (abs(y - yg) > 1))
    [xatt,yatt] = FattCal(x, y, xg, yg, c1);
    [xrep,yrep] = FrepCal(x, y, xo, yo, c2, disTh);

    xdiv = xatt + xrep;
    ydiv = yatt + yrep;

    % max 5
    theta = atan2(ydiv, xdiv);
    theta = atan2(sin(theta), cos(theta));


    vel = sqrt(ydiv ^ 2 + xdiv ^ 2);
    if vel > 5
        vel = 5;
    end

    xData = [xData, x];
    yData = [yData, y];
    x = x + vel * cos(theta) * dt;
    y = y + vel * sin(theta) * dt;
    plot(xData,yData,'-');
    xlim([0 100])
    ylim([0 100])
%     pause(0.01)


end

plot(xData,yData,'-', xo, yo, '+');
xlim([0 100])
ylim([0 100])