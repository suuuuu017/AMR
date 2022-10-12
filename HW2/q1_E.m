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

c2 = 10000;
% c2 = 100000;
maxval = 5000;

disTh = 40;

oCenter = [40, 60];

x2 = linspace(0, 100, 101);
y2 = x2';

z2 = (0.5 * c2 * (1 ./ (DistanceCal(x2, y2, oCenter(1), oCenter(2))) ...
    - 1/disTh).^2) .* (DistanceCal(x2, y2, oCenter(1), oCenter(2)) <= disTh);
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

x = start(1);
y = start(2);
xg = goal(1);
yg = goal(2);
xo = oCenter(1);
yo = oCenter(2);

xData = [];
yData = [];
theta = 0;

while (abs(x - xg) > 2 || (abs(y - yg) > 2))
    xData = [xData, x];
    yData = [yData, y];
    currE = z2(x,y);
    surr = [z2(x+1, y), z2(x-1, y),z2(x, y+1),z2(x, y-1),...
        z2(x+1, y+1),z2(x+1, y-1),z2(x-1, y+1),z2(x-1, y-1)];
    minSurr = min(surr);
    if minSurr == surr(1)
        x = x+1;
        y = y;
    elseif minSurr == surr(2)
        x = x-1;
        y = y;
    elseif minSurr == surr(3)
        x = x;
        y = y+1;
    elseif minSurr == surr(4)
        x = x;
        y = y-1;
    elseif minSurr == surr(5)
        x = x+1;
        y = y+1;
    elseif minSurr == surr(6)
        x = x+1;
        y = y-1;
    elseif minSurr == surr(7)
        x = x-1;
        y = y+1;
    elseif minSurr == surr(8)
        x = x-1;
        y = y-1;
    end
end

plot(xData,yData,'-');
xlim([0 100])
ylim([0 100])