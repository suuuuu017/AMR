function [xatt,yatt] = FattCal(x, y, xg, yg, c1)
xatt = -1 * c1 * (x - xg);
yatt = -1 * c1 * (y - yg);
end