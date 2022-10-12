function [xrep,yrep] = FrepCal(x, y, xo, yo, c2, disTh)
leftPart = c2 * (1 / DistanceCal(x, y, xo, yo) - 1 / disTh) * (1 / DistanceCal(x, y, xo, yo) .^ 2);
xrep = leftPart * (x - xo) / sqrt((x - xo) ^ 2 + (y - yo) ^ 2);
yrep = leftPart * (y - yo) / sqrt((x - xo) ^ 2 + (y - yo) ^ 2);


end