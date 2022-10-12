function [xrep,yrep] = FrepCalBox(x, y, oxmin, oxmax, oymin, oymax, c2, disTh)
leftPart = c2 * (1 / BoxDistanceCal(x, y, oxmin, oxmax, oymin, oymax) - 1 ./ disTh) ...
    * (1 / BoxDistanceCal(x, y, oxmin, oxmax, oymin, oymax) ^ 2);
xvec = max(0.00001, x - oxmax);
xvec = max(xvec, oxmin - x);
yvec = max(0.00001, y - oymax);
yvec = max(yvec, oymin - y);
xrep = leftPart * xvec / sqrt(xvec ^ 2 + yvec ^ 2);
yrep = leftPart * yvec / sqrt(xvec ^ 2 + yvec ^ 2);
end