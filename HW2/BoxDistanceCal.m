function dis = BoxDistanceCal(x, y, oxmin, oxmax, oymin, oymax)
    xdis = max(oxmin - x, x - oxmax);
    ydis = max(oymin - y, y - oymax);
    xdis = max(xdis, zeros(1, size(x, 2)));
    ydis = max(ydis, zeros(size(y, 1), 1));
    dis = sqrt(xdis .^ 2 + ydis .^ 2);
end