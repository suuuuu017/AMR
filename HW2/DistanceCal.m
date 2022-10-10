function dis = DistanceCal(x, y, ox, oy)
    dis = sqrt((x - ox) .^ 2 + (y - oy) .^ 2);
end