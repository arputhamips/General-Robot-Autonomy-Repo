function expected = expectedSensorReading(position, map)
% Computes the expected sensor readings at a given grid position
% INPUT:
%   position - [x, y] grid cell location
%   map - Wall coordinates [x1, y1, x2, y2]
% OUTPUT:
%   expected - [North, East, South, West] expected distances

x = position(1);
y = position(2);

expected = [3, 3, 3, 3]; % Initialize with max range

for i = 1:size(map, 1)
    x1 = map(i, 1); y1 = map(i, 2);
    x2 = map(i, 3); y2 = map(i, 4);

    % Check North direction
    if y1 > y && y2 > y && x1 <= x && x2 >= x
        expected(1) = min(expected(1), y1 - y);
    end

    % Check South direction
    if y1 < y && y2 < y && x1 <= x && x2 >= x
        expected(3) = min(expected(3), y - y1);
    end

    % Check East direction
    if x1 > x && x2 > x && y1 <= y && y2 >= y
        expected(2) = min(expected(2), x1 - x);
    end

    % Check West direction
    if x1 < x && x2 < x && y1 <= y && y2 >= y
        expected(4) = min(expected(4), x - x1);
    end
end

end