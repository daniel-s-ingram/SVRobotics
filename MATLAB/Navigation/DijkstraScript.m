n = 10;

map = false(n);
obstacles = rand(n) > 0.65;
map(obstacles) = true;

% load house;
% [m,n] = size(house);

start_pos = [randi(n), randi(n)];
goal_pos = [randi(n), randi(n)];
map(goal_pos) = false;

Dijkstra(map, start_pos, goal_pos);
ASTAR(map, start_pos, goal_pos);

pause;
close all;