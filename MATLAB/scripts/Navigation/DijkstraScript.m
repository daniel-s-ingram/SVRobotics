map = false(50);
obstacles = rand(50) > 0.6;
map(obstacles) = true;

start_pos = [5, 5];
goal_pos = [25, 25];
map(goal_pos) = false;

Dijkstra(map, start_pos, goal_pos);