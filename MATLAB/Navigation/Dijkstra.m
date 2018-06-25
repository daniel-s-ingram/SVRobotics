function Dijkstra(input_map, start_pos, goal_pos)
tic;
cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
        0.5 0.5 0.5];

figure('NumberTitle', 'off', 'Name', 'Dijkstra''s Algorithm');
colormap(cmap);

[nrows, ncols] = size(input_map);

map = zeros(nrows,ncols);
map(~input_map) = 1;
map(input_map)  = 2;

start_node = sub2ind(size(map), start_pos(1), start_pos(2));
goal_node  = sub2ind(size(map), goal_pos(1),  goal_pos(2));

map(start_node) = 5;
map(goal_node)  = 6;

distance_map = Inf(nrows,ncols);
parent_map = zeros(nrows,ncols);

distance_map(start_node) = 0;
distance = 0;

image(1.5, 1.5, map);
grid on;
axis image;
drawnow;
pause;

while true
    map(start_node) = 5;
    map(goal_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    [min_distance, current_node] = min(distance_map(:));
    
    if ((current_node == goal_node) || isinf(min_distance))
        break;
    end;
    
    map(current_node) = 3;
    distance = distance_map(current_node);
    distance_map(current_node) = Inf;
    
    [i, j] = ind2sub(size(distance_map), current_node);
    neighbors = [];
    
    if i - 1 > 0
        neighbors = [neighbors, sub2ind(size(map), i-1, j)];
    end
    
    if i + 1 <= nrows
        neighbors = [neighbors, sub2ind(size(map), i+1, j)];
    end
    
    if j - 1 > 0
        neighbors = [neighbors, sub2ind(size(map), i, j-1)];
    end
    
    if j + 1 <= ncols
        neighbors = [neighbors, sub2ind(size(map), i, j+1)];
    end
    
    for k = neighbors
        if (map(k) == 1) || (map(k) == 6)
            if distance_map(k) > min_distance + 1
                distance_map(k) = distance + 1;
                parent_map(k) = current_node;
                map(k) = 4;
            end
        end
    end
    %pause;
end

if (isinf(distance_map(goal_node)))
    route = [];
else
    route = [goal_node];
    
    while (parent_map(route(1)) ~= 0)
        route = [parent_map(route(1)), route];
    end
    
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end
toc;
disp(length(route));
end
