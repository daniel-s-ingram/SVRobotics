function ASTAR(input_map, start_pos, goal_pos)
tic;
cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
        0.5 0.5 0.5];

figure('NumberTitle', 'off', 'Name', 'A* Algorithm');
colormap(cmap);

[nrows, ncols] = size(input_map);

map = zeros(nrows,ncols);
map(~input_map) = 1;
map(input_map)  = 2;

start_node = sub2ind(size(map), start_pos(1), start_pos(2));
goal_node  = sub2ind(size(map), goal_pos(1),  goal_pos(2));

map(start_node) = 5;
map(goal_node)  = 6;

parent_map = zeros(nrows, ncols);
 
[X, Y] = meshgrid (1:ncols, 1:nrows);

xd = goal_pos(1);
yd = goal_pos(2);

H = abs(X - xd) + abs(Y - yd);
H = H';

f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

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
    
    [min_f, current_node] = min(f(:));
    
    if ((current_node == goal_node) || isinf(min_f))
        break;
    end;
    
    map(current_node) = 3;
    f(current_node) = Inf;
    
    [i, j] = ind2sub(size(f), current_node);
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
            if g(k) > g(current_node) + 1
                g(k) = g(current_node) + 1;
                f(k) = H(k);
                map(k) = 4;
                parent_map(k) = current_node;
            end
        end
    end
    %pause;
end

%% Construct route from start to dest by following the parent links
if (isinf(f(goal_node)))
    route = [];
else
    route = [goal_node];
    
    while (parent_map(route(1)) ~= 0)
        route = [parent_map(route(1)), route];
    end

    % Snippet of code used to visualize the map and the path
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
