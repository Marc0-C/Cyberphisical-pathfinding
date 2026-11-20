
function [distance_grid, explored] = pathfind_1(course, start, goal, use_astar)
if nargin < 4
    use_astar = true;
end

assert(ismatrix(course));
assert(isvector(start) && numel(start) == 2);
assert(isvector(goal ) && numel(goal ) == 2);
start = start(:);
goal = goal(:);

assert(all(start == floor(start)) && all(isfinite(start)));
assert(all(goal == floor(goal)) && all(isfinite(goal)));

assert(start(1) > 0 && start(1) <= size(course, 1));
assert(start(2) > 0 && start(2) <= size(course, 2));

assert(goal(1) > 0 && goal(1) <= size(course, 1));
assert(goal(2) > 0 && goal(2) <= size(course, 2));

assert(course(start(1), start(2)) == 0, "Start cell is blocked.");
assert(course(goal(1), goal(2)) == 0, "Goal cell is blocked.");

h = @(p) use_astar * (abs(p(1) - goal(1)) + abs(p(2) - goal(2)));

distance_grid = inf(size(course));
distance_grid(start(1), start(2)) = 0;
explored = false(size(course));

queue = [h(start); 0; start];

while ~isempty(queue)
    % take the waiting cell with the smallest f (most promising)
    [~, min_index] = min(queue(1, :));
    queue_col = queue(:, min_index);
    queue(:, min_index) = [];
    dist = queue_col(2);
    node_pos = queue_col(3:4);

    if dist ~= distance_grid(node_pos(1), node_pos(2))
        continue;
    end
    explored(node_pos(1), node_pos(2)) = true;

    if all(node_pos == goal)
        return;
    end

    neighbour_positions = [node_pos + [1; 0], node_pos + [0; 1], node_pos + [-1; 0], node_pos + [0; -1]];
    for i = 1:4
        neighbour_pos = neighbour_positions(:, i);
        % inside the map and not blocked
        if neighbour_pos(1) > 0 && neighbour_pos(2) > 0 && neighbour_pos(1) <= size(course, 1) && neighbour_pos(2) <= size(course, 2) && course(neighbour_pos(1), neighbour_pos(2)) == 0
            new_dist = dist + 1;
            if new_dist < distance_grid(neighbour_pos(1), neighbour_pos(2))
                distance_grid(neighbour_pos(1), neighbour_pos(2)) = new_dist;   % store g, not f
                queue(:, end+1) = [new_dist + h(neighbour_pos); new_dist; neighbour_pos];
            end
        end
    end
end
end