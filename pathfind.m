
function distance_grid = pathfind(course, start, goal, use_astar)
	if nargin < 4, use_astar = true; end
	assert(course(goal(1), goal(2)) == 0);
	h = @(p) use_astar * (abs(p(1) - goal(1)) + abs(p(2) - goal(2)));
	distance_grid = inf(size(course));
	distance_grid(start(1), start(2)) = 0;
	queue = [h(start); 0; start];
	while ~isempty(queue)
		[~, min_index] = min(queue(1, :));
		queue_col = queue(:, min_index);
		queue(:, min_index) = [];
		dist = queue_col(2);
		node_pos = queue_col(3:4);
		if dist ~= distance_grid(node_pos(1), node_pos(2))
			continue;
		end
		if all(node_pos == goal)
			return;
		end
		neighbour_positions = [node_pos + [1; 0], node_pos + [0; 1], node_pos + [-1; 0], node_pos + [0; -1]];
		for i = 1:4
			neighbour_pos = neighbour_positions(:, i);
			if neighbour_pos(1) > 0 && neighbour_pos(2) > 0 && neighbour_pos(1) <= size(course, 1) && neighbour_pos(2) <= size(course, 2) && course(neighbour_pos(1), neighbour_pos(2)) == 0
				new_dist = dist + 1;
				if new_dist < distance_grid(neighbour_pos(1), neighbour_pos(2))
					distance_grid(neighbour_pos(1), neighbour_pos(2)) = new_dist;
					queue(:, end+1) = [new_dist + h(neighbour_pos); new_dist; neighbour_pos];
				end
			end
		end
	end
end