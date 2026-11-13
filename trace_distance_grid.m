
function steps = trace_distance_grid(distance_grid, goal)
	assert(ismatrix(distance_grid));
	assert(isvector(goal) && numel(goal) == 2);
	goal = goal(:);
	assert(all(goal == floor(goal)) && all(isfinite(goal)));
	assert(goal(1) > 0 && goal(1) <= size(distance_grid, 1));
	assert(goal(2) > 0 && goal(2) <= size(distance_grid, 2));

	max_dist = distance_grid(goal(1), goal(2));
	if ~isfinite(max_dist)
		fprintf("Goal is unreachable.");
		steps = [];
		return;
	end
	steps = zeros(max_dist + 1, 2);
	node_pos = goal;
	dist = max_dist;
	
	for i = 1:max_dist
		steps(end-i+1, :) = node_pos.';
		dist = dist - 1;
		neighbour_positions = [node_pos + [1; 0], node_pos + [0; 1], node_pos + [-1; 0], node_pos + [0; -1]];
		new_pos = [-1, -1];
		for j = 1:4
			neighbour_pos = neighbour_positions(:, j);
			if neighbour_pos(1) > 0 && neighbour_pos(2) > 0 && neighbour_pos(1) <= size(distance_grid, 1) && neighbour_pos(2) <= size(distance_grid, 2) && distance_grid(neighbour_pos(1), neighbour_pos(2)) == dist
				new_pos = neighbour_pos;
				break;
			end
		end
		if new_pos(1) < 0
			fprintf("Distance grid could not be traced.");
			steps = [];
			return;
		end
		node_pos = new_pos;
	end
	steps(1, :) = node_pos.';
end
