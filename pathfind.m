% calculat eht egrid with all of the distances
function distance_grid = pathfind(course, start, goal)
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

	assert(course(start(1), start(2)) == 0);

	distance_grid = inf(size(course));
	distance_grid(start(1), start(2)) = 0;
	queue = [0; start];

	while ~isempty(queue)
		[~, min_index] = min(queue(1, :));
		queue_col = queue(:, min_index);
		node_pos = queue_col(2:end);
		queue(:, min_index) = [];
		dist = distance_grid(node_pos(1), node_pos(2));
		if queue_col(1) ~= dist
			continue;
		end

		if all(node_pos == goal)
			return;
		end

		neighbour_positions = [node_pos + [1; 0], node_pos + [0; 1], node_pos + [-1; 0], node_pos + [0; -1]];
		for i = 1:4
			neighbour_pos = neighbour_positions(:, i);
			if neighbour_pos(1) > 0 && neighbour_pos(2) > 0 && neighbour_pos(1) <= size(course, 1) && neighbour_pos(2) <= size(course, 2) && course(neighbour_pos(1), neighbour_pos(2)) == 0
				old_dist = distance_grid(neighbour_pos(1), neighbour_pos(2));
				new_dist = dist + 1;
				if new_dist < old_dist
					distance_grid(neighbour_pos(1), neighbour_pos(2)) = new_dist;
					queue(:, end+1) = [new_dist; neighbour_pos];
				end
			end
		end
	end
end
