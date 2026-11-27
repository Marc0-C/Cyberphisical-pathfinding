
function path_gui
	course;

	fig = figure("Name","Advanced Pathfinding System","Position", [100 ,100 ,900 ,600]);
	% Axes where the map and paths are drawn
	h_axes = axes("Parent", fig ,"Units","pixels","Position" ,[250 ,100 ,600 ,450]) ;
	uicontrol("Style", "pushbutton", "Position", [20, 520, 120, 30], "String", "Calculate Path", "Callback", @(s , e) calculate_path(h_axes));
	uicontrol("Style", "pushbutton", "Position", [20, 480, 120, 30], "String", "Step", "Callback", @(s , e) step(h_axes));
	uicontrol("Style", "pushbutton", "Position", [20, 440, 120, 30], "String", "Add Obstacle", "Callback", @(s , e) update_grid(h_axes));
	
	setappdata(h_axes, "start", [15 2]);
	setappdata(h_axes, "goal", [15 15]);
	setappdata(h_axes, "agent", [15 2]);
	setappdata(h_axes, "course", course_initial);
	setappdata(h_axes, "distance_grid", inf(16));
	setappdata(h_axes, "steps", []);
  setappdata(h_axes, "courses", {course_initial, course_obstacle_1, course_obstacle_2, course_obstacle_3});
	setappdata(h_axes, "course_idx", 1);
  
	draw_course(h_axes);
	
	% Inputs for grid size and obstacle density
	%uicontrol("Style", "text", "Position" ,[20 ,540 ,120 ,20] ,"String","Grid Size :","HorizontalAlignment","left") ;
	%gridSizeEdit = uicontrol("Style","edit","Position" ,[150 ,540 ,80 ,20] ,"String","16") ;
	%uicontrol("Style", "text", "Position", [20 ,500 ,120 ,20], "String", "Obstacle Density(0 -1):" , "HorizontalAlignment", "left") ;
	%obstacleDensityEdit = uicontrol("Style","edit","Position" ,[150 ,500 ,80 ,20] ,"String","0.2");
	% Start /End coordinate inputs
	%uicontrol("Style","text","Position" ,[20 ,420 ,120 ,20] ,"String","Start(x,y):","HorizontalAlignment","left");
	%startPointEdit = uicontrol("Style","edit","Position" ,[150 ,420 ,80 ,20] ,"String","1 ,1") ;
	%uicontrol("Style","text","Position" ,[20 ,380 ,120 ,20] ,"String","End(x,y):","HorizontalAlignment","left") ;
	%endPointEdit = uicontrol("Style","edit","Position" ,[150 ,380 ,80 ,20] ,"String","16 ,16");
	% Buttons and dropdown
	
	%{
	uicontrol("Style","pushbutton","Position" ,[20 ,460 ,120 ,30] ,"String","Generate Map","Callback", @(s , e) generateMap( gridSizeEdit , obstacleDensityEdit , hAxes ));
	uicontrol("Style","pushbutton","Position" ,[20 ,340 ,120 ,30] ,"String","Plot Start /End","Callback", @(s , e) plotStartEnd( startPointEdit , endPointEdit , hAxes ) ) ;
	uicontrol("Style","pushbutton","Position" ,[20 ,300 ,120 ,30] ,"String","Find Path","Callback", @(s , e) findPath( startPointEdit , endPointEdit , hAxes ) ) ;
	uicontrol("Style","text","Position" ,[20 ,260 ,120 ,20] ,"String","Select Path :","HorizontalAlignment","left") ;
	pathDropdown = uicontrol("Style","popupmenu","Position" ,[150 ,260 ,120 ,22] ,"String" ,{"Select path"} ,"Callback", @(s , e) highlightSelectedPath( hAxes ) ) ;
	setappdata( hAxes ,"pathDropdown", pathDropdown ) ;
	uicontrol("Style","pushbutton","Position" ,[20 ,220 ,150 ,30] ,"String","Generate	Alternative Path",	"Callback", @(s , e) findAlternativePath( startPointEdit , endPointEdit , hAxes ) ) ;
	uicontrol("Style","pushbutton","Position" ,[20 ,180 ,150 ,30] ,"String","Start Simulation","Callback", @(s , e) simulateDronePath(startPointEdit, endPointEdit, hAxes));
	%}
end

function draw_course(h_axes)
	start = getappdata(h_axes, "start");
	goal = getappdata(h_axes, "goal");
	agent = getappdata(h_axes, "agent");
	course = getappdata(h_axes, "course");
	distance_grid = getappdata(h_axes, "distance_grid");
	steps = getappdata(h_axes, "steps");

	grey_image = uint8(255.99 * (1 - course));
	rgb_image = cat(3, grey_image, grey_image, grey_image);
	finite_cells = isfinite(distance_grid);
	colour_indices = round(rescale(distance_grid(finite_cells), 1, 256));
	colour_map = uint8(255 * parula(256));
	pixel_colours = reshape(rgb_image, [], 3);
	pixel_colours(finite_cells(:), :) = colour_map(colour_indices, :);
	rgb_image = reshape(pixel_colours, size(rgb_image));

	rgb_image(start(1), start(2), :) = [0, 255, 0];
	rgb_image(goal(1), goal(2), :) = [255, 0, 0];

	imshow(rgb_image);

	hold on;
	for row = 1:size(distance_grid, 1)
		for col = 1:size(distance_grid, 2)
			if ~isfinite(distance_grid(row, col))
				continue;
			end
			text(col - 0.45, row - 0.45, num2str(distance_grid(row, col)), ...
				'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
				'FontSize', 8, 'Color', 'k');
		end
	end

	if size(steps, 1) > 1
		directions = diff(steps);
		arrow_starts = steps(1:end-1, :) + 0.25 - 0.15 * directions;
		quiver(arrow_starts(:, 2), arrow_starts(:, 1), ...
			0.3 * directions(:, 2), 0.3 * directions(:, 1), 0, ...
			'Color', 'r', 'LineWidth', 1.2, 'MaxHeadSize', 1);
	end
	plot(agent(2), agent(1), 'o', 'Color', [0.8, 0.2, 1], ...
		'MarkerFaceColor', [0.8, 0.2, 1], 'MarkerSize', 6);
	hold off;
end

function calculate_path(h_axes)
	goal = getappdata(h_axes, "goal");
	course = getappdata(h_axes, "course");
	agent = getappdata(h_axes, "agent");

	distance_grid = pathfind(course, agent, goal);
	steps = trace_distance_grid(distance_grid, goal);

	setappdata(h_axes, "distance_grid", distance_grid);
	setappdata(h_axes, "steps", steps);
	
	draw_course(h_axes);
end

function step(h_axes)
	agent = getappdata(h_axes, "agent");
	steps = getappdata(h_axes, "steps");

	index = -1;
	for i=1:size(steps, 1)
		if steps(i, 1) == agent(1) && steps(i, 2) == agent(2)
			index = i;
			break;
		end
	end

	if index > 0 && index < size(steps, 1)
		agent(:) = steps(index + 1, :);
		setappdata(h_axes, "agent", agent);
	end
	
	draw_course(h_axes);
end

function update_grid(h_axes)
	course;

	start = getappdata(h_axes, "start");
	goal = getappdata(h_axes, "goal");
	agent = getappdata(h_axes, "agent");
	course_ = getappdata(h_axes, "course");
	distance_grid = getappdata(h_axes, "distance_grid");
	steps = getappdata(h_axes, "steps");

	% Add obstacle.
	course_id = randi(3);
	if course_id == 1
		course_ = course_obstacle_1;
	elseif course_id == 2
		course_ = course_obstacle_2;
	else
		course_ = course_obstacle_3;
	end

	% Get current location in path.
	index = -1;
	for i=1:size(steps, 1)
		if steps(i, 1) == agent(1) && steps(i, 2) == agent(2)
			index = i;
			break;
		end
	end
	if index > 0
		% Check if the route is still valid.
		valid = true;
		for i=index:(length(steps))
			if course_(steps(i, 1), steps(i, 2)) == 1
				valid = false;
				break;
			end
		end
		if ~valid
			fprintf("Route invalidated, recalculating.");
			distance_grid = pathfind(course_, agent, goal);
			steps = trace_distance_grid(distance_grid, goal);
		end
	end

	setappdata(h_axes, "course", course_);
	setappdata(h_axes, "distance_grid", distance_grid);
	setappdata(h_axes, "steps", steps);

	draw_course(h_axes);
end
