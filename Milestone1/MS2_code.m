% Define parameters for visualization and fitness calculation
params.TotalArea = 1000 * 1000; % 1 km^2
params.MaxTime = 1000;          % Maximum time in seconds
params.w1 = 1.0;                % Weight for explored area
params.w2 = 0.5;                % Weight for time taken
params.v_min = 5;               % Minimum speed (m/s)
params.v_max = 20;              % Maximum speed (m/s)
params.communicationRadius = 500; % Communication radius (meters)
params.coverageRadius = 100;    % Coverage radius per UAV (meters)
params.penaltyFactor = 1000;    % Penalty factor for violations
params.TimeScalingFactor = 100; % Scaling factor to convert speed to time
params.gridResolution = 10;     % Grid resolution in meters
params.AreaDimensions = [1000, 1000]; % Dimensions of the total area (meters)
params.safeDistance = 50;       % Minimum safe distance between drones to avoid collisions (meters)

% Define restricted areas as a function handle
params.restrictedArea = @(pos) (norm(pos - [500, 500]) <= 100);

% Initialize UAV positions and speeds
N = 5; % Number of UAVs
positions = [100, 100;
             200, 200;
             300, 300;
             400, 400;
             600, 600]; % [N x 2] positions in meters
         
% Initialize random directions and speeds
speeds = [10; 15; 20; 5; 18]; % [N x 1] speeds in m/s
directions = rand(N, 2) - 0.5; % Random initial directions
directions = directions ./ vecnorm(directions, 2, 2); % Normalize directions

% Initialize targets (3 static targets)
targets = [800, 800;
           150, 850;
           650, 300]; % [3 x 2] target positions in meters
exploredTargets = false(3, 1); % Boolean array to track if a target is explored

% Time parameters for simulation
dt = 1;           % Time step (seconds)
totalSteps = 10000; % Max number of steps (fail-safe termination)

% Initialize coverage grid for explored area calculation
gridX = 0:params.gridResolution:params.AreaDimensions(1);
gridY = 0:params.gridResolution:params.AreaDimensions(2);
[gridX, gridY] = meshgrid(gridX, gridY);
exploredGrid = false(size(gridX)); % Grid to track explored area

% Set up figure for animation
figure;
hold on;

% Plot search area
rectangle('Position', [0, 0, params.AreaDimensions(1), params.AreaDimensions(2)], ...
    'EdgeColor', 'k', 'LineWidth', 1.5, 'LineStyle', '--');
fill([0, 0, params.AreaDimensions(1), params.AreaDimensions(1)], ...
     [0, params.AreaDimensions(2), params.AreaDimensions(2), 0], ...
     'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
text(50, 950, 'Search Area', 'Color', 'g', 'FontWeight', 'bold');

% Plot restricted area
theta = linspace(0, 2*pi, 100);
x_restricted = 500 + 100 * cos(theta);
y_restricted = 500 + 100 * sin(theta);
fill(x_restricted, y_restricted, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'k');
text(460, 500, 'Restricted Area', 'Color', 'r', 'FontWeight', 'bold');

% Plot targets (create individual target markers)
targetMarkers = gobjects(3, 1); % Pre-allocate array for target markers
for t = 1:3
    targetMarkers(t) = plot(targets(t,1), targets(t,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    text(targets(t,1) + 10, targets(t,2), sprintf('Target %d', t), 'Color', 'r', 'FontWeight', 'bold');
end

% Initialize handles for UAVs and coverage
uavMarkers = gobjects(N, 1);
uavCircles = gobjects(N, 1);
uavComms = gobjects(N, N);

% Set plot limits
xlim([0, params.AreaDimensions(1)]);
ylim([0, params.AreaDimensions(2)]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('UAV Search Area, Coverage, Targets, and Movement');

% Initialize UAV markers and coverage circles
for i = 1:N
    uavMarkers(i) = plot(positions(i,1), positions(i,2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    uavCircles(i) = viscircles(positions(i,:), params.coverageRadius, 'LineStyle', '--', 'EdgeColor', [0.6 0.6 1], 'LineWidth', 1.5);
end

% Main animation loop
step = 0;
while ~all(exploredTargets) && step < totalSteps
    step = step + 1;
    
    % Update UAV positions based on speed and direction
    positions = positions + speeds .* directions * dt;
    
    % Check boundary conditions (bounce back if hitting boundary)
    for i = 1:N
        % Boundary constraint
        if positions(i, 1) <= 0 || positions(i, 1) >= params.AreaDimensions(1)
            directions(i, 1) = -directions(i, 1); % Reverse x-direction
        end
        if positions(i, 2) <= 0 || positions(i, 2) >= params.AreaDimensions(2)
            directions(i, 2) = -directions(i, 2); % Reverse y-direction
        end
        
        % Restricted area avoidance
        if params.restrictedArea(positions(i,:))
            directions(i, :) = -directions(i, :); % Reverse direction if inside restricted area
        end
        
        % Flight speed constraint
        if speeds(i) < params.v_min
            speeds(i) = params.v_min;
        elseif speeds(i) > params.v_max
            speeds(i) = params.v_max;
        end
    end
    
    % Communication and Collision Avoidance Constraints
    for i = 1:N
        for j = i+1:N
            distance = norm(positions(i,:) - positions(j,:));
            
            % Adjust directions if they exceed communication range
            if distance >= params.communicationRadius
                direction_correction = positions(j,:) - positions(i,:);
                direction_correction = direction_correction / norm(direction_correction); % Normalize
                directions(i,:) = directions(i,:) + 0.1 * direction_correction;
                directions(j,:) = directions(j,:) - 0.1 * direction_correction;
                directions(i,:) = directions(i,:) / norm(directions(i,:)); % Normalize new direction
                directions(j,:) = directions(j,:) / norm(directions(j,:));
            end
            
            % Collision avoidance: if drones come too close, adjust directions
            if distance < params.safeDistance
                % Move them apart if they are too close
                avoidance_direction = positions(i,:) - positions(j,:);
                avoidance_direction = avoidance_direction / norm(avoidance_direction); % Normalize
                directions(i,:) = directions(i,:) + 0.3 * avoidance_direction;
                directions(j,:) = directions(j,:) - 0.3 * avoidance_direction;
                directions(i,:) = directions(i,:) / norm(directions(i,:)); % Normalize new direction
                directions(j,:) = directions(j,:) / norm(directions(j,:));
                
                % Immediately adjust positions to maintain safe distance
                overlap_correction = (params.safeDistance - distance) / 2;
                positions(i,:) = positions(i,:) + overlap_correction * avoidance_direction;
                positions(j,:) = positions(j,:) - overlap_correction * avoidance_direction;
            end
        end
    end
    
    % Check if any targets are explored by UAVs
    for i = 1:N
        for t = 1:3
            if ~exploredTargets(t) && norm(positions(i,:) - targets(t,:)) <= params.coverageRadius
                exploredTargets(t) = true; % Mark target as explored
                set(targetMarkers(t), 'Color', 'g'); % Change target marker color to green
                text(targets(t,1) + 10, targets(t,2), sprintf('Target %d Explored', t), 'Color', 'g', 'FontWeight', 'bold');
            end
        end
    end
    
    % Update UAV markers and coverage areas
    for i = 1:N
        % Update position
        set(uavMarkers(i), 'XData', positions(i,1), 'YData', positions(i,2));
        
        % Update coverage circle
        delete(uavCircles(i)); % Remove the old circle
        uavCircles(i) = viscircles(positions(i,:), params.coverageRadius, 'LineStyle', '--', 'EdgeColor', [0.6 0.6 1], 'LineWidth', 1.5);
    end
    
    % Mark explored areas in grid
    for i = 1:N
        distancesToGrid = sqrt((gridX - positions(i,1)).^2 + (gridY - positions(i,2)).^2);
        exploredGrid = exploredGrid | (distancesToGrid <= params.coverageRadius); % Mark grid points within coverage radius
    end
    
    % Calculate explored area and fitness value at each iteration
    exploredArea = sum(exploredGrid(:)) * (params.gridResolution^2); % Area in square meters
    fitnessValue = params.w1 * (exploredArea / params.TotalArea) - params.w2 * (step / params.MaxTime);
    
    % Display the fitness value at each iteration
    fprintf('Step: %d, Fitness Value: %.4f\n', step, fitnessValue);
    
    % Remove previous communication lines and redraw them
    delete(findall(gca, 'Type', 'Line', 'Color', 'm'));
    for i = 1:N
        for j = i+1:N
            distance = norm(positions(i,:) - positions(j,:));
            if distance < params.communicationRadius
                plot([positions(i,1), positions(j,1)], [positions(i,2), positions(j,2)], 'm-.', 'LineWidth', 1.5);
            end
        end
    end
    
    % Pause for animation effect
    pause(0.1);
end

% Finalize the plot
legend('Search Area', 'Restricted Area', 'Target', 'UAV Position', 'Communication Link', ...
       'Location', 'northeastoutside');
grid on;
hold off;

% Display total steps to find all targets
fprintf('All targets found in %d steps.\n', step);
