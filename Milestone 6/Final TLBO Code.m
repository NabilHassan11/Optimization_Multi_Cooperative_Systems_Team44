clc;
clear;
close all;

% TLBO Parameters
TLBO.ClassSize = 20; % Population Size
TLBO.MaxIterations = 60;

% Problem Parameters
step_size = 1; % Constant step size for UAV movement
num_UAVs = 3;
v_UAV_max = 3;
alpha = 1;
gamma = 0.5;
delta = 9;
tolerance = 3;

% Restricted Area
restricted_area_center = [125, 125];
restricted_area_radius = 15;

% Initial Positions
initial_position = [50, 50];
p_UAV_init = repmat(initial_position, num_UAVs, 1)';

% Define bounds for positions
% Each UAV has its position in a 2D plane (X, Y), so bounds should be a 2x2 matrix
% Row 1: Lower bounds for X and Y
% Row 2: Upper bounds for X and Y
bounds = [0, 0;  % Lower bounds for X, Y
    250, 250]; % Upper bounds for X, Y (adjust according to your problem)


% Designated Targets
designated_targets = [200, 70; 100, 200; 200, 200]';

% Objective Function
objective_function = @(x) sum(alpha * exploration_time(x)) - ...
    sum(gamma * coverage_efficiency(x)) + ...
    sum(delta * designated_targets_penalty(x, designated_targets));

% Constraints
constraints = @(x) nonlcon(x);

% Initialize Population
class = cell(TLBO.ClassSize, 1);
for i = 1:TLBO.ClassSize
    student = zeros(4, num_UAVs);
    for j = 1:num_UAVs
        vx = rand * 2 - 1;
        vy = rand * 2 - 1;
        vx = max(-v_UAV_max, min(v_UAV_max, vx));
        vy = max(-v_UAV_max, min(v_UAV_max, vy));
        px = p_UAV_init(1, j);
        py = p_UAV_init(2, j);
        student(:, j) = [vx; vy; px; py];
    end
    class{i} = student;
end

% Initialize Visualization
best_positions = p_UAV_init;
path_log = best_positions;
all_best_cost_log = [];
all_paths = cell(num_UAVs, 1);
for j = 1:num_UAVs
    all_paths{j} = p_UAV_init(:, j)';
end

% Main Figure
figure;

% Subplot 1: UAV Positions
subplot(1, 3, 1);
hold on;
title('UAV Positions');
axis([0 250 0 250]);
viscircles(restricted_area_center, restricted_area_radius, 'LineStyle', '--', 'Color', 'r');
plot(initial_position(1), initial_position(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Start Position');
plot(designated_targets(1, :), designated_targets(2, :), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Targets');
uav_plot = plot(nan, nan, 'ks', 'MarkerSize', 8);

% Subplot 2: Fitness per TLBO Run
subplot(1, 3, 2);
hold on;
title('Fitness Value per TLBO Run');
xlabel('Iteration');
ylabel('Best Fitness');
plot_per_run = plot(nan, nan, 'LineWidth', 2);

% Subplot 3: Overall Fitness
subplot(1, 3, 3);
hold on;
title('Overall Fitness Over Iterations');
xlabel('Iteration');
ylabel('Best Fitness');
plot_overall = plot(nan, nan, 'LineWidth', 2);

% Initialize variables
lifetime_best_fitness = []; % Array to store the lifetime best fitness
current_lifetime_best = Inf; % Track the lifetime best fitness (initialize to a large value)


% Main Loop
while true
    bestcostarr = [];
    best_cost_log = zeros(TLBO.MaxIterations, 1);
    penalty_value = 1e4;

    % Initialize global best tracker for the entire TLBO run
    global_best_solution = [];
    global_best_fitness = Inf;

    for gen = 1:TLBO.MaxIterations
        costs = zeros(TLBO.ClassSize, 1);

        % Evaluate Fitness and Determine Initial Teacher
        for i = 1:TLBO.ClassSize
            student = class{i};
            cost = objective_function(student);
            [c, ceq] = nonlcon(student);
            if any(c > 0) || any(ceq ~= 0)
                penalty = penalty_value * sum(c(c > 0));
                cost = cost + penalty;
            end
            costs(i) = cost;

            % Update global best solution
            if cost < global_best_fitness
                global_best_fitness = cost;
                global_best_solution = student;
            end
        end

        % Determine the teacher
        [~, best_idx] = min(costs); % Find the best solution index
        teacher = class{best_idx}; % Best solution acts as the teacher
        mean_solution = zeros(size(class{1}));

        % Calculate mean solution per UAV
        for uav = 1:num_UAVs
            uav_data = cellfun(@(s) s(:, uav), class, 'UniformOutput', false);
            uav_data_stack = cat(3, uav_data{:});
            mean_solution(:, uav) = mean(uav_data_stack, 3);
        end

        % Teaching Phase
        for i = 1:TLBO.ClassSize
            r = rand; % Random factor
            T = randi([1, 2]); % Teaching factor (1 or 2)
            updated = class{i}; % Start with the current student

            % Update both velocity and position for each UAV
            for j = 1:num_UAVs
                % Update velocity
                updated(1:2, j) = class{i}(1:2, j) + r * (teacher(1:2, j) - T * mean_solution(1:2, j));
                %updated(3:4, j) = class{i}(3:4, j) + r * (teacher(3:4, j) - T * mean_solution(3:4, j));

                updated(1:2, j) = max(-v_UAV_max, min(v_UAV_max, updated(1:2, j))); % Clamp velocity

                % Update position based on new velocity
                updated(3:4, j) = updated(3:4, j) + updated(1:2, j);
                updated(3:4, j) = max(bounds(1, :)', min(bounds(2, :)', updated(3:4, j))); % Clamp position
            end

            class{i} = updated; % Assign back the updated student
        end


        % Re-evaluate Fitness After Teaching Phase
        new_costs = zeros(1, TLBO.ClassSize);
        for i = 1:TLBO.ClassSize
            student = class{i};
            cost = objective_function(student);
            [c, ceq] = nonlcon(student);
            if any(c > 0) || any(ceq ~= 0)
                penalty = penalty_value * sum(c(c > 0));
                cost = cost + penalty;
            end
            new_costs(i) = cost;

            % Update global best solution
            if cost < global_best_fitness
                global_best_fitness = cost;
                global_best_solution = student;
            end
        end

        % Update the lifetime best fitness if needed
        if global_best_fitness < current_lifetime_best
            current_lifetime_best = global_best_fitness;
            lifetime_best_fitness = [lifetime_best_fitness; current_lifetime_best]; % Append to lifetime log
        end

        % Check if teacher has changed
        [~, new_best_idx] = min(new_costs);
        if new_best_idx ~= best_idx
            teacher_changed = true;
        else
            teacher_changed = false;
        end

        % Update teacher
        teacher = class{new_best_idx};

        % Learning Phase
        for i = 1:TLBO.ClassSize
            partner_idx = randi(TLBO.ClassSize); % Randomly select a partner
            while partner_idx == i
                partner_idx = randi(TLBO.ClassSize); % Ensure it's not the same individual
            end

            partner = class{partner_idx};
            updated = class{i}; % Start with the current student

            % Update both velocity and position for each UAV
            for j = 1:num_UAVs
                if costs(i) < costs(partner_idx)
                    % Move away from worse solution (partner)
                    updated(1:2, j) = class{i}(1:2, j) + rand * (class{i}(1:2, j) - partner(1:2, j));
                    %updated(3:4, j) = class{i}(3:4, j) + rand * (class{i}(3:4, j) - partner(3:4, j));

                else
                    % Move toward better solution (partner)
                    updated(1:2, j) = class{i}(1:2, j) - rand * (class{i}(1:2, j) - partner(1:2, j));
                    %updated(3:4, j) = class{i}(3:4, j) + rand * (class{i}(3:4, j) - partner(3:4, j));

                end
                updated(1:2, j) = max(-v_UAV_max, min(v_UAV_max, updated(1:2, j))); % Clamp velocity

                % Update position based on new velocity
                updated(3:4, j) = updated(3:4, j) + updated(1:2, j);
                updated(3:4, j) = max(bounds(1, :)', min(bounds(2, :)', updated(3:4, j))); % Clamp position
            end

            class{i} = updated; % Assign back the updated student
        end

        % Reset the class except the teacher
        for i = 1:TLBO.ClassSize
            if i ~= best_idx % Skip the teacher
                % Select 60% of the students to inherit the teacher's solution
                if rand <= 0.6
                    % 75% of the class inherits the teacher's previous solution
                    class{i}(1:2, :) = teacher(1:2, :); % Set the velocities to teacher's values
                    class{i}(3:4, :) = teacher(3:4, :); % Set the positions to teacher's values
                else
                    % 25% of the class gets random noise
                    class{i}(1:2, :) = class{i}(1:2, :) + 0.1 * randn(2, num_UAVs); % Add noise to velocities
                    class{i}(3:4, :) = class{i}(3:4, :) + 0.1 * randn(2, num_UAVs); % Add noise to positions

                    % Clamp velocities and positions
                    class{i}(1:2, :) = max(-v_UAV_max, min(v_UAV_max, class{i}(1:2, :))); % Clamp velocities
                    class{i}(3:4, :) = max(bounds(1, :)', min(bounds(2, :)', class{i}(3:4, :))); % Clamp positions
                end
            end
        end


        % Update Best Cost
        best_cost_log(gen) = costs(best_idx);
        bestcostarr = [bestcostarr; global_best_fitness];

        % Update Fitness Plots
        set(plot_per_run, 'YData', bestcostarr, 'XData', 1:length(bestcostarr));
        all_best_cost_log = [all_best_cost_log; global_best_fitness];
        set(plot_overall, 'YData', all_best_cost_log, 'XData', 1:length(all_best_cost_log));
    end

    % After all iterations, update the UAV positions with the best global solution
    best_solution = global_best_solution; % Best solution from the final iteration
    uav_positions = zeros(num_UAVs, 2);
    stopped_uavs = false(1, num_UAVs); % Track which UAVs have stopped

    for j = 1:num_UAVs
        if stopped_uavs(j)
            % If the UAV has already reached its target, skip further updates
            continue;
        end

        % Extract velocity and position from the best solution
        vx = best_solution(1, j);
        vy = best_solution(2, j);
        px = best_solution(3, j);
        py = best_solution(4, j);

        % Get the current position for the UAV
        current_position = best_positions(:, j);

        % Calculate the direction to move
        direction = [px; py] - current_position; % Ensure column vectors
        if norm(direction) > 0
            direction = direction / norm(direction); % Normalize to unit vector
        end

        % Incremental step towards the target position
        retry_limit = 10; % Maximum number of retries for generating a valid position
        retries = 0;
        valid_position_found = false;

        while retries < retry_limit && ~valid_position_found
            tentative_position = current_position + step_size * direction;

            % Check constraints (restricted area and bounds)
            distance_to_restricted = norm(tentative_position - restricted_area_center');
            if distance_to_restricted >= restricted_area_radius + 10
                % Valid position found
                valid_position_found = true;
            else
                % Generate a new random direction to escape the restricted area
                direction = rand(2, 1) - 0.5; % Random direction vector
                direction = direction / norm(direction); % Normalize to unit vector
                retries = retries + 1;
            end
        end

        if valid_position_found
            % Update to the valid tentative position
            next_position = tentative_position;
        else
            % If no valid position found, keep the current position
            next_position = current_position;
        end

        % Check if the UAV has reached its target within the tolerance
        if norm(next_position - designated_targets(:, j)) < tolerance
            stopped_uavs(j) = true; % Mark this UAV as stopped
            next_position = designated_targets(:, j); % Snap to the target for accuracy
            fprintf('UAV %d has reached its target.\n', j);
        end

        % Update the UAV position
        best_positions(:, j) = next_position; % Correct dimension
        uav_positions(j, :) = next_position'; % Convert to row vector for logging
        all_paths{j} = [all_paths{j}; next_position']; % Log the path as a row vector
    end


    % Update UAV Path Plot
    subplot(1, 3, 1);
    set(uav_plot, 'XData', uav_positions(:, 1), 'YData', uav_positions(:, 2));
    for j = 1:num_UAVs
        plot(all_paths{j}(:, 1), all_paths{j}(:, 2), '-', 'LineWidth', 1.5, ...
            'DisplayName', ['UAV ' num2str(j)]);
    end
    drawnow; % Ensure real-time updates

    initial_positions = best_positions;

    % Check Completion
    if all(vecnorm(best_positions' - designated_targets', 2, 2) < tolerance)
        disp('All UAVs have reached their designated targets.');

        % Final Convergence Plot
        figure;
        plot(1:length(lifetime_best_fitness), lifetime_best_fitness, 'LineWidth', 2);
        grid on;
        title('TLBO Overall Convergence Curve');
        xlabel('Runs');
        ylabel('Best Fitness Value');
        legend('Overall Best Fitness', 'Location', 'Best');
        set(gca, 'FontSize', 12);

        break;
    end
end

function T_explore = exploration_time(x)
num_UAVs = size(x, 2);  % Get number of UAVs
T_explore = zeros(1, num_UAVs);  % Initialize array for individual UAV times
for i = 1:num_UAVs
    v_UAV = norm([x(1, i), x(2, i)]);  % Velocity of UAV i
    T_explore(i) = 100 / v_UAV;  % Exploration time for UAV i
end
end

function C_eff = coverage_efficiency(x)
num_UAVs = size(x, 2);  % Get number of UAVs
area = 62500;  % Area of exploration
C_eff = zeros(1, num_UAVs);  % Initialize array for individual efficiencies
for i = 1:num_UAVs
    C_UAV = sum(x(3:4, i));  % Coverage for UAV i
    C_eff(i) = C_UAV / area;  % Efficiency for UAV i
end
end


function penalty = designated_targets_penalty(x, designated_targets)
num_UAVs = size(x, 2);  % Get number of UAVs
penalty = zeros(1, num_UAVs);  % Initialize array for individual penalties
for i = 1:num_UAVs
    p_UAV = x(3:4, i)';  % Position of UAV i
    target = designated_targets(:, i)';  % Target for UAV i
    distance_to_target = norm(p_UAV - target);  % Distance to target
    penalty(i) = distance_to_target;  % Penalty for UAV i
end
end


function [c, ceq] = nonlcon(x)
num_UAVs = size(x, 2);
d_safe = 10.0;  % Safe distance between UAVs
restricted_area_center = [125, 125];
restricted_area_radius = 15;

% Initialize constraints
c = zeros(num_UAVs, 1);

% Check each UAV's distance to restricted area
for i = 1:num_UAVs
    distance_to_restricted = norm(x(3:4, i)' - restricted_area_center) - (restricted_area_radius + 10);
    c(i) = -distance_to_restricted;  % This will be negative if in violation
end

% Check for safe distance between each pair of UAVs
safe_distance_violations = 0;
for i = 1:num_UAVs
    for j = i+1:num_UAVs
        distance_between_UAVs = norm(x(3:4, i) - x(3:4, j));
        if distance_between_UAVs < d_safe
            safe_distance_violations = safe_distance_violations + 1;
        end
    end
end

% Add the safe distance violation count to the constraint array
c(num_UAVs + 1) = safe_distance_violations;


% No equality constraints
ceq = [];
end
