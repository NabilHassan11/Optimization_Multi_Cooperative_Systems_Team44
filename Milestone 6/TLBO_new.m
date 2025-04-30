
clc;
clear;
close all;

TLBO.ClassSize = 20;
TLBO.MaxIterations = 60;

% Problem Parameters
num_UAVs = 3;
v_UAV_max = 3;
d_safe = 1.0;
alpha = 1;
gamma = 0.5;
delta = 9;
tolerance = 3;
step_size = 1;

% Define restricted area
restricted_area_center = [125, 125];
restricted_area_radius = 15;

bounds = [0, 0;  % Lower bounds for X, Y
    250, 250];

% Define initial positions (all UAVs start from the same location)
initial_position = [50, 50];
p_UAV_init = repmat(initial_position, num_UAVs, 1)';

% Define target positions scattered around the map
designated_targets = [200, 70; 100, 200; 200, 200]';

% Objective function
objective_function = @(x) sum(alpha * exploration_time(x)) - ...
    sum(gamma * coverage_efficiency(x)) + ...
    sum(delta * designated_targets_penalty(x, designated_targets));

% Constraints (speed limits, restricted area)
constraints = @(x) nonlcon(x);

% Initialize population as cell array of 3x4 matrices
classroom = cell(TLBO.ClassSize, 1);
for i = 1:TLBO.ClassSize
    student = zeros(4, num_UAVs);  % 3x4 matrix for each individual
    for j = 1:num_UAVs
        vx = rand * 2 - 1;  % Independent velocity in x for UAV j
        vy = rand * 2 - 1;  % Independent velocity in y for UAV j

        px = p_UAV_init(1, j);
        py = p_UAV_init(2, j);
        student(:, j) = [vx; vy; px; py];
    end
    classroom{i} = student;
end

% Array to store the best position and velocity for each UAV at each step
best_positions = p_UAV_init;
path_log = best_positions;

% Store fitness values for plotting
all_best_cost_log = [];

% Set up the main figure with subplots for visualization
figure;

% Subplot for UAV path visualization
subplot(1, 3, 1);
hold on;
title('UAV Positions');
axis([0 250 0 250]);
viscircles(restricted_area_center, restricted_area_radius, 'LineStyle', '--', 'Color', 'r');
plot(initial_position(1), initial_position(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Start Position');
plot(designated_targets(1,:), designated_targets(2,:), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Targets');
uav_plot = plot(nan, nan, 'ks', 'MarkerSize', 8); % Placeholder for UAVs

% Subplot for fitness per GA run
subplot(1, 3, 2);
hold on;
title('Fitness Value per TLBO Run');
xlabel('Generation');
ylabel('Best Fitness');
plot_per_run = plot(nan, nan, 'LineWidth', 2);

% Subplot for overall fitness across all GA runs
subplot(1, 3, 3);
hold on;
title('Overall Fitness Function Over Generations Across TLBO Runs');
xlabel('Generation');
ylabel('Best Fitness');
plot_overall = plot(nan, nan, 'LineWidth', 2);

% Initialize a cell array to store the path for each UAV
all_paths = cell(num_UAVs, 1);
for j = 1:num_UAVs
    all_paths{j} = p_UAV_init(:, j)'; % Start with the initial positions
end

% Initialize a cell array to store the path for each UAV
all_paths = cell(num_UAVs, 1);
for j = 1:num_UAVs
    all_paths{j} = p_UAV_init(:, j)'; % Start with the initial positions
end

bestcostarr = [];
best_cost_log = zeros(TLBO.MaxIterations,1);
penalty_value = 1e3;

% Initialize global best tracker for the entire TLBO run
global_best_teacher = [];
global_best_fitness = Inf;
% Main loop until all UAVs reach their targets
while true

    for iter = 1:TLBO.MaxIterations
        costs = zeros(TLBO.ClassSize,1);
        for stu = 1:TLBO.ClassSize
            student = classroom{stu};
            cost = objective_function(student);

            [c,ceq] = nonlcon(student);

            
            costs(stu) = cost;
        end

        [~,best_idx] = min(costs);
        teacher = classroom{best_idx};
        mean_sol = zeros(size(classroom{1}));

        if isempty(global_best_teacher)
            global_best_fitness = costs(best_idx);
            global_best_teacher = teacher;
        end

        if costs(best_idx) < global_best_fitness
            global_best_fitness = costs(best_idx);
            global_best_teacher = teacher;
        end

        bestcostarr = [bestcostarr, global_best_fitness];

        for uav = 1:num_UAVs
            % Step 1: Convert the classroom cell array into a 3D array
            % Assuming classroom contains matrices of size (4 x num_UAVs)
            classroom_array = cat(3, classroom{:}); % Dimensions: (4 x num_UAVs x num_students)

            % Step 2: Compute the mean across all students (3rd dimension)
            mean_sol = mean(classroom_array, 3); % Dimensions: (4 x num_UAVs)

            % mean_sol now contains:
            % Row 1: Mean vx for each UAV
            % Row 2: Mean vy for each UAV
            % Row 3: Mean px for each UAV
            % Row 4: Mean py for each UAV


        end

        % Teaching Phase
        for i = 1:TLBO.ClassSize
            updated_student = classroom{i};

            for uav = 1:num_UAVs
                r = rand;
                T = randi([1,2]);
                updated_student(1:2, uav) = classroom{i}(1:2,uav) + r * (teacher(1:2,uav) - T * mean_sol(1:2,uav));
                updated_student(3:4, uav) = classroom{i}(3:4,uav) + r * (teacher(3:4,uav) - T * mean_sol(3:4,uav));
                updated_student(1:2, uav) = max(-v_UAV_max, min(v_UAV_max, updated_student(1:2, uav))); % Clamp velocity

            end
            classroom{i} = updated_student;
        end

        % Re-evaluate fitness after teaching phase
        new_costs = zeros(1, TLBO.ClassSize);
        for i = 1:TLBO.ClassSize
            student = classroom{i};
            cost = objective_function(student);
            [c, ceq] = nonlcon(student);
            if any(c > 0) || any(ceq ~= 0)
                penalty = penalty_value * sum(c(c > 0));
                cost = cost + penalty;
            end
            new_costs(i) = cost;
        end

        % Get the classroom's best fitness & update the teacher
        [~, new_best_idx] = min(new_costs);
        teacher = classroom{new_best_idx};

        if costs(new_best_idx) < global_best_fitness
            global_best_fitness = costs(new_best_idx);
            global_best_teacher = teacher;
        end

        % Learning phase
        for i = 1:TLBO.ClassSize
            partner_id = randi(TLBO.ClassSize);

            while partner_id == i
                partner_id = randi(TLBO.ClassSize);
            end

            partner = classroom{partner_id};
            updated_student = zeros(4,num_UAVs);

            for uav = 1:num_UAVs
                r = rand;
                if costs(i) < costs(partner_id)
                    updated_student(: , uav) = classroom{i}(:,uav) + r * (classroom{i}(:,uav) - partner(: , uav));
                else
                    updated_student(:, uav) = classroom{i}(:, uav) - r * (classroom{i}(:, uav) - partner(:, uav));

                end
                updated_student(1:2, uav) = max(-v_UAV_max, min(v_UAV_max, updated_student(1:2, uav))); % Clamp velocity
            end
            classroom{i} = updated_student;
        end
    end

    % Reset the class except the teacher
    for i = 1:TLBO.ClassSize
        if i ~= best_idx % Skip the teacher
            % Reinitialize the student by adding random noise to the current solution
            classroom{i}(1:2, :) = classroom{i}(1:2, :) + 0.1 * randn(2, num_UAVs); % Add noise to velocities
            classroom{i}(3:4, :) = classroom{i}(3:4, :) + 0.1 * randn(2, num_UAVs); % Add noise to positions

            % Clamp velocities and positions
            classroom{i}(1:2, :) = max(-v_UAV_max, min(v_UAV_max, classroom{i}(1:2, :))); % Clamp velocities
        end
    end

    % Update fitness plots
    set(plot_per_run, 'YData', bestcostarr, 'XData', 1:length(bestcostarr));
    all_best_cost_log = [all_best_cost_log; global_best_fitness];
    set(plot_overall, 'YData', all_best_cost_log, 'XData', 1:length(all_best_cost_log));

    best_solution = global_best_teacher;
    uav_positions = zeros(num_UAVs,2);
    for uav = 1:num_UAVs
        vx = global_best_teacher(1 , uav);
        vy = global_best_teacher(2 , uav);
        px = global_best_teacher(3 , uav);
        py = global_best_teacher(4 , uav);

        current_position = best_positions(: , uav)';

        direction = [px,py] - current_position;
        if norm(direction) > 0
            direction = direction / norm(direction);
        end
        tentative_position = current_position + (direction * step_size);

        distance_to_restricted = norm(tentative_position - restricted_area_center);
        if distance_to_restricted < restricted_area_radius + 5
            next_position = current_position; % find a fix to this
        else
            next_position = tentative_position;
        end

        best_positions(: , uav) = next_position';
        uav_positions(uav, :) = next_position;
        all_paths{uav} = [all_paths{j}; next_position];
    end

    % Select the "UAV Positions" subplot and update it
    subplot(1, 3, 1); % Ensure we are plotting on the correct subplot
    set(uav_plot, 'XData', uav_positions(:, 1), 'YData', uav_positions(:, 2));

    % Draw the UAV paths dynamically
    for j = 1:num_UAVs
        plot(all_paths{j}(:, 1), all_paths{j}(:, 2), '-', 'LineWidth', 0.5, ...
            'DisplayName', ['UAV ' num2str(j)]);
    end

    drawnow;
    pause(0.001); % Adding a pause to slow down visualization

    % Check if UAVs reached designated targets
    if all(vecnorm(best_positions' - designated_targets', 2, 2) < tolerance)
        disp('All UAVs have reached their designated targets.');

        % Create a new figure for the overall convergence curve
        figure;
        plot(1:length(all_best_cost_log), all_best_cost_log, '-o', 'LineWidth', 1.5, 'MarkerSize', 1.5);
        grid on;
        title('Overall Convergence Curve');
        xlabel('Iterations');
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
d_safe = 2;  % Safe distance between UAVs
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

% Debugging output
fprintf('Constraint values: %s\n', mat2str(c));

% No equality constraints
ceq = [];
end