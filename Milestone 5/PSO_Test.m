clc;
clear;
close all;

% PSO Parameters
PSO.SwarmSize = 20;
PSO.MaxIterations = 40; % Max number of iterations per point
PSO.weight = 0.792; % Constant weight
PSO.c1 = 1.5;
PSO.c2 = 1.5;

% Problem Parameters
num_UAVs = 3;
v_UAV_max = 3;
d_safe = 30.0; % Safe distance
tolerance = 3; % Target tolerance
alpha = 1; gamma = 0.5; delta = 9;

% Define restricted area
restricted_area_center = [125, 130];
restricted_area_radius = 15;

% Define initial positions
initial_position = [50, 50];
p_UAV_init = repmat(initial_position, num_UAVs, 1)';

% Define target positions
designated_targets = [200, 70; 100, 200; 200, 200]';

% Objective function
objective_function = @(x) sum(alpha * exploration_time(x)) - ...
    sum(gamma * coverage_efficiency(x)) + ...
    sum(delta * designated_targets_penalty(x, designated_targets));

% Constraints
constraints = @(x) nonlcon(x);

% Initialize swarm
swarm = cell(PSO.SwarmSize, 1);
for i = 1:PSO.SwarmSize
    bird = zeros(4, num_UAVs);
    for j = 1:num_UAVs
        bird(:, j) = [rand * 2 - 1; rand * 2 - 1; p_UAV_init(:, j)];
    end
    swarm{i} = bird;
end

% Initialize personal and global bests
personal_best_costs = inf(PSO.SwarmSize, 1);
personal_best_positions = cell(PSO.SwarmSize, 1);
global_best_cost = inf;
global_best_position = [];

all_best_cost_log = []; % Initialize to store fitness values across runs
bestcostarr = [];

% Visualization setup
figure;

% Subplot for UAV positions
subplot(1, 3, 1);
hold on;
title('UAV Positions');
axis([0 250 0 250]);
viscircles(restricted_area_center, restricted_area_radius, 'LineStyle', '--', 'Color', 'r');
plot(initial_position(1), initial_position(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Start Position');
plot(designated_targets(1, :), designated_targets(2, :), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Targets');
uav_plot = plot(nan(1, num_UAVs), nan(1, num_UAVs), 'ks', 'MarkerSize', 8, 'DisplayName', 'UAVs');

% Subplot for best fitness per PSO run
subplot(1, 3, 2);
hold on;
title('Fitness Per PSO Run');
xlabel('Swarm');
ylabel('Best Fitness');
plot_per_run = plot(nan, nan, 'LineWidth', 2, 'DisplayName', 'Best Fitness Per Run');
grid on;

% Subplot for overall fitness across PSO runs
subplot(1, 3, 3);
hold on;
title('Overall Fitness Over Iterations');
xlabel('Iterations');
ylabel('Best Fitness');
plot_overall = plot(nan, nan, 'LineWidth', 2, 'DisplayName', 'Overall Best Fitness');
grid on;

% Initialize UAV positions
uav_positions = zeros(num_UAVs, 2); % Store [x, y] for each UAV
for j = 1:num_UAVs
    uav_positions(j, :) = p_UAV_init(:, j)';
end

% Initialize UAV paths
all_paths = cell(num_UAVs, 1);
for j = 1:num_UAVs
    all_paths{j} = uav_positions(j, :);
end

% Main PSO loop
while true
    best_cost_log = zeros(PSO.MaxIterations, 1); % Track fitness per iteration in the current run
    penalty_value = 2e3; % Penalty for constraint violation

    for iter = 1:PSO.MaxIterations
        % Evaluate fitness for each particle
        costs = zeros(PSO.SwarmSize, 1);
        for i = 1:PSO.SwarmSize
            bird = swarm{i};
            cost = objective_function(bird);

            % Apply constraints and penalties
            [c, ceq] = nonlcon(bird);
            if any(c > 0) || any(ceq ~= 0)
                penalty = penalty_value * sum(c(c > 0));
                cost = cost + penalty;
            end
            costs(i) = cost;

            % Update personal bests
            if cost < personal_best_costs(i)
                personal_best_costs(i) = cost;
                personal_best_positions{i} = bird;
            end
        end

        % Update global best
        [min_cost, min_index] = min(personal_best_costs);
        if min_cost < global_best_cost
            global_best_cost = min_cost;
            global_best_position = personal_best_positions{min_index};
        end

        % PSO velocity and position update
        for i = 1:PSO.SwarmSize
            bird = swarm{i};
            for j = 1:num_UAVs
                v_current = bird(1:2, j);
                p_current = bird(3:4, j);

                r1 = rand;
                r2 = rand;
                cognitive = PSO.c1 * r1 * (personal_best_positions{i}(3:4, j) - p_current);
                social = PSO.c2 * r2 * (global_best_position(3:4, j) - p_current);

                v_new = PSO.weight * v_current + cognitive + social;
                v_new = max(min(v_new, v_UAV_max), -v_UAV_max);
                p_new = p_current + v_new;

                bird(1:2, j) = v_new;
                bird(3:4, j) = p_new;
            end
            swarm{i} = bird;
        end

        best_cost_log(iter) = global_best_cost;
    end

    % Update overall fitness log
    all_best_cost_log = [all_best_cost_log; best_cost_log];
    bestcostarr = [bestcostarr; best_cost_log];

    % Update fitness plot
    subplot(1, 3, 2);
    set(plot_per_run, 'XData', 1:PSO.MaxIterations, 'YData', best_cost_log);
    subplot(1, 3, 3);
    set(plot_overall, 'XData', 1:length(all_best_cost_log), 'YData', all_best_cost_log);
    
    % Move UAVs one step in the direction of the global best
    for j = 1:num_UAVs
        direction = global_best_position(3:4, j)' - uav_positions(j, :);
        step = direction / norm(direction); % Normalize for constant step size
        uav_positions(j, :) = uav_positions(j, :) + step; % Move by 1 step
        all_paths{j} = [all_paths{j}; uav_positions(j, :)];
    end

    % Update path visualization
    subplot(1, 3, 1);
    set(uav_plot, 'XData', uav_positions(:, 1), 'YData', uav_positions(:, 2));
    for j = 1:num_UAVs
        plot(all_paths{j}(:, 1), all_paths{j}(:, 2), '-', 'LineWidth', 1.5, ...
            'DisplayName', ['UAV ' num2str(j)]);
    end

    drawnow;
    pause(0.1);

    % Check if UAVs reached targets
    if all(vecnorm(uav_positions - designated_targets', 2, 2) < tolerance)
        disp('All UAVs have reached their designated targets.');
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
d_safe = 30.0;  % Safe distance between UAVs
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