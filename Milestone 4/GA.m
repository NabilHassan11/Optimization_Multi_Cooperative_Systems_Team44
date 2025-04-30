clc;
clear;
close all;

% GA Parameters
GA.NumGenerations = 40;       % Number of generations for each step
GA.PopSize = 20;
GA.EliteRatio = 0.1;
GA.CrossoverRatio = 0.6;
GA.MutationRatio = 1 - GA.EliteRatio - GA.CrossoverRatio;
GA.Alpha = 0.4;
GA.NoiseScale = 0.35;
step_size = 3;   % Constant step size for UAV movement

% Problem Parameters
num_UAVs = 3;
v_UAV_max = 3;
d_safe = 1.0;
T_max = 100;
alpha = 1;
gamma = 0.5;
delta = 9;
tolerance = 3 ;

% Define restricted area
restricted_area_center = [125, 125];
restricted_area_radius = 10;

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
population = cell(GA.PopSize, 1);
for i = 1:GA.PopSize
    individual = zeros(4, num_UAVs);  % 3x4 matrix for each individual
    for j = 1:num_UAVs
        vx = rand * 2 - 1;  % Independent velocity in x for UAV j
        vy = rand * 2 - 1;  % Independent velocity in y for UAV j
        px = p_UAV_init(1, j);
        py = p_UAV_init(2, j);
        individual(:, j) = [vx; vy; px; py];
    end
    population{i} = individual;
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
title('Fitness Value per GA Run');
xlabel('Generation');
ylabel('Best Fitness');
plot_per_run = plot(nan, nan, 'LineWidth', 2);

% Subplot for overall fitness across all GA runs
subplot(1, 3, 3);
hold on;
title('Overall Fitness Function Over Generations Across GA Runs');
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

% Main loop until all targets are reached or max steps
while true
    % Reset best cost array at the start of each GA run
    bestcostarr = [];

    % Run the Genetic Algorithm for a fixed number of generations
    best_cost_log = zeros(GA.NumGenerations, 1);
    penalty_value = 1e3; % Adjust penalty value to a moderate amount

    for gen = 1:GA.NumGenerations
        % Evaluate fitness for each individual
        costs = zeros(GA.PopSize, 1);
        for i = 1:GA.PopSize
            individual = population{i};

            % Calculate the objective function value for the individual
            cost = objective_function(individual);

            % Evaluate constraints
            [c, ceq] = nonlcon(individual);

            % Apply penalty if constraints are violated
            if any(c > 0) || any(ceq ~= 0)
                penalty = penalty_value * sum(c(c > 0));
                cost = cost + penalty;
            end
            costs(i) = cost;
        end

        % Select elite individuals
        [~, sorted_idx] = sort(costs);
        elite_count = round(GA.EliteRatio * GA.PopSize);
        elite_population = population(sorted_idx(1:elite_count));

        % Record the best cost of the current generation
        best_cost_log(gen) = costs(sorted_idx(1));
        bestcostarr = [bestcostarr; costs(sorted_idx(1))];

        % Generate new individuals with crossover and mutation
        new_population = elite_population;
        while numel(new_population) < GA.PopSize
            parent1 = elite_population{randi([1 elite_count])};
            parent2 = elite_population{randi([1 elite_count])};

            % Perform crossover and mutation
            child1 = crossover(parent1, parent2, GA.Alpha);
            child2 = crossover(parent2, parent1, GA.Alpha);

            if rand < GA.MutationRatio
                child1 = mutate(child1, GA.NoiseScale);
                child2 = mutate(child2, GA.NoiseScale);
            end

            % Add new individuals to population
            new_population{end+1} = child1;
            new_population{end+1} = child2;
        end
        population = new_population(1:GA.PopSize);
    end

    % Update fitness plots
    set(plot_per_run, 'YData', bestcostarr, 'XData', 1:length(bestcostarr));
    all_best_cost_log = [all_best_cost_log; best_cost_log];
    set(plot_overall, 'YData', all_best_cost_log, 'XData', 1:length(all_best_cost_log));

    % Retrieve best solution
    best_chromosome = population{sorted_idx(1)};

    % Move each UAV towards the best next position by the step size
    uav_positions = zeros(num_UAVs, 2);
    for j = 1:num_UAVs
        vx = best_chromosome(1, j);
        vy = best_chromosome(2, j);
        px = best_chromosome(3, j);
        py = best_chromosome(4, j);
        current_position = best_positions(:, j)';

        % Calculate the direction to move towards the target position
        direction = [px, py] - current_position;
        if norm(direction) > 0
            direction = direction / norm(direction); % Normalize direction
        end

        % Tentative next position
        tentative_position = current_position + direction * step_size;

        % Check constraints for the tentative position
        distance_to_restricted = norm(tentative_position - restricted_area_center);
        if distance_to_restricted < restricted_area_radius
            next_position = current_position; % Stay in place
        else
            next_position = tentative_position; % Update position
        end

        % Store updated position and log to path
        best_positions(:, j) = next_position';
        uav_positions(j, :) = next_position;
        all_paths{j} = [all_paths{j}; next_position]; % Log the path
    end

    % Select the "UAV Positions" subplot and update it
    subplot(1, 3, 1); % Ensure we are plotting on the correct subplot
    set(uav_plot, 'XData', uav_positions(:, 1), 'YData', uav_positions(:, 2));

    % Draw the UAV paths dynamically
    for j = 1:num_UAVs
        plot(all_paths{j}(:, 1), all_paths{j}(:, 2), '-', 'LineWidth', 1.5, ...
             'DisplayName', ['UAV ' num2str(j)]);
    end

    drawnow;
    pause(0.1); % Adding a pause to slow down visualization

    % Check if UAVs reached designated targets
    if all(vecnorm(best_positions' - designated_targets', 2, 2) < tolerance)
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
    restricted_area_radius = 10;

    % Initialize constraints
    c = zeros(num_UAVs, 1);

    % Check each UAV's distance to restricted area
    for i = 1:num_UAVs
        distance_to_restricted = norm(x(3:4, i)' - restricted_area_center) - (restricted_area_radius + 20);
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

function child = crossover(parent1, parent2, alpha)
    child = alpha * parent1 + (1 - alpha) * parent2;
end

function mutated = mutate(individual, noise_scale)
    mutated = individual + noise_scale * randn(size(individual));
end

K = 7;  % Number of runs
best_fitness_values = zeros(K, 1);  % Array to store best fitness of each run

for run = 1:K
    fprintf('Run %d/%d\n', run, K);
    % Execute the main script here (assume the script is saved as a function)
    untitled6;  % Replace 'main_script' with the name of your function/script file
    
    % Retrieve the best fitness from this run
    best_fitness_values(run) = all_best_cost_log(end); % Assuming all_best_cost_log contains best fitness
    
    % Clear variables except K and results
    clearvars -except K best_fitness_values
end

% Calculate the statistics
std_dev = std(best_fitness_values);  % Standard deviation
mean_best_fitness = mean(best_fitness_values);  % Mean of best fitness

% Display the results
fprintf('Standard Deviation of Best Fitness: %.4f\n', std_dev);
fprintf('Mean of Best Fitness: %.4f\n', mean_best_fitness);
fprintf('Best Fitness Values for Each Run: %s\n', mat2str(best_fitness_values));
