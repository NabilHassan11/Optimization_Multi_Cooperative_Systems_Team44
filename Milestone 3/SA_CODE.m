clc;
clear;
close all;

% Define SA parameters
SA.Max_num_iter = 1000;
SA.T_init = 500;
SA.T_final = 0.1;
SA.T_current = SA.T_init;
SA.Beta = (SA.T_init - SA.T_final) / SA.Max_num_iter;
SA.alpha = 0.99;
SA.Linear_Cooling = 1; % Use linear cooling
SA.Chk_Probability = 0; % Check acceptance probability
SA.inner_loop_max_iter = 100; % Number of inner iterations before cooling

% Parameters
num_UAVs = 3;        % Number of UAVs
v_UAV_max = 10;      % Max velocity of UAVs (m/s)
d_safe = 1.0;        % Safe distance from restricted areas (m)
T_max = 100;         % Max exploration time (s)
alpha = 1;           % Weight for exploration time
gamma = 0.5;         % Weight for coverage efficiency
delta = 2;           % Weight for designated target reaching
tolerance = 1;       % Distance tolerance for reaching the target

% Randomize restricted area parameters (center and radius)
restricted_area_center = [rand(1) * 250, rand(1) * 250];  % Random restricted area center (x, y)
restricted_area_radius = rand(1) * 20 + 5;                % Random restricted area radius (5m to 25m)

% Function to check if a point is outside the restricted area
isOutsideRestrictedArea = @(point) norm(point(1:2) - restricted_area_center(1:2)) >= (restricted_area_radius + d_safe);

% Randomize designated targets for each UAV (x, y)
designated_targets = zeros(num_UAVs, 2);
for i = 1:num_UAVs
    while true
        target = [rand(1) * 250, rand(1) * 250];  % Random target location
        if isOutsideRestrictedArea(target)
            designated_targets(i, :) = target;
            break;
        end
    end
end

% Randomize initial conditions for UAV positions (x, y)
p_UAV_init = zeros(num_UAVs, 2);
for i = 1:num_UAVs
    while true
        uav_pos = [rand(1) * 250, rand(1) * 250]; % Random initial position
        if isOutsideRestrictedArea(uav_pos)
            p_UAV_init(i, :) = uav_pos;
            break;
        end
    end
end

% Objective function combining exploration time, coverage, and designated targets
objective = @(x) alpha * exploration_time(x) - ...
                 gamma * coverage_efficiency(x) + ...
                 delta * designated_targets_penalty(x, designated_targets);

% Constraints (speed limits, restricted area)
constraints = @(x) nonlcon(x);

% Initial guess for decision variables for each UAV
x0 = [];
for i = 1:num_UAVs
    x0 = [x0, [1, 1, p_UAV_init(i, 1), p_UAV_init(i, 2)]];
end

% Bounds for decision variables for each UAV
bounds.lower = repmat([-v_UAV_max, -v_UAV_max, -Inf, -Inf], 1, num_UAVs);
bounds.upper = repmat([v_UAV_max, v_UAV_max, Inf, Inf], 1, num_UAVs);

% Simulated annealing optimization with nested loop
[best_solution, best_cost] = simulated_annealing(x0, objective, SA, bounds, designated_targets, tolerance, num_UAVs, restricted_area_center, restricted_area_radius);

disp('Best Solution:');
disp(best_solution);
disp('Best Cost:');
disp(best_cost);

% Final Visualization Part
figure;
hold on;
grid on;
axis([0 250 0 250]);
xlabel('X (m)');
ylabel('Y (m)');
title('Final UAV Trajectories, Designated Targets, and Restricted Area');

% Plot designated targets
for i = 1:num_UAVs
    plot(designated_targets(i,1), designated_targets(i,2), 'ro', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Target %d', i));
end

% Plot initial UAV positions
for i = 1:num_UAVs
    plot(p_UAV_init(i,1), p_UAV_init(i,2), 'bo', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('Initial UAV %d', i));
end

% Plot final UAV positions
for i = 1:num_UAVs
    plot(best_solution(4*(i-1)+3), best_solution(4*(i-1)+4), 'go', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', sprintf('Final UAV %d', i));
end

% Plot restricted area (visualized as a circle)
theta = linspace(0, 2*pi, 50);
x_circle = restricted_area_center(1) + restricted_area_radius * cos(theta);
y_circle = restricted_area_center(2) + restricted_area_radius * sin(theta);
fill(x_circle, y_circle, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(x_circle, y_circle, 'r', 'LineWidth', 2);

% Create legend
legend show;
hold off;

%% Objective function components

% Exploration Time Function
function T_explore = exploration_time(x)
    num_UAVs = length(x)/4;
    T_explore = 0;
    for i = 1:num_UAVs
        v_UAV = sqrt(x(4*(i-1)+1)^2 + x(4*(i-1)+2)^2);
        T_explore = T_explore + (100 / v_UAV);
    end
end

% Coverage Efficiency Function
function C_eff = coverage_efficiency(x)
    num_UAVs = length(x)/4;
    area = 62500;
    C_eff = 0;
    for i = 1:num_UAVs
        C_UAV = (x(4*(i-1)+3) + x(4*(i-1)+4));
        C_eff = C_eff + C_UAV;
    end
    C_eff = C_eff / area;
end

% Designated Targets Penalty Function
function penalty = designated_targets_penalty(x, designated_targets)
    num_UAVs = length(x)/4;
    penalty = 0;
    for i = 1:num_UAVs
        p_UAV = [x(4*(i-1)+3), x(4*(i-1)+4)];
        target = designated_targets(i, :);
        distance_to_target = norm(p_UAV - target);
        penalty = penalty + distance_to_target;
    end
end

% Nonlinear constraints function
function [c, ceq] = nonlcon(x)
    num_UAVs = length(x)/4;
    c = [];
    ceq = [];

    for i = 1:num_UAVs
        for j = i+1:num_UAVs
            dist = norm([x(4*(i-1)+3), x(4*(j-1)+4)] - [x(4*(j-1)+3), x(4*(j-1)+4)]);
            c = [c; d_safe - dist];
        end
    end

    for i = 1:num_UAVs
        c = [c; x(4*(i-1)+3) - 250; -x(4*(i-1)+3)];
        c = [c; x(4*(i-1)+4) - 250; -x(4*(i-1)+4)];
    end
end

% Simulated Annealing Function with real-time plotting and nested loop
function [best_solution, best_cost] = simulated_annealing(x0, objective, SA, bounds, designated_targets, tolerance, num_UAVs, restricted_area_center, restricted_area_radius)
    current_solution = x0;
    current_cost = objective(current_solution);
    best_solution = current_solution;
    best_cost = current_cost;
    
    temperature_log = zeros(1, SA.Max_num_iter);
    best_cost_log = zeros(1, SA.Max_num_iter);
    best_positions_log = zeros(SA.Max_num_iter, length(current_solution));
    
    all_targets_reached = false;
    
    % Initial Plot Setup for real-time visualization
    figure;
    hold on;
    grid on;
    axis([0 250 0 250]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('UAV Positions and Designated Targets During Optimization');
    
    colors = ['b', 'g', 'c', 'm', 'y']; % Colors for different UAVs
    % Plot initial UAV positions
    for i = 1:num_UAVs
        plot(current_solution(4*(i-1)+3), current_solution(4*(i-1)+4), 'bo', ...
            'MarkerSize', 10, 'MarkerFaceColor', colors(i), 'DisplayName', sprintf('UAV %d', i));
    end
    
    % Plot designated targets
    for i = 1:num_UAVs
        plot(designated_targets(i,1), designated_targets(i,2), 'ro', ...
            'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Target %d', i));
    end
    
    % Plot restricted area (visualized as a circle)
    theta = linspace(0, 2*pi, 50);
    x_circle = restricted_area_center(1) + restricted_area_radius * cos(theta);
    y_circle = restricted_area_center(2) + restricted_area_radius * sin(theta);
    fill(x_circle, y_circle, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    plot(x_circle, y_circle, 'r', 'LineWidth', 2);
    
    % Simulated Annealing Loop
    for k = 1:SA.Max_num_iter
        for inner_loop = 1:SA.inner_loop_max_iter
            new_solution = current_solution + SA.T_current * (rand(size(current_solution)) - 0.5);
            
            % Ensure new solution respects bounds
            new_solution = max(bounds.lower, min(bounds.upper, new_solution));
            
            new_cost = objective(new_solution);
            
            % Check if new solution is accepted
            if new_cost < current_cost || rand < exp((current_cost - new_cost) / SA.T_current)
                current_solution = new_solution;
                current_cost = new_cost;
                
                if new_cost < best_cost
                    best_solution = new_solution;
                    best_cost = new_cost;
                end
            end
            
            % Break if all targets are reached
            if all_targets_reached
                break;
            end
        end
        
        % Cooling schedule
        SA.T_current = SA.alpha * SA.T_current;
        
        % Store data for analysis and plotting
        temperature_log(k) = SA.T_current;
        best_cost_log(k) = best_cost;
        best_positions_log(k, :) = best_solution;
        
        % Real-time update of the UAV positions after each temperature change
        cla;
        hold on;
        for i = 1:num_UAVs
            plot(best_solution(4*(i-1)+3), best_solution(4*(i-1)+4), 'bo', ...
                'MarkerSize', 10, 'MarkerFaceColor', colors(i), 'DisplayName', sprintf('UAV %d', i));
        end
        
        % Replot designated targets
        for i = 1:num_UAVs
            plot(designated_targets(i,1), designated_targets(i,2), 'ro', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Target %d', i));
        end
        
        % Replot restricted area
        fill(x_circle, y_circle, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        plot(x_circle, y_circle, 'r', 'LineWidth', 2);
        title(sprintf('UAV Positions (Temp: %.2f, Best Cost: %.2f)', SA.T_current, best_cost));
        drawnow;
    end
    
    hold off;
    
    % Plot cost vs iteration
    figure;
    plot(best_cost_log, 'LineWidth', 2);
    xlabel('Iteration');
    ylabel('Best Cost');
    title('Simulated Annealing Optimization Progress');
    
end
