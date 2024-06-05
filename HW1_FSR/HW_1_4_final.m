clc; clear;
% Load the map
load('image_map.mat');


% Define the start and goal points
start_point = [30, 125];
goal_point = [135, 400];

% Set maximum number of iterations
max_iterations = 400;

% Define parameters
delta = 5; % Step size for extending the tree

% Initialize tree
tree = [start_point, 0]; % Tree structure: [x, y, parent_index]

% Initialize plot
figure;
imshow(image_map);
hold on;

% Plot start and goal points
plot(start_point(2), start_point(1), 'ro', 'MarkerSize', 10);
plot(goal_point(2), goal_point(1), 'go', 'MarkerSize', 10);

% RRT algorithm
for i = 1:max_iterations
    
    % Generate random point
    random_point = [randi(size(image_map, 1)), randi(size(image_map, 2))];

    % Find nearest point in the tree
    distances = sqrt(sum((tree(:, 1:2) - random_point).^2, 2));
    [~, nearest_index] = min(distances);
    nearest_point = tree(nearest_index, 1:2);
    
    % Extend tree towards the random point
    direction = (random_point - nearest_point) / norm(random_point - nearest_point);
    new_point = nearest_point + delta * direction;
    
    % Check if new point is valid (not in collision)
    if image_map(round(new_point(1)), round(new_point(2))) == 0
        continue; % Skip this iteration if collision occurs
    end
    
    % Check for collision along the path
    path_points = [linspace(nearest_point(1), new_point(1), 10)', linspace(nearest_point(2), new_point(2), 10)'];
    collision = false;
    for j = 1:size(path_points, 1)
        if image_map(round(path_points(j, 1)), round(path_points(j, 2))) == 0
            collision = true;
            break;
        end
    end
    
    % Add new point to the tree if no collision along the path
    if ~collision
        % Add new point to the tree
        tree = [tree; new_point, nearest_index];
        
        % Plot new point
        plot(new_point(2), new_point(1), 'b.');
        
        % Plot segment from nearest point to new point
            plot([nearest_point(2), new_point(2)], [nearest_point(1), new_point(1)], 'b');


 end

   end

% Check if solution found
if i == max_iterations
    distances_2 = sqrt(sum((tree(:, 1:2) - goal_point).^2, 2));
    [~, nearest_index_2] = min(distances_2);
    nearest_point_2 = tree(nearest_index_2, 1:2);
    new_point= nearest_point_2;

% check collision
collision = false;
path_points = [linspace(goal_point(1), new_point(1), 10)', linspace(goal_point(2), new_point(2), 10)'];
    for j = 1:size(path_points, 1)
        if image_map(round(path_points(j, 1)), round(path_points(j, 2))) == 0
            collision = true;
            disp('Collision at last: End')
            break;
        end
    end
    
    % Add new point to the tree if no collision along the path
    if ~collision
       plot([nearest_point_2(2), goal_point(2)], [nearest_point_2(1), goal_point(1)], 'b');
        disp('Forced in the last iteration the nearest point to goal point')

    end     
    
end

if ~collision
% Plot path
    path = [new_point];
    while norm(path(end,:) - start_point) > 1
        parent_index = tree(find(ismember(tree(:,1:2),path(end,:),'rows')),3);
        path = [path; tree(parent_index,1:2)];
    end
    plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    plot([new_point(2), goal_point(2)], [new_point(1), goal_point(1)], 'r-', 'LineWidth', 2);

else
    disp('Invalid path: Restart')
end