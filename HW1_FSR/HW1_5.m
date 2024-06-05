
clc; clear;

% Matrix dimension
n_row = 7;
n_col = 10;

% Matrices initialization
map = zeros(n_row,n_col);
map_with_path= zeros(n_row, n_col);

% Set obstacles cell values to 100
map(1,2)= 100; 
map(1,3)= 100;
map(2,3)= 100;
map(3,3)= 100;
map(4,3)= 100;
map(5,3)= 100;
map(3,5)= 100;
map(4,5)= 100;
map(5,5)= 100;
map(5,6)= 100;
map(7,7)= 100;
map(2,8)= 100;
map(3,8)= 100;
map(4,8)= 100;
map(5,8)= 100;
map(2,9)= 100;
map_with_path(1,2)= 100;
map_with_path(1,3)= 100;
map_with_path(2,3)= 100;
map_with_path(3,3)= 100;
map_with_path(4,3)= 100;
map_with_path(5,3)= 100;
map_with_path(3,5)= 100;
map_with_path(4,5)= 100;
map_with_path(5,5)= 100;
map_with_path(5,6)= 100;
map_with_path(7,7)= 100;
map_with_path(2,8)= 100;
map_with_path(3,8)= 100;
map_with_path(4,8)= 100;
map_with_path(5,8)= 100;
map_with_path(2,9)= 100;
q_G =  0;

% Initialization of starting cell coordinates
start_row = 1;
start_col = 1;

% Counter for the value to assign to adjacent cells
increase_value = 1;

% Variable to define whether to examine 4 or 8 cells
max_direction = 4; %8

% Initialization of cells to examine for increment
parent_cells = [start_row, start_col];

% Define starting cell coordinates for the path
startRow = 1;
startCol = 8;
map_with_path(startRow, startCol)= 11;

% Initialize vector for cell positions
positions = [startRow, startCol];

% Find path to cell with value 0
currentRow = startRow;
currentCol = startCol;


% Continue until all cells have been examined
while ~isempty(parent_cells)

    % New cells to examine for increment
   child_cells = [];
    
    % Iteration through cells to examine
    for i = 1:size(parent_cells, 1)
        start_row = parent_cells(i, 1);
        start_col = parent_cells(i, 2);
        
        % DEFINITION OF ADJACENT CELLS AS: NORTH, SOUTH, EAST, WEST (NORT-EAST,NORTH-WEST,SOUTH-EAST,SOUTH-WEST)
        for direction = 1:max_direction
            if direction == 1 % North
                adj_row = start_row - 1;
                adj_col = start_col;
            elseif direction == 2 % South
                adj_row = start_row + 1;
                adj_col = start_col;
            elseif direction == 3 % East
                adj_row = start_row;
                adj_col = start_col + 1;
            elseif direction == 4 % West
                adj_row = start_row;
                adj_col = start_col - 1;
%             elseif direction == 5 % North-East
%                 adj_row = start_row - 1;
%                 adj_col = start_col + 1;
%             elseif direction == 6 % North-West
%                 adj_row = start_row - 1;
%                 adj_col = start_col - 1;
%             elseif direction == 7 % South-East
%                 adj_row = start_row + 1;
%                 adj_col = start_col + 1;
%             else % South-West
%                 adj_row = start_row + 1;
%                 adj_col = start_col - 1;
            end
            
            % Check if adjacent cell is within matrix bounds
            if (adj_row < 1 || adj_row> size(map,1))
                continue
            elseif (adj_col < 1 || adj_col > size(map,2))
                continue
            elseif (map(adj_row, adj_col) == 0 && map(start_row,start_col)~=100 )  % check collision
      
                % Assign incremented value to adjacent cell
                map(adj_row, adj_col) = increase_value;

                % Add adjacent cell to list of new cells to examine
                child_cells = [child_cells; adj_row, adj_col];

            end
        end
    end
    
    % Increment the value to assign to adjacent cells
    increase_value = increase_value +1 ;
    
    % Update the list of cells to examine for the next increment
    parent_cells = child_cells;

    % Definition of q_G
    map(1,1)=q_G;

end

% Display the obtained matrix
disp('map:');
disp(map);

while map(currentRow, currentCol) ~= 0
    % Define coordinates of adjacent cells
    neighbors = [
        currentRow-1, currentCol;  % North
        currentRow+1, currentCol;  % South
        currentRow, currentCol-1;  % West
        currentRow, currentCol+1;   % East
%         currentRow-1, currentCol+1; % North-East
%         currentRow-1, currentCol-1; % North-West
%         currentRow+1, currentCol+1; % South-East
%         currentRow+1, currentCol-1  % South-West
    ];
    
    % Find adjacent cell with the lowest value
    minValue = inf;
    minIndex = [];
    for i = 1:size(neighbors, 1)
        row = neighbors(i, 1);
        col = neighbors(i, 2);
        if row >= 1 && row <= size(map, 1) && col >= 1 && col <= size(map, 2)
            if map(row, col) < minValue
                minValue = map(row, col);
                minIndex = [row, col];
            end
        end
    end
    
    % Update current cell coordinates
    currentRow = minIndex(1);
    currentCol = minIndex(2);
    
    % Add current cell coordinates to positions vector
    positions = [positions; currentRow, currentCol];
    map_with_path(currentRow,currentCol)= 11;
end

% Display positions of cells with decreasing values
disp('Positions of cells with decreasing values:');
disp(positions);
disp ('Path');
disp(map_with_path);
