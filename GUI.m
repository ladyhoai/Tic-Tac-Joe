
% Create a highlighting 4x4 Tic Tac Toe grid

% Create a figure for the grid
global fig;
fig = figure('Position', [100, 100, 600, 480], 'MenuBar', 'none', ...
             'Name', 'Highlighting 4x4 Tic Tac Toe Grid', 'NumberTitle', 'off');
global mapborder;
global row;
global col;

mapborder = [115 506 52 440];
hold on;
axis equal;
xlim([0, 8]);
ylim([0, 8]);

% Create a cell array to store rectangle handles
rectHandles = gobjects(8, 8); % Preallocate handles

% Draw grid and store rectangle handles
for i = 1:8
    for j = 1:8
        % Draw a rectangle for each cell
        rectHandles(i, j) = fill([j-1, j-1, j, j], [i-1, i, i, i-1], 'w'); 
    end
end

% Set the axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Highlighting 4x4 Tic Tac Toe Grid');

% Hide axis ticks
set(gca, 'XTick', [], 'YTick', []);

% Set the callback function for mouse movement
set(fig, 'WindowButtonMotionFcn', @(src, event) mouseMove(rectHandles));
set(fig, 'WindowButtonDownFcn', @(src, event) mouseClickCallback(src));

hold off;

% Function to be called when the mouse moves
function mouseMove(rectHandles)
    global fig;
    global mapborder;
    global row;
    global col;
    % Get the current point of the mouse in figure coordinates
    mousePos = get(fig, 'CurrentPoint');
    x = mousePos(1, 1);  % X position
    y = mousePos(1, 2);  % Y position
    
    % Determine which grid cell the mouse is over
    col = floor(mapValue(x, mapborder(1, 1), mapborder(1, 2), 1, 8));  % Column index (1 to 4)
    row = floor(mapValue(y, mapborder(1, 3), mapborder(1, 4), 1, 8));  % Row index (1 to 4)
    % Reset all cells to white
    for r = 1:8
        for c = 1:8
            set(rectHandles(r, c), 'FaceColor', 'w');  % Reset color to white
        end
    end
    
    % Highlight the hovered cell if it's within bounds
    if row >= 1 && row <= 8 && col >= 1 && col <= 8
        set(rectHandles(row, col), 'FaceColor', [0.8, 0.8, 0.8]);  % Light gray
        % fprintf('Row: %d, Column: %d\n', row, col);
    end
end

% Callback function to handle mouse clicks
function mouseClickCallback(~)
    global row;
    global col;
    global placed;
    fprintf('Row: %d, Column: %d\n',8 - row, 8 - col);
    % Define the length of the 'X' lines
    xCenter = col - 0.5;
    yCenter = row - 0.5;
    hold on;

    % lineLength = 0.3; % You can adjust this as needed
    % 
    % % Draw the 'X' using lines
    % line([xCenter - lineLength, xCenter + lineLength], ...
    %      [yCenter - lineLength, yCenter + lineLength], 'Color', 'r', 'LineWidth', 2);
    % line([xCenter - lineLength, xCenter + lineLength], ...
    %      [yCenter + lineLength, yCenter - lineLength], 'Color', 'r', 'LineWidth', 2);
    % 
    % Parameters for the circle
    theta = linspace(0, 2 * pi, 100); % Angle parameter for circle
    radius = 0.3; % Adjust as needed

    % Calculate circle coordinates
    xCircle = radius * cos(theta) + xCenter;
    yCircle = radius * sin(theta) + yCenter;

    % Draw the circle using the calculated coordinates
    plot(xCircle, yCircle, 'Color', 'b', 'LineWidth', 2);
    placed = 1;

end

function mappedValue = mapValue(x, a, b, c, d)
    % Map x from range [a, b] to range [c, d]
    mappedValue = c + ((x - a) * (d - c)) / (b - a);
end

