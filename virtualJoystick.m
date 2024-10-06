function virtualJoystick(fig, pos)
    % Check if the provided figure handle is valid
    if ~ishandle(fig) || ~strcmp(get(fig, 'Type'), 'figure')
        error('Invalid figure handle provided.');
    end

    % Use the provided figure for plotting
    figure(fig);
    global bot;
    global player;

    global jointArrayBot;
    global jointArrayPlayer;

    global jointArrayBotValue;
    global jointArrayPlayerValue;

    global sliderArrayBot;
    global sliderArrayPlayer;

    global estop;
    
    % Create the joystick area
    axesHandle = gca; % Get current axes handle
  
    axis(axesHandle, 'equal');
    axis(axesHandle, [-1 1 -1 1]);
    hold(axesHandle, 'on');

    % Set the new center for the joystick
    joystickCenterX = pos(1);  % Desired X position for the joystick center
    joystickCenterY = pos(2);  % Desired Y position for the joystick center

    joystickCenterX2 = pos(3);  % Desired X position for the joystick center
    joystickCenterY2 = pos(4);  % Desired Y position for the joystick center


    % Draw a circular boundary for the joystick
    theta = linspace(0, 2*pi, 28.6);
    plot(axesHandle, joystickCenterX + 0.3 * cos(theta), joystickCenterY + 0.3 * sin(theta), 'k--'); % Joystick boundary
    joystickHandle = plot(axesHandle, joystickCenterX, joystickCenterY, 'ro', 'MarkerSize', 20); % Joystick position

    plot(axesHandle, joystickCenterX2 + 0.3 * cos(theta), joystickCenterY2 + 0.3 * sin(theta), 'k--'); % Joystick boundary
    joystickHandle2 = plot(axesHandle, joystickCenterX2, joystickCenterY2, 'ro', 'MarkerSize', 20); % Joystick position

    
    % State variable to check if the joystick is being moved
    isMoving = false;
    isMoving2 = false;

    % Mouse click function
    function mouseClick(~, ~)

        if estop == true
            pt = get(axesHandle, 'CurrentPoint'); % Get mouse position
            x = pt(1, 1);
            y = pt(1, 2);
            
            % Check if the click is within the joystick area
            if norm([x - joystickCenterX, y - joystickCenterY]) <= 0.5
                isMoving = true; % Set flag to true if inside joystick area
                % Update joystick position
                set(joystickHandle, 'XData', x, 'YData', y);
    
            elseif norm([x - joystickCenterX2, y - joystickCenterY2]) <= 0.5
                isMoving2 = true; % Set flag to true if inside joystick area
                % Update joystick position
                set(joystickHandle2, 'XData', x, 'YData', y);
            end

        else
            disp('Stop the system first before controlling');
        end

    end

    

    % Mouse release function
    function mouseRelease(~, ~)
        % Reset joystick to the center
        set(joystickHandle, 'XData', joystickCenterX, 'YData', joystickCenterY);
        set(joystickHandle2, 'XData', joystickCenterX2, 'YData', joystickCenterY2);
        isMoving = false; % Reset the flag on release
        isMoving2 = false;

    end

    function checkStateAndMoveRobot(~, ~)
        if estop == true
            if isMoving
                
                xBot = mapValue(get(joystickHandle, 'xData'), -1, -0.4, -0.008, 0.008);
                yBot = mapValue(get(joystickHandle, 'yData'), 0.4, 1, -0.008, 0.008);
                
                Tr = bot.model.fkine(jointArrayBot);
                currentPoint = transl(Tr);
                nextJointState = bot.calculateRMRC([currentPoint(1) + xBot, currentPoint(2) + yBot, currentPoint(3)]', tr2rpy(Tr.T, 'deg'), jointArrayBot, 20);
                jointArrayBot = nextJointState(end-1, :);
                bot.animateWithGripper(jointArrayBot);
                for i = 1:6
                    set(sliderArrayBot(i), 'Value', jointArrayBot(i));
                    set(jointArrayBotValue(i), 'String', num2str(jointArrayBot(i), 3));
                end
            
            elseif isMoving2
              
                xBot = mapValue(get(joystickHandle2, 'xData'), 0.3, 0.9, -0.008, 0.008);
                yBot = mapValue(get(joystickHandle2, 'yData'), 0.4, 1, -0.008, 0.008);
                
                Tr = player.model.fkine(jointArrayPlayer);
                currentPoint = transl(Tr);
                nextJointState = player.calculateRMRC([currentPoint(1) + xBot, currentPoint(2) + yBot, currentPoint(3)]', tr2rpy(Tr.T, 'deg'), jointArrayPlayer, 20);
                jointArrayPlayer = nextJointState(end-1, :);
                player.animateWithGripper(jointArrayPlayer);
                for i = 1:6
                    set(sliderArrayPlayer(i), 'Value', jointArrayPlayer(i));
                    set(jointArrayPlayerValue(i), 'String', num2str(jointArrayPlayer(i), 3));
                end
            end

        end
    end

    function mappedValue = mapValue(x, a, b, c, d)
        % Map x from range [a, b] to range [c, d]
        mappedValue = c + ((x - a) * (d - c)) / (b - a);
    end

    % Assign mouse callback functions to the figure
    set(fig, 'WindowButtonMotionFcn', @mouseMove, ...
             'WindowButtonUpFcn', @mouseRelease, ...
             'WindowButtonDownFcn', @mouseClick);

    t = timer;
    t.ExecutionMode = 'fixedRate';  % Executes at fixed intervals
    t.Period = 0.2;                % 50 ms interval (0.05 seconds)
    t.TimerFcn = @checkStateAndMoveRobot;  % Function to call on each execution
    
    % Start the timer
    start(t);
end


