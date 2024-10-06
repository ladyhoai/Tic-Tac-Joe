function teachFunction()
    
    global figControl;
    global estop; 
    global bot;
    global player;
    global jointArrayBot;
    global jointArrayPlayer;
    
    global jointArrayBotValue;
    global jointArrayPlayerValue;
    
    global sliderArrayBot;
    global sliderArrayPlayer;
    
    jointArrayBot = zeros(1, 7);
    jointArrayPlayer = zeros(1, 7);
    hold on;
    
    virtualJoystick(figControl, [-0.7, 0.7, 0.6, 0.7]);
    axis off;
    
    
    % Number of sliders
    num_sliders = 6;
    
    % Adjusted dimensions and spacing to fit within the figure
    slider_width = 100;   % Width for each horizontal slider
    slider_height = 20;   % Reduced height for horizontal sliders
    label_height = 15;    % Reduced height for each label
    spacing = 10;         % Reduced spacing between sliders
    initial_x_position = 20;  % Leftmost slider position
    right_initial_x_position = 500;  % Rightmost slider position (mirrored)
    initial_y_position = 280; % Shifted down from 300 by 20 pixels
    label_offset = 0;    % Space between the label and the slider
    
    % Create the Up arrow button for player
    hButtonUpP = uicontrol('Style', 'pushbutton', 'String', '↑',...
        'Position', [380, 100, 50, 50],...  % [x, y, width, height]
        'BackgroundColor', 'g',...  % Set background color to green
        'FontSize', 12, ...
        'UserData', 'PU', ...
        'Callback', @UpDownCallBack);            % Set font size
    
    % Create the Down arrow button for player
    hButtonDownP = uicontrol('Style', 'pushbutton', 'String', '↓',...
        'Position', [380, 40, 50, 50],...  % [x, y, width, height]
        'BackgroundColor', 'g',...  % Set background color to green
        'FontSize', 12, ...
        'UserData', 'PD',...
        'Callback', @UpDownCallBack);            % Set font size
    
    % Create the Up arrow button for bot
    hButtonUpB = uicontrol('Style', 'pushbutton', 'String', '↑',...
        'Position', [190, 100, 50, 50],...  % [x, y, width, height]
        'BackgroundColor', 'g',...  % Set background color to green
        'FontSize', 12, ...
        'UserData', 'BU', ...
        'Callback', @UpDownCallBack);            % Set font size
    
    % Create the Down arrow button
    hButtonDownB = uicontrol('Style', 'pushbutton', 'String', '↓',...
        'Position', [190, 40, 50, 50],...  % [x, y, width, height]
        'BackgroundColor', 'g',...  % Set background color to green
        'FontSize', 12, ...
        'UserData', 'BD', ...
        'Callback', @UpDownCallBack);            % Set font size
    
    for i = 1:num_sliders
        % Calculate slider position (horizontally aligned)
        slider_position_x = initial_x_position;
        slider_position_y = initial_y_position - (i-1) * (slider_height + spacing + label_height);
        label_position_y = slider_position_y + slider_height + label_offset;  % Position label above slider
    
        % Create each horizontal slider
        sliderArrayBot(end+1) = uicontrol( ...
              'Style', 'slider', ...
              'Position', [slider_position_x, slider_position_y, slider_width, slider_height], ...
              'Parent', figControl, ...
              'Min', -pi, ...
              'Max', pi, ...
              'Value', 0, ...
              'UserData', compose('L%d', i), ...
              'Callback', @sliderCallback);  % Attach to the main figure
    
        % Create text label to display joint values to the right of the slider
        jointArrayBotValue(end+1) = uicontrol('Style', 'text', ...
                'String', num2str(0), ...  % Initial value displayed
                'Position', [slider_position_x + slider_width + 10, slider_position_y, 40, slider_height], ...  % Adjust position to the right of the slider
                'HorizontalAlignment', 'left', ...  % Left align the text
                'Parent', figControl);
        
        % Create text label for each slider (Joint 1, Joint 2, ..., Joint 6)
        uicontrol('Style', 'text', ...
                  'String', ['Joint ' num2str(i)], ...
                  'Position', [slider_position_x, label_position_y, slider_width, label_height], ...
                  'HorizontalAlignment', 'center', ...  % Center align the text
                  'Parent', figControl);  % Attach to the main figure
    end
    
    for i = 1:num_sliders
        % Calculate slider position (horizontally aligned)
        right_slider_position_x = right_initial_x_position;
        right_slider_position_y = initial_y_position - (i-1) * (slider_height + spacing + label_height);
        
        % Position label 20 pixels lower than usual
        right_label_position_y = right_slider_position_y + slider_height + label_offset;
    
        % Create each horizontal slider on the right side
        sliderArrayPlayer(end+1) = uicontrol( ...
              'Style', 'slider', ...
              'Position', [right_slider_position_x, right_slider_position_y, slider_width, slider_height], ...
              'Parent', figControl, ...
              'Min', -pi, ...
              'Max', pi, ...
              'Value', 0, ...
              'UserData', compose('R%d', i), ...
              'Callback', @sliderCallback);  % Attach to the main figure
    
        jointArrayPlayerValue(end+1) = uicontrol('Style', 'text', ...
                  'String', num2str(0), ...
                  'Position', [right_slider_position_x - 50, right_slider_position_y, 40, slider_height], ...
                  'HorizontalAlignment', 'right', ...  % Center align the text
                  'Parent', figControl);  % Attach to the main figure
        
        % Create text label for each right slider (Joint 1, Joint 2, ..., Joint 6)
        uicontrol('Style', 'text', ...
                  'String', ['Joint ' num2str(i)], ...
                  'Position', [right_slider_position_x, right_label_position_y, slider_width, label_height], ...
                  'HorizontalAlignment', 'center', ...  % Center align the text
                  'Parent', figControl);  % Attach to the main figure
    end
    
    function sliderCallback(hObject, ~)
        % Get the current value of the slider
        sliderValue = get(hObject, 'Value');
        getStr = get(hObject, 'UserData');
        joint = char(getStr(1));
        jointNumber = str2double(joint(2));
        if (estop == true) 
            if (joint(1) == 'L')
                jointArrayBot(jointNumber) = sliderValue;
                bot.animateWithGripper(jointArrayBot);
                set(jointArrayBotValue(jointNumber), 'String', num2str(sliderValue, 3));
    
            elseif (joint(1) == 'R')
                jointArrayPlayer(jointNumber) = sliderValue;
                player.animateWithGripper(jointArrayPlayer);
                set(jointArrayPlayerValue(jointNumber), 'String', num2str(sliderValue, 3));
            end
            % Display the value in the command window (or use it as needed)
            % fprintf('Slider Value: %.2f\n', sliderValue);
        else
            disp('Please stop the system first before teaching');
        end
    end
    
    function UpDownCallBack(src, ~)
        if estop == true
            whichButton = src.UserData;
            if (whichButton(1) == 'B')
                Tr = bot.model.fkine(jointArrayBot);
                currentPoint = transl(Tr);
                if whichButton(2) == 'U'
                    currentPoint(3) = currentPoint(3) + 0.02;
                elseif whichButton(2) == 'D'
                    currentPoint(3) = currentPoint(3) - 0.02;
                end
            
                nextJointState = bot.calculateRMRC(currentPoint', tr2rpy(Tr.T, 'deg'), jointArrayBot, 20);
                jointArrayBot = nextJointState(end-1, :);
                bot.animateWithGripper(jointArrayBot);
                for y = 1:6
                    set(sliderArrayBot(y), 'Value', jointArrayBot(y));
                    set(jointArrayBotValue(y), 'String', num2str(jointArrayBot(y), 3));
                end
        
            elseif whichButton(1) == 'P'
                Tr = player.model.fkine(jointArrayPlayer);
                currentPoint = transl(Tr);
                if whichButton(2) == 'U'
                    currentPoint(3) = currentPoint(3) + 0.02;
                elseif whichButton(2) == 'D'
                    currentPoint(3) = currentPoint(3) - 0.02;
                end
                
                nextJointState = player.calculateRMRC(currentPoint', tr2rpy(Tr.T, 'deg'), jointArrayPlayer, 20);
                jointArrayPlayer = nextJointState(end-1, :);
                player.animateWithGripper(jointArrayPlayer);
                for y = 1:6
                    set(sliderArrayPlayer(y), 'Value', jointArrayPlayer(y));
                    set(jointArrayPlayerValue(y), 'String', num2str(jointArrayPlayer(y), 3));
                end
            end

        else
            disp('Stop system first before teaching');
        end
    end
end
