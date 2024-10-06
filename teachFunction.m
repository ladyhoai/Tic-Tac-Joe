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

    global estop; 
    global bot;
    global player;
    global jointArrayBot;
    global jointArrayPlayer;

    global jointArrayBotValue;
    global jointArrayPlayerValue;

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

