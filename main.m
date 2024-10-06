clc;
clear all;
close all;

if isempty(gcp('nocreate'))
    parpool;
end
% the UR3e can be placed on a pole so that it could perform weird pose.
% The CRX10iA can help the UR3e reach far goals.
view([135, 30]);
axis([-1.5 1 -0.6 1.5 -0.2 1.5]);
hold on;
global placed;
placed = 1;
global clickedX;
global clickedY;
global estop;
posPlayer = eye(4); posBot = eye(4);
posBot(1:3, 4) = [0.5, 0.6, 0];
posPlayer(1:3, 4) = [-1, 0.6, 0];
global bot;
global player;
% initial bot end effector pos x = 0 y = 0.8 z = 0.3
bot = CRX10IA(posBot);

%initial player end effector pos x = -0.8 y = 0.8 z = 0.3
player = UR3e(posPlayer);
GUI;

% fig2 = figure('Position', [100, 100, 600, 480], 'MenuBar', 'none', ...
%              'Name', 'Control', 'NumberTitle', 'off');
% 
initialMark = [4, 3];
boardGame = board();
drawX(9 - initialMark(1, 2) - 0.5,9 - initialMark(1, 1) - 0.5);

boardGame.placeMark(initialMark(1, 1), 9-initialMark(1, 2), 1);

bot.MoveRobot(bot.homePos);
player.MoveRobot(player.homePos);

bot.currentPoint = 1;
player.currentPoint = 1;

bot.pickAndPlace([initialMark(2)-1, initialMark(1)-1]);

bot.currentPoint = 1;
player.currentPoint = 1;

placed = 0;
state = 1;
while true
    drawnow;

    if (estop == true)
        continue;
    end
    if placed == 1
        
        if (state == 1)
            boardGame.placeMark(9-clickedX, clickedY, 2);
            f = parfeval(@boardGame.getBestMove, 1, 4);
            state = state + 1;
        end
        
        % Only one of the statements below will run, therefore the state in
        % the first statement won't likely affect the second one.
        if (8 - clickedY > 1 || (8-clickedY == 1 && (8 - clickedX == 0 || 8 - clickedX == 7)))
           
            if state == 2
                player.fetchObjToTransfer();
                if (estop == true) continue; end
                state = state + 1;
            end
            if state == 3
                
                bot.receiveTransferAndPlace(player, [8-clickedY 8-clickedX]);
                if (estop == true) continue; end
            end

        else
            if state == 2
                player.pickAndPlace([8-clickedY 8-clickedX]);
                if (estop == true) continue; end
                state = state + 1;
            end
        end
     
        if (estop == true) continue; end

        if state == 3
            drawnow;
            next = fetchOutputs(f);
            boardGame.placeMark(next(1,1), next(1,2), 1);
            drawX(next(1,2) - 0.5, 9 - next(1,1) - 0.5);
            posX = 8 - next(1,2); posY = next(1,1) - 1;
            state = state + 1;
        end

        if (estop == true) continue; end

        if (posX == 0)
            if state == 4
                player.MoveRobot([-0.5, 0.6, 0.5], [90, 90, 90]);
                if (estop == true) continue; end
                player.currentPoint = 1;
                state = state + 1;
            end
            if state == 5
                bot.fetchObjToTransfer();
                if (estop == true) continue; end
                state = state + 1;
            end
            if state == 6
                player.receiveTransferAndPlace(bot, [posX, posY]);
                if (estop == true) continue; end
            end

        else
            if state == 4
                bot.pickAndPlace([posX, posY]);
                if (estop == true) continue; end
            end
        end

         state = 1;
         placed = 0;
    end
end


function drawX(xCenter, yCenter)
    global fig;
    figure(fig);
    hold on;
    lineLength = 0.3; % You can adjust this as needed

    % Draw the 'X' using lines
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter - lineLength, yCenter + lineLength], 'Color', 'r', 'LineWidth', 2);
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter + lineLength, yCenter - lineLength], 'Color', 'r', 'LineWidth', 2);
    hold off;
end



