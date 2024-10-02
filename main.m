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
placed = 0;
global row;
global col;
posPlayer = eye(4); posBot = eye(4);
posBot(1:3, 4) = [0.5, 0.6, 0];
posPlayer(1:3, 4) = [-1, 0.6, 0];

% initial bot pos x = 0 y = 0.8 z = 0.3
bot = CRX10IA(posBot);

%initial player pos x = -0.8 y = 0.8 z = 0.3
player = UR3e(posPlayer);
GUI;

initialMark = [4, 3];
boardGame = board();
drawX(9 - initialMark(1, 2) - 0.5,9 - initialMark(1, 1) - 0.5);

boardGame.placeMark(initialMark(1, 1), 9-initialMark(1, 2), 1);

bot.MoveRobot(bot.homePos);
player.MoveRobot(player.homePos);

bot.pickAndPlace([initialMark(2)-1, initialMark(1)-1]);

while true
    drawnow;
    if placed == 1
        boardGame.placeMark(9-row, col, 2);
        f = parfeval(@boardGame.getBestMove, 1, 4);

        if (8 - col > 1 || (8-col == 1 && (8 - row == 0 || 8 - row == 7)))
            player.fetchObjToTransfer();
            bot.receiveTransferAndPlace(player, [8-col 8-row]);
        else
            player.pickAndPlace([8-col 8-row]);
        end

        drawnow;
        placed = 0;
        if (placed == 0)
            next = fetchOutputs(f);
            boardGame.placeMark(next(1,1), next(1,2), 1);
            drawX(next(1,2) - 0.5, 9 - next(1,1) - 0.5);
            posX = 8 - next(1,2); posY = next(1,1) - 1;
            if (posX == 1)
                player.MoveRobot([-0.5, 0.6, 0.5], [90, 90, 90]);
                bot.fetchObjToTransfer();
                player.receiveTransferAndPlace(bot, [posX, posY]);
            else
                bot.pickAndPlace([posX, posY]);
            end
        end
    end
end


function drawX(xCenter, yCenter)
    hold on;
    lineLength = 0.3; % You can adjust this as needed

    % Draw the 'X' using lines
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter - lineLength, yCenter + lineLength], 'Color', 'r', 'LineWidth', 2);
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter + lineLength, yCenter - lineLength], 'Color', 'r', 'LineWidth', 2);

end



