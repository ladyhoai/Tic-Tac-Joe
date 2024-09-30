clc;
clear all;
close all;
% the UR3e can be placed on a pole so that it could perform weird pose.
% The CRX10iA can help the UR3e reach far goals.
view([135, 30]);
axis auto;
hold on;
global placed;
placed = 0;
global row;
global col;
posPlayer = eye(4); posBot = eye(4);
posBot(1:3, 4) = [0.5, 0.6, 0];
posPlayer(1:3, 4) = [-1, 0.6, 0];

bot = CRX10IA(posBot);
player = UR3e(posPlayer);
GUI;

initialMark = [4, 3];
boardGame = board();
drawX(9 - initialMark(1, 2) - 0.5,9 - initialMark(1, 1) - 0.5);

boardGame.placeMark(initialMark(1, 1), 9-initialMark(1, 2), 1);
bot.pickAndPlace([initialMark(2)-1, initialMark(1)-1]);
axis manual;

while true
    drawnow;
    if placed == 1
        boardGame.placeMark(9-row, col, 2);

        if (8 - col > 2)
            player.fetchObjToTransfer();
            bot.receiveTransferAndPlace(player, [8-col 8-row]);
        else
            player.pickAndPlace([8-col 8-row]);
        end

        drawnow;
        placed = 0;
        if (placed == 0)
            next = boardGame.getBestMove(4);
            boardGame.placeMark(next(1,1), next(1,2), 1);
            drawX(next(1,2) - 0.5, 9 - next(1,1) - 0.5);
            bot.pickAndPlace([8-next(1,2), next(1,1)-1]);
        end
    end
end

% bot.pickAndPlace([8 8]);
% bot.MoveRobot([0, 1.2, 0.5])
% player.pickAndPlace([1 1]);
% bot.pickAndPlace([3 3]);
% player.pickAndPlace([1 2]);
% player.pickAndPlace([0 0]);
% bot.pickAndPlace([0 2]);
function drawX(xCenter, yCenter)
    hold on;
    lineLength = 0.3; % You can adjust this as needed

    % Draw the 'X' using lines
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter - lineLength, yCenter + lineLength], 'Color', 'r', 'LineWidth', 2);
    line([xCenter - lineLength, xCenter + lineLength], ...
         [yCenter + lineLength, yCenter - lineLength], 'Color', 'r', 'LineWidth', 2);

end



