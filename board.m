classdef board < handle
    properties(Access=public)
        availableMove = 8*8;
        boardGrid = [0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;
                     0 0 0 0 0 0 0 0;];
        positions;
    end

    methods

        function bestMove = getBestMove(self, maxDepth)
            % Initialize bestMove with -1 (invalid) and bestValue as negative infinity
            bestMove = [-1, -1];
            bestValue = -Inf;
            
            self.positions = self.findNearbyZeros(self.boardGrid);
            % disp(self.positions);
            
            % Loop through each cell on the board
            for idx = 1:size(self.positions, 1)
                row = self.positions(idx, 1);
                col = self.positions(idx, 2);
                % Check if the tile is empty (represented by 0 in your case)
                if self.boardGrid(row, col) == 0
                    % Simulate the move by setting the tile to 'X' (represented by 1)
                    self.boardGrid(row, col) = 1;
                    
                    % Call the minimax function to evaluate the move
                    moveValue = self.miniMax(maxDepth, -Inf, Inf, false);
                    
                    % Undo the move (set the tile back to blank)
                    self.boardGrid(row, col) = 0;
                    
                    % If the current move is better, update bestMove and bestValue
                    if moveValue > bestValue
                        bestMove = [row, col];
                        bestValue = moveValue;
                    end
                end
            end
        end


        function boardVal = miniMax(self, depth, alpha, beta, isMax)

            boardVal = self.evaluateBoard(self.boardGrid, depth);
            
            % Terminal node (win/lose/draw) or max depth reached
            if abs(boardVal) > 0 || depth == 0 || self.availableMove == 0
                return;
            end
            
            % bWidth = size(self.boardGrid, 1);  % Assuming board is an 8x8 matrix
            
            if isMax
                highestVal = -Inf;
                for idx = 1:size(self.positions, 1)

                    row = self.positions(idx, 1);
                    col = self.positions(idx, 2);
                    if self.boardGrid(row, col) == 0   % 0 represents an empty tile
                        self.boardGrid(row, col) = 1;  % 1 represents 'X'
                        highestVal = max(highestVal, self.miniMax(depth - 1, alpha, beta, false));
                        self.boardGrid(row, col) = 0;  % Undo move
                        alpha = max(alpha, highestVal);
                        if alpha >= beta
                            boardVal = highestVal;
                            return;
                        end
                    end
                 
                end
                boardVal = highestVal;
                return;

            else
                lowestVal = Inf;
                for idx = 1:size(self.positions, 1)

                    row = self.positions(idx, 1);
                    col = self.positions(idx, 2);
                    if self.boardGrid(row, col) == 0   % 0 represents an empty tile
                        self.boardGrid(row, col) = 2;  % 2 represents 'O'
                        lowestVal = min(lowestVal, self.miniMax(depth - 1, alpha, beta, true));
                        self.boardGrid(row, col) = 0;  % Undo move
                        beta = min(beta, lowestVal);
                        if beta <= alpha
                            boardVal = lowestVal;
                            return;
                        end
                    end
                    
                end
                boardVal = lowestVal;
                return;
            end
        end


        function placeMark(self, row, col, mark) 
            self.boardGrid(row, col) = mark;
            self.availableMove = self.availableMove - 1;
            self.boardGrid
        end

        function score = evaluateBoard(self, grid, depth)
            % board: 8x8 matrix representing the tic-tac-toe board
            % 1 represents 'X', 2 represents 'O', and 0 represents a blank cell
            % depth: the depth in the minimax tree or number of moves made
        
            % Constants
            winCondition = 4;         % Need 4 in a row to win
            Xwin = 1 * winCondition;   % Sum for 4 consecutive 'X'
            Owin = 2 * winCondition;   % Sum for 4 consecutive 'O'
                
            bWidth = size(grid, 1);   % The board is 8x8
        
            % Check rows for 4 consecutive marks
            for row = 1:bWidth
                for col = 1:(bWidth - winCondition + 1)
                    temp_arr = grid(row, col:col+winCondition-1);
                    rowSum = sum(temp_arr);
                    if rowSum == Xwin && all(temp_arr == 1)
                        score = 10 - depth;
                        % disp('X win row!');
                        return;
                    elseif rowSum == Owin && all(temp_arr == 2)
                        score = -10 + depth;
                        % disp('O win row!');
                        return;
                    end
                    score = 0;
                end
            end
        
            % Check columns for 4 consecutive marks
            for col = 1:bWidth
                for row = 1:(bWidth - winCondition + 1)
                    temp_arr = grid(row:row+winCondition-1, col);
                    colSum = sum(temp_arr);
                    if colSum == Xwin && all(temp_arr == 1)
                        score = 10 - depth;
                        % disp('X win column!');
                        return;
                    elseif colSum == Owin && all(temp_arr == 2)
                        score = -10 + depth;
                        % disp('Y win column!');
                        return;
                    end
                    score = 0;
                end
            end
        
            % Check diagonals (top-left to bottom-right) for 4 consecutive marks
            for row = 1:(bWidth - winCondition + 1)
                for col = 1:(bWidth - winCondition + 1)
                    temp_arr = diag(grid(row:row+winCondition-1, col:col+winCondition-1));
                    diag1Sum = sum(temp_arr);
                    if diag1Sum == Xwin && all(temp_arr == 1)
                        score = 10 - depth;
                        % disp('X win diag!');
                        return;
                    elseif diag1Sum == Owin && all(temp_arr == 2)
                        score = -10 + depth;
                        % disp('Y win diag!');
                        return;
                    end
                end
            end
        
            % Check diagonals (top-right to bottom-left) for 4 consecutive marks
            for row = 1:(bWidth - winCondition + 1)
                for col = winCondition:bWidth
                    temp_arr = diag(flipud(grid(row:row+winCondition-1, col-winCondition+1:col)));
                    diag2Sum = sum(temp_arr);
                    if diag2Sum == Xwin && all(temp_arr == 1)
                        score = 10 - depth;
                        % disp('X win diag!');
                        return;
                    elseif diag2Sum == Owin && all(temp_arr == 2)
                        score = -10 + depth;
                        % disp('Y win diag!');
                        return;
                    end
                end
            end
        
            % No winner
            score = 0;
            % disp('No one win!');
        end

        function nearbyZeros = findNearbyZeros(self, grid)
            % Initialize an empty array to hold the positions of nearby zeros
            nearbyZeros = [];
        
            % Get the size of the board
            [rows, cols] = size(grid);
        
            % Directions for neighboring cells (up, down, left, right, and diagonals)
            directions = [
                -1,  0;  % Up
                 1,  0;  % Down
                 0, -1;  % Left
                 0,  1;  % Right
                -1, -1;  % Top-left diagonal
                -1,  1;  % Top-right diagonal
                 1, -1;  % Bottom-left diagonal
                 1,  1;  % Bottom-right diagonal
            ];
        
            % Iterate over each cell in the grid
            for i = 1:rows
                for j = 1:cols
                    % Check if the current cell is non-zero
                    if grid(i, j) ~= 0
                        % Check all neighboring positions
                        for k = 1:size(directions, 1)
                            ni = i + directions(k, 1);  % New row index
                            nj = j + directions(k, 2);  % New column index
        
                            % Check if the new position is within bounds
                            if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols
                                % Check if the neighbor cell is zero
                                if grid(ni, nj) == 0
                                    % Store the position (1-indexed)
                                    nearbyZeros = [nearbyZeros; ni, nj];
                                end
                            end
                        end
                    end
                end
            end
        
            % Remove duplicate entries (in case of adjacent non-zero cells leading to the same zero)
            nearbyZeros = unique(nearbyZeros, 'rows');
            if ~isempty(nearbyZeros)
                randIndices = randperm(size(nearbyZeros, 1));
                nearbyZeros = nearbyZeros(randIndices, :);
            end

        end

    end
end
