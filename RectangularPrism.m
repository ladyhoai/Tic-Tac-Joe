classdef RectangularPrism < handle

    properties(Access = public)
            posArray = [];
            vertex;
            faceNormals;
            face;
            cube; 
    end

    methods
        function self = RectangularPrism(lower,upper,plotOptions,axis_h)
            if nargin<4
                    axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=false;
                    plotOptions.plotFaces=true;
                end
            end
            hold on
            
            self.vertex(1,:)=lower;
            self.vertex(2,:)=[upper(1),lower(2:3)];
            self.vertex(3,:)=[upper(1:2),lower(3)];
            self.vertex(4,:)=[upper(1),lower(2),upper(3)];
            self.vertex(5,:)=[lower(1),upper(2:3)];
            self.vertex(6,:)=[lower(1:2),upper(3)];
            self.vertex(7,:)=[lower(1),upper(2),lower(3)];
            self.vertex(8,:)=upper;
            
            self.face=[1,2,3;1,3,7;
                 1,6,5;1,7,5;
                 1,6,4;1,4,2;
                 6,4,8;6,5,8;
                 2,4,8;2,3,8;
                 3,7,5;3,8,5;
                 6,5,8;6,4,8];
            
            self.faceNormals = zeros(size(self.face,1),3);
            for faceIndex = 1:size(self.face,1)
                v1 = self.vertex(self.face(faceIndex,1)',:);
                v2 = self.vertex(self.face(faceIndex,2)',:);
                v3 = self.vertex(self.face(faceIndex,3)',:);
                self.faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
        
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [.2 .2 .8];
                
                self.cube = patch('Faces',self.face,'Vertices',self.vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
            end

            self.createControlPanel();
        end

        function moveCube(self, xDir, yDir, zDir)
            self.vertex = self.vertex + [xDir, yDir, zDir];
            set(self.cube, 'Vertices', self.vertex);
        end

        function res = checkCollide(self, robot, q)
            if (IsCollision(robot, q, self.face, self.vertex, self.faceNormals, true))
                res = true;
            else
                res = false;
            end
        end

        function createControlPanel(self)
            % Create a figure for the control panel
            f = figure('Position', [620, 80, 350, 200], 'MenuBar', 'none', 'Name', 'Move Cube', 'NumberTitle', 'off');
            
            % Create up arrow button
            uicontrol('Style', 'pushbutton', 'String', '↑', ...
                      'Position', [120, 130, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('forward'));
        
            % Create down arrow button
            uicontrol('Style', 'pushbutton', 'String', '↓', ...
                      'Position', [120, 50, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('backward'));
        
            % Create left arrow button
            uicontrol('Style', 'pushbutton', 'String', '←', ...
                      'Position', [50, 90, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('left'));
        
            % Create right arrow button
            uicontrol('Style', 'pushbutton', 'String', '→', ...
                      'Position', [190, 90, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('right'));

            uicontrol('Style', 'pushbutton', 'String', '↑', ...
                      'Position', [270, 130, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('up'));
        
            % Create down arrow button
            uicontrol('Style', 'pushbutton', 'String', '↓', ...
                      'Position', [270, 50, 60, 40], ...
                      'FontSize', 16, ...
                      'Callback', @(src, event) move('down'));


            function move(direction)
                % Callback function for button press
                switch direction
                    case 'forward'
                        self.moveCube(0.015, 0, 0);
                    case 'backward'
                        self.moveCube(-0.015, 0, 0);
                    case 'left'
                        self.moveCube(0, 0.015, 0);
                    case 'right'
                        self.moveCube(0, -0.015, 0);
                    case 'up'
                        self.moveCube(0, 0, 0.015);
                    case 'down'
                        self.moveCube(0, 0, -0.015);
                end
                % You can add your movement logic here
            end
        end

    end
end