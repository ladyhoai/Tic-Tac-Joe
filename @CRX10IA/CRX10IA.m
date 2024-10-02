classdef CRX10IA < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'CRX10IA';
        handLeft;
        handRight;
        leftHandTrans = transl(0,0,-0.01)  * troty(-pi/2);
        rightHandTrans = transl(0,0,-0.01)  * troty(-pi/2) * trotx(-pi);
        armJoint = [0 0 0 0 0 0 0];
        homePos = [0, 0.8, 0.3];
        mapStartTopRight = [-0.75, 0.25];
        OPlaced = 1;
        OArray;
        helperBot;
        attachGripper = 0;
        originalTransform;
    end
    
    methods
%% Constructor
    function self = CRX10IA(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            self.handLeft = GripperHand(self.model.fkine(self.model.getpos()).T * self.rightHandTrans * trotz(pi/1.5)); 
            self.handRight = GripperHand(self.model.fkine(self.model.getpos()).T * self.leftHandTrans * trotz(pi/1.5)); 
            self.helperBot = helper();
            self.OArray = self.helperBot.placeXandO("XTick.PLY", self.OArray, baseTr);

            a = PlaceObject("XTick.PLY", [0,0,0]);
            self.originalTransform = get(a, 'Vertices');
            delete(a);
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.2503,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0.2604,'a',0.71,'alpha',-pi,'qlim', deg2rad([-180 180]), 'offset',pi/2);
            link(3) = Link('d',0.2604,'a',0,'alpha',-pi/2,'qlim', deg2rad([-180 180]), 'offset', pi);
            link(4) = Link('d',0.54,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.15,'a',0,'alpha',pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',	0.16,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            link(7) = Link('d',	0.09,'a',0,'alpha',0,'qlim',[0 0], 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end
        
        %give location as the actual location on the tick tac toe grid,
        %then it will be translated to xyz coordinate
        function pickAndPlace(self, location)
            actualX = self.mapStartTopRight(1) + 0.1 * location(1);
            actualY = self.mapStartTopRight(2) + 0.1 * location(2);

            posObj = self.helperBot.objLocation(self.OPlaced);

            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]', [180 0 0], false, 'RMRC');
            self.MoveRobot([posObj(1), posObj(2), posObj(3)]);
            self.attachGripper = '1';
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            % self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            self.MoveRobot(self.homePos', [180 0 0], false, 'RMRC');
            self.MoveRobot([actualX, actualY, posObj(3) + 0.2]', [180 0 0], false, 'RMRC');
            self.MoveRobot([actualX, actualY, 0]);
            self.attachGripper = '0';
            self.MoveRobot([actualX, actualY, 0.2]);
            self.MoveRobot(self.homePos', [180 0 0], false, 'RMRC');

            self.OPlaced = self.OPlaced + 1;
        end


        function MoveRobot(self, goal, orientation, objHandler, varargin)
            if (nargin == 2)
                endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(180, "deg") * trotz(0, "deg"));
            else
                endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(orientation(1), "deg") * troty(orientation(2), "deg") * trotz(orientation(3), "deg"));
            end
            pose1 = self.model.ikcon(endEffector * transl(0,0,-0.08), self.armJoint);

            if (isempty(varargin))
                endTraj = jtraj(self.armJoint, pose1, 50);
            else
                for i = 1:2:length(varargin)
                    switch varargin{i}
                        case 'RMRC'
                            endTraj = self.calculateRMRC(goal, orientation); % the qMatrix returned by RMRC calculation
                            endTraj(end, :) = [];
                            
                    end
                end
            end

            for i = 1:size(endTraj, 1)
                self.model.animate(endTraj(i, :));
                fkineEndTraj = self.model.fkine(endTraj(i, :));
                fkineEndTraj = fkineEndTraj.T * transl(0,0,0.06);

                self.armJoint = endTraj(i, :);
                endPose = self.model.fkine(self.armJoint);
                self.handLeft.model.base = endPose.T * self.leftHandTrans * trotz(pi/2.55); 
                self.handRight.model.base = endPose.T * self.rightHandTrans * trotz(pi/2.55);
                self.handLeft.model.animate(self.handLeft.model.getpos());
                self.handRight.model.animate(self.handRight.model.getpos());
                if (nargin == 4 && isempty(varargin))   
                    objHandler.updateTransferredObject(fkineEndTraj);
                end

                if self.attachGripper == '1'
                    newVer =  (fkineEndTraj(1:3,1:3) * self.originalTransform')' + fkineEndTraj(1:3, 4)';
                    set(self.OArray(self.OPlaced), 'Vertices', newVer);
                end
                drawnow;
            end
        end

        function receiveTransferAndPlace(self, objHandler, pos)
            self.MoveRobot([-0.4, 0.6, 0.5], [0, -90, 90]);
            self.MoveRobot([-0.52, 0.6, 0.5], [0, -90, 90]);
            self.MoveRobot([-0.4, 0.6, 0.5], [0, -90, 90], objHandler);
            objHandler.MoveRobot(objHandler.homePos);
            actualX = self.mapStartTopRight(1) + 0.1 * pos(1);
            actualY = self.mapStartTopRight(2) + 0.1 * pos(2);
            self.MoveRobot([actualX, actualY, 0.2], [180 0 0], objHandler);
            self.MoveRobot([actualX, actualY, 0], [180 0 0], objHandler);
            self.MoveRobot([actualX, actualY, 0.2], [180 0 0]);
            self.MoveRobot(self.homePos', [180 0 0], false, 'RMRC');
        end

        function payload = fetchObjToTransfer(self)
            posObj = self.helperBot.objLocation(self.OPlaced);
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]', [180 0 0], false, 'RMRC');
            self.MoveRobot([posObj(1), posObj(2), posObj(3)]);

            self.attachGripper = '1';
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]);
            self.MoveRobot(self.homePos);

            self.MoveRobot([-0.4, 0.6, 0.5], [0, -90, 90]);
            self.MoveRobot([-0.52, 0.6, 0.5], [0, -90, 90]);

            self.attachGripper = '0';
            self.MoveRobot([-0.4, 0.6, 0.5], [0, -90, 90]);
            self.MoveRobot(self.homePos);

            payload = self.OArray(self.OPlaced);
            self.OPlaced = self.OPlaced + 1;
        end

        function updateTransferredObject(self, transform)
            newVer =  (transform(1:3,1:3) * self.originalTransform')' + transform(1:3, 4)';
            set(self.OArray(self.OPlaced - 1), 'Vertices', newVer);
        end

        function res = calculateRMRC(self, goal, orientation) 
            steps = 50;
            deltaT = 0.05; % Discrete time step
            
            x = zeros(3,steps); % for translation
            rotTraj = zeros(3, steps); % 3 rows for roll, pitch, yaw
            currentPose = self.model.fkine(self.armJoint).T;
            currentTrans = currentPose(1:3, 4);
            currentRot = currentPose(1:3, 1:3);

            RGoal = rpy2r(deg2rad(orientation(1)), deg2rad(orientation(2)), deg2rad(orientation(3)));
            s = lspb(0,1,steps); % Create interpolation scalar

            for i = 1:steps
                x(:,i) = currentTrans*(1-s(i)) + s(i)*goal; % Create trajectory in x-y
                Ri = currentRot * (1 - s(i)) + RGoal * s(i); % Interpolate between rotation matrices
                rotTraj(:,i) = tr2rpy(Ri); % Convert back to roll, pitch, yaw
            end

            qMatrix = nan(steps,7);
            qMatrix(1, :) = self.armJoint;

             for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate
                xdot_rot = (rotTraj(:,i+1) - rotTraj(:,i)) / deltaT; % Rotational velocity
                xdot = [xdot; xdot_rot];
                J = self.model.jacob0(qMatrix(i,:)); % Get the Jacobian at the current state
                qdot = pinv(J)*xdot; % Solve velocitities via RMRC
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot'; % Update next
            end
            res = qMatrix;
        end

    end
end
