classdef UR3e < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR3e';
        handLeft;
        handRight;
        leftHandTrans = transl(0,0,-0.01)  * troty(-pi/2);
        rightHandTrans = transl(0,0,-0.01)  * troty(-pi/2) * trotx(-pi);
        armJoint = [0 0 0 0 0 0 0];
        homePos = [-0.85, 0.95, 0.2];
        mapStartTopRight = [-0.75, 0.25];
        XPlaced = 1;
        XArray;
        helperPlayer;
        attachGripper = 0;
        originalTransform;

         %these variables hold the current state of the system
        estop = false;
        currentFinalGoal;
        currentTraj;
        currentStep = 1;
        currentPoint = 1;
    end
    
    methods
%% Constructor
        function self = UR3e(baseTr,useTool,toolFilename)
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

            self.helperPlayer = helper();
            self.XArray = self.helperPlayer.placeXandO("OTick.PLY", self.XArray, baseTr, -0.25);
            a = PlaceObject("OTick.PLY", [0,0,0]);
            self.originalTransform = get(a, 'Vertices');
            delete(a);
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',pi);
            link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', pi/2);
            link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            link(7) = Link('d',	0.09,'a',0,'alpha',0,'qlim',[0 0], 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end      

        function pickAndPlace(self, location)
            actualX = self.mapStartTopRight(1) + 0.1 * location(1);
            actualY = self.mapStartTopRight(2) + 0.1 * location(2);

            posObj = self.helperPlayer.objLocation(self.XPlaced);

            if self.currentPoint == 1
                self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]', [180 0 0], false, 'RMRC');
            end

            if self.currentPoint == 2
                self.MoveRobot([posObj(1), posObj(2), posObj(3)]);
            end

            if self.currentPoint == 3
                self.attachGripper = '1';
                self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]);
            end

            if self.currentPoint == 4
                self.MoveRobot(self.homePos);
            end

            if self.currentPoint == 5
                self.MoveRobot([actualX, actualY, posObj(3) + 0.1]);
            end

            if self.currentPoint == 6
                self.MoveRobot([actualX, actualY, 0.08]', [180 0 0], false, 'RMRC');
            end

            if self.currentPoint == 7
                self.attachGripper = '0';
                self.MoveRobot([actualX, actualY, 0.1]);
            end

            if self.currentPoint == 8
                self.MoveRobot(self.homePos);
            end

            if self.estop == true
                return;
            end

            self.currentPoint = 1;
            self.XPlaced = self.XPlaced + 1;
        end

        function MoveRobot(self, goal, orientation, objHandler, varargin)

            if self.estop == true
                return;
            end

            global sliderArrayPlayer;
            global jointArrayPlayer;
            global jointArrayPlayerValue;

            if (self.currentStep == 1)
                if (nargin == 2)
                    endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(180, "deg") * trotz(0, "deg"));
                else
                    endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(orientation(1), "deg") * troty(orientation(2), "deg") * trotz(orientation(3), "deg"));
                end
                pose1 = self.model.ikcon(endEffector * transl(0,0,-0.075), self.armJoint);
    
                if (isempty(varargin))
                    self.currentTraj = jtraj(self.armJoint, pose1, 50);
                else
                    for i = 1:2:length(varargin)
                        switch varargin{i}
                            case 'RMRC'
                                self.currentTraj = self.calculateRMRC(goal, orientation); % the qMatrix returned by RMRC calculation
                                self.currentTraj(end, :) = [];
                        end
                    end
                end
            end

            for i = self.currentStep:size(self.currentTraj, 1)
                if self.estop == true
                    self.currentStep = i;
                    return;
                end

                self.model.animate(self.currentTraj(i, :));

                for y = 1:6
                    set(sliderArrayPlayer(y), 'Value', self.currentTraj(i, y));
                    set(jointArrayPlayerValue(y), 'String', num2str(self.currentTraj(i, y), 3));
                end
                jointArrayPlayer = self.currentTraj(i, :);

                fkineEndTraj = self.model.fkine(self.currentTraj(i, :));
                fkineEndTraj = fkineEndTraj.T * transl(0,0,0.06);

                self.armJoint = self.currentTraj(i, :);
                endPose = self.model.fkine(self.armJoint);
                self.handLeft.model.base = endPose.T * self.leftHandTrans * trotz(pi/2.4); 
                self.handRight.model.base = endPose.T * self.rightHandTrans * trotz(pi/2.4);
                self.handLeft.model.animate(self.handLeft.model.getpos());
                self.handRight.model.animate(self.handRight.model.getpos());

                if (nargin == 4 && isempty(varargin))   
                    objHandler.updateTransferredObject(fkineEndTraj);
                end

                if self.attachGripper == '1'
                    newVer =  (fkineEndTraj(1:3,1:3) * self.originalTransform')' + fkineEndTraj(1:3, 4)';
                    set(self.XArray(self.XPlaced), 'Vertices', newVer);
                end

                drawnow;
            end

            self.currentStep = 1;
            self.currentPoint = self.currentPoint + 1;
        end

        function receiveTransferAndPlace(self, objHandler, pos)
            actualX = self.mapStartTopRight(1) + 0.1 * pos(1);
            actualY = self.mapStartTopRight(2) + 0.1 * pos(2);
            if self.currentPoint == 1
                self.MoveRobot([actualX, actualY, 0.1], [180 0 0], objHandler);
            end
            if self.currentPoint == 2
                self.MoveRobot([actualX, actualY, 0], [180 0 0], objHandler);
            end
            if self.currentPoint == 3
                self.MoveRobot([actualX, actualY, 0.1], [180 0 0]);
            end
            if self.currentPoint == 4
                self.MoveRobot(self.homePos);
            end

            if self.estop == true
                return;
            end

            self.currentPoint = 1;
        end
        
        function payload = fetchObjToTransfer(self)
            posObj = self.helperPlayer.objLocation(self.XPlaced);

            if self.currentPoint == 1
                self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]', [180 0 0], false, 'RMRC');
            end

            if self.currentPoint == 2
                self.MoveRobot([posObj(1), posObj(2), posObj(3)]);
            end

            if self.currentPoint == 3
                self.attachGripper = '1';
                self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.1]);
            end

            if self.currentPoint == 4
                self.MoveRobot(self.homePos);
            end

            if self.currentPoint == 5
                self.MoveRobot([-0.5, 0.6, 0.5], [90, 90, 90]);
            end

            if self.estop == true
                return;
            end

            self.currentPoint = 1;
            self.attachGripper = '0';
            payload = self.XArray(self.XPlaced);
            self.XPlaced = self.XPlaced + 1;
        end

        function updateTransferredObject(self, transform)
            newVer =  (transform(1:3,1:3) * self.originalTransform')' + transform(1:3, 4)';
            set(self.XArray(self.XPlaced - 1), 'Vertices', newVer);
        end

        function res = calculateRMRC(self, goal, orientation, curJointAngle, numStep) 
            if (nargin == 5)
                steps = numStep;
            else
                steps = 50;
            end

            deltaT = 0.05; % Discrete time step
            
            x = zeros(3,steps); % for translation
            rotTraj = zeros(3, steps); % 3 rows for roll, pitch, yaw
            currentPose = self.model.fkine(curJointAngle).T;
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
            qMatrix(1, :) = curJointAngle;

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

        function stop(self)
            self.estop = true;
        end

        function resume(self)
            self.estop = false;
        end

        function animateWithGripper(self, jointVal)
            self.model.animate(jointVal);
            endPose = self.model.fkine(jointVal);
            self.handLeft.model.base = endPose.T * self.leftHandTrans * trotz(pi/2.4); 
            self.handRight.model.base = endPose.T * self.rightHandTrans * trotz(pi/2.4);
            self.handLeft.model.animate(self.handLeft.model.getpos());
            self.handRight.model.animate(self.handRight.model.getpos());
        end


    end
end
