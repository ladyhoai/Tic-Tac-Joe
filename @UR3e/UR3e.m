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
        mapStartTopRight = [-0.75, 0.25];
        XPlaced = 1;
        XArray;
        helperPlayer;
        attachGripper = 0;
        originalTransform;
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
            self.XArray = self.helperPlayer.placeXandO("OTick.PLY", self.XArray, baseTr);
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

            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            self.MoveRobot([posObj(1), posObj(2), posObj(3)]);
            self.attachGripper = '1';
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            % self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            self.MoveRobot([actualX, actualY, posObj(3) + 0.2]);
            self.MoveRobot([actualX, actualY, 0]);
            self.attachGripper = '0';
            self.MoveRobot([actualX, actualY, 0.2]);
            
            self.XPlaced = self.XPlaced + 1;
        end

        function MoveRobot(self, goal, orientation)
            if (nargin == 2)
                endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(180, "deg") * trotz(0, "deg"));
            else
                endEffector = transl(goal(1), goal(2), goal(3)) * (trotx(orientation(1), "deg") * troty(orientation(2), "deg") * trotz(orientation(3), "deg"));
            end
            pose1 = self.model.ikcon(endEffector * transl(0,0,-0.075), self.armJoint);
            endTraj = jtraj(self.armJoint, pose1, 50);
            for i = 1:size(endTraj, 1)
                self.model.animate(endTraj(i, :));
                fkineEndTraj = self.model.fkine(endTraj(i, :));
                fkineEndTraj = fkineEndTraj.T * transl(0,0,0.06);

                self.armJoint = endTraj(i, :);
                endPose = self.model.fkine(self.armJoint);
                self.handLeft.model.base = endPose.T * self.leftHandTrans * trotz(pi/2.4); 
                self.handRight.model.base = endPose.T * self.rightHandTrans * trotz(pi/2.4);
                self.handLeft.model.animate(self.handLeft.model.getpos());
                self.handRight.model.animate(self.handRight.model.getpos());

                if self.attachGripper == '1'
                    newVer =  (fkineEndTraj(1:3,1:3) * self.originalTransform')' + fkineEndTraj(1:3, 4)';
                    set(self.XArray(self.XPlaced), 'Vertices', newVer);
                end

                drawnow;
            end
        end

        
        function payload = fetchObjToTransfer(self)
            posObj = self.helperPlayer.objLocation(self.XPlaced);
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            self.MoveRobot([posObj(1), posObj(2), posObj(3)]);
            self.attachGripper = '1';
            self.MoveRobot([posObj(1), posObj(2), posObj(3) + 0.2]);
            self.MoveRobot([-0.5, 0.6, 0.6], [90, 90, 0]);
            self.attachGripper = '0';
            payload = self.XArray(self.XPlaced);
            self.XPlaced = self.XPlaced + 1;
        end

        function updateTransferredObject(self, transform)
            newVer =  (transform(1:3,1:3) * self.originalTransform')' + transform(1:3, 4)';
            set(self.XArray(self.XPlaced - 1), 'Vertices', newVer);
        end

    end
end
