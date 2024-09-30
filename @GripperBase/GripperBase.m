classdef GripperBase < RobotBaseClass
    % LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)
        plyFileNameStem = 'GripperBase';
        handLeft;
        handRight;
        leftHandTrans = [0 0 1 0; 0 1 0 0.017; -1 0 0 -0.08; 0 0 0 1];
        rightHandTrans = [0 0 -1 0; 0 -1 0 -0.017; -1 0 0 -0.08; 0 0 0 1];
    end

    methods
        % Define robot Function
        function self = GripperBase(baseTr)
%             axis([-1 1 -1 1 -1 1])
%             hold on
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.model.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist','nobase','notiles','nojaxes'};
            self.PlotAndColourRobot();

            self.handLeft = GripperHand(self.model.base.T * self.leftHandTrans * trotz(pi/1.5)); 
            self.handRight = GripperHand(self.model.base.T * self.rightHandTrans * trotz(pi/1.5)); 
            drawnow;
        end

        % Create the robot model
        function CreateModel(self)
            link(1) = Link('d',-0.0145,'a',0,'alpha',0,'qlim',0);
            self.model = SerialLink(link,'name',self.name);

        end

    end
end
