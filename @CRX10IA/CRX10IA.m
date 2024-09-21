classdef CRX10IA < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'CRX10IA';
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
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.2503,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0.2604,'a',0.71,'alpha',-pi,'qlim', deg2rad([-360 360]), 'offset',pi/2);
            link(3) = Link('d',0.2604,'a',0,'alpha',-pi/2,'qlim', deg2rad([-360 360]), 'offset', pi/2);
            link(4) = Link('d',0.54,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.15,'a',0,'alpha',pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',	0.16,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
