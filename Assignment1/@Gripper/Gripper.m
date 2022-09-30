classdef Gripper < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-3 3 -3 3 -1.5 4];   
      
    end
    
    methods%% Class for Gripper robot simulation
        function self = Gripper(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetGripperRobot();
            self.PlotAndColourRobot();%robot,workspace);
            %self.model.teach;
            drawnow
        end

        %% GetGripperRobot
        % Given a name (optional), create and return a Gripper robot model
        function GetGripperRobot(self)
            pause(0.001);
            name = ['Gripper_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            L(1) = Link([0     0       0      pi/2     0]);           
            L(2) = Link([0     0.07     0       0       0]);
        %    L(3) = Link([0    0.1519       0      pi/2  0]);


%           L3 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%           L4 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
%           L5 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
%           L6 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
%           L7 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
%           L8 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

    L(1).qlim = [-360 360]*pi/180;
    L(2).qlim = [-360 360]*pi/180;
 %   L(3).qlim = [-360 360]*pi/180;
              
            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base * trotx(pi/2) * troty(pi/2)*transl(1.3,0,0.1)*troty(pi/2);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['gripper_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end
