classdef LinearUR3 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-3 3 -3 3 -1.5 4];   
      
    end
    
    methods%% Class for LinearUR3 robot simulation
        function self = LinearUR3(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetLinearUR3Robot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetLinearUR3Robot
        % Given a name (optional), create and return a LinearUR3 robot model
        function GetLinearUR3Robot(self)
            pause(0.001);
            name = ['LinearUR3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            L(1) = Link([pi     0       0      pi/2     1]);           
            L(2) = Link([0     0.08     0       0       0]);
            L(3) = Link([0    0.1519       0      pi/2  0]);
            L(4) = Link([0      0    -0.24365   0       0]);
            L(5) = Link([0      0    -0.21325   0       0]);
            L(6) = Link([0    0.11235   0      pi/2	    0]);
            L(7) = Link([0    0.08535   0     -pi/2     0]);
            L(8) = Link([0    0.0819       0       0    0]);

    L(1).qlim = [-0.8 0];
    L(2).qlim = [-360 360]*pi/180;
    L(3).qlim = [-360 360]*pi/180;
    L(4).qlim = [-360 360]*pi/180;
    L(5).qlim = [-360 360]*pi/180;
    L(6).qlim = [-360 360]*pi/180;
    L(7).qlim = [-360 360]*pi/180;
    L(8).qlim = [-360 360]*pi/180;
    
            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base * trotx(pi/2) * troty(pi/2)*transl(1.3,0,0.1)*troty(pi/2);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['LinUR3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
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
