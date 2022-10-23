classdef Kuka < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1 1 -1 1 -0.5 1.5];   
      
    end
    
    methods%% Class for Kuka robot simulation
        function self = Kuka(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetKukaRobot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetKukaRobot
        % Given a name (optional), create and return a Kuka robot model
        function GetKukaRobot(self)
            pause(0.001);
            name = ['Kuka_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
<<<<<<< HEAD
            L1 = Link('d',0.34,'a',0,'alpha',-pi/2,'qlim',deg2rad([-170 170]), 'offset',0);
            L2 = Link('d',0,'a',0,'alpha',pi/2,'qlim', deg2rad([-120 120]), 'offset',0);
            L3 = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim', deg2rad([-170 170]), 'offset', 0);
            L4 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-120 120]),'offset', pi);
            L5 = Link('d',0.39,'a',0,'alpha',-pi/2,'qlim',deg2rad([-170 170]), 'offset',0);
            L6 = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-120 120]), 'offset', 0);
            L7 = Link('d',0.08,'a',0,'alpha',0,'qlim',deg2rad([-175 175]), 'offset', 0); 
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
            
            
%             L(1) = Link([0    0.157     0        0       0]);           
%             L(2) = Link([0     0.18     0      -pi/2     0]);
%             L(3) = Link([0      0      0     pi/2     pi/2]);
%             
%             L(4) = Link([0     0     1     -pi/2    0]);
%             L(5) = Link([0    0    0.5      -pi/2     pi/2]);
%             L(6) = Link([0    0.148    0      0     0]);
%             L(7) = Link([0     0.133    0      0     0]);
=======
            L(1) = Link([0     0.157    0      0     0]);           
            L(2) = Link([0     0.116    0      0     0]);
            L(3) = Link([0     0.269    0      0     0]);
            L(4) = Link([0     0.132    0      0     0]);
            L(5) = Link([0     0.269    0      0     0]);
            L(6) = Link([0     0.148    0      0     0]);
            L(7) = Link([0     0.133    0      0     0]);
     

    L(1).qlim = [-170 170]*pi/180;
    L(2).qlim = [-120 120]*pi/180;
    L(3).qlim = [-170 170]*pi/180;
    L(4).qlim = [-120 120]*pi/180;
    L(5).qlim = [-170 170]*pi/180;
    L(6).qlim = [-120 120]*pi/180;
    L(7).qlim = [-175 175]*pi/180;
  
    
            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base*transl(0,-0.55,0);
>>>>>>> a1d3b453bb6197af4194b35b858d03496f3c94ee
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['kuka_link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
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
