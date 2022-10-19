classdef DobotBarista < handle
    properties   
%         plyFileNameStem = 'DobotBarista';
        %> Robot Model
        model;

        %> defaultRealQ 
        defaultRealQ  = [0,pi/4,pi/4,0,0];
        
        %> workspace
        workspace = [-1 1 -1 1 -0.5 1.5]; 
    end

    methods
%% Define robot Function  
        function self = DobotBarista()
            if nargin < 1
                dobotBase = transl(0,0,0);
            end
            self.CreateModel();            
            self.PlotAndColourRobot();            
%             self.model.animate(self.RealQToModelQ(self.defaultRealQ))
            
            drawnow
        end

%% Create the robot model
        function CreateModel(self)       
            L(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            L(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            L(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(15),deg2rad(170)]);
            L(4) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',0, 'qlim',[deg2rad(-180),deg2rad(180)]);
            L(5) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',0, 'qlim',[deg2rad(-85),deg2rad(85)]);

            self.model = SerialLink(L,'name', DobotBarista, 'base', dobotBase);
        end   


%% plot and colour robot
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['DobotBarista_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
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

%% Test Move DobotBarista
    function TestMoveDobot(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);                       
            for i = 1:50                
                self.model.animate(self.RealQToModelQ(qPath(i,:)));
%                 hold on;
%                 trplot(self.model.fkine(self.RealQToModelQ(qPath(i,:))));
                pause(0.2);
            end
        end
    end

    methods
%% RealQToModelQ
        % Convert the real Q to the model Q
%         function modelQ = RealQToModelQ(realQ)
%             modelQ = realQ;
%             modelQ(3) = DobotBarista.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
%             modelQ(4) = pi - realQ(2) - modelQ(3);    
%         end
        
%% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
%         function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
%             modelQ3 = pi/2 - realQ2 + realQ3;
%         end
        
%% ModelQToRealQ
        % Convert the model Q to the real Q
%         function realQ = ModelQToRealQ( modelQ )
%             realQ = modelQ;
%             realQ(3) = DobotBarista.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
%         end
        
%% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
%         function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
%             realQ3 = modelQ3 - pi/2 + modelQ2;
%         end
    end
end