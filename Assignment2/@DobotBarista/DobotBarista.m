classdef DobotBarista < handle
    properties
        model;
        dobotPointCloud = [];
        
        workspace = [-1 1 -1 1 -0.02 1.5];
        defaultQ = [0 pi/4 pi/2 -pi/4 0];
    end
    
    methods
        function self = DobotBarista()
            if nargin < 1
                dobotBase = transl(0,0,0);
            end
            self.getDobot();
            self.plotRobot();
        end
        
        function getDobot(self)
            name = ['DobotBarista'];
            
            L(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            L(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            L(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(15),deg2rad(170)]);
            L(4) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',0, 'qlim',[deg2rad(-180),deg2rad(180)]);
            L(5) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',0, 'qlim',[deg2rad(-85),deg2rad(85)]);
            
            self.model = SerialLink(L,'name', name);
            self.model.base = self.model.base*transl(0,0.9,0.55)*trotz(pi);
        end
        
        function plotRobot(self)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex +1}] = plyread(['DobotBarista_J',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            self.model.plot3d(self.defaultQ,'noarrow','workspace',self.workspace);
        end
        
        
    end
end