classdef Bricks < handle
    %Bricks A way of creating bricks
    %   The bricks can be moved around randomly. It is then possible to query
    %   the current location (base) of the bricks.
    
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of bricks
        brickCount = 2;
        
        %> A cell structure of \c brickCount brick models
        brick;
        
        %> paddockSize in meters
        paddockSize = [6, 6];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = Bricks(brickCount)
            if 0 < nargin
                self.brickCount = brickCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,-1.5,self.maxHeight];

            % Create the required number of bricks
            for i = 1:self.brickCount
                self.brick{i} = self.GetBrickModel(['brick',num2str(i)]);
                % Random spawn
                self.brick{i}.base = se3(se2((2 * rand()-1) * self.paddockSize(1)/2 ...
                                         , (2 * rand()-1) * self.paddockSize(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi));
                 % Plot 3D model
                plot3d(self.brick{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            camlight;
        end
        
        function delete(self)
%             cla;
        end       
        
        %% PlotSingleRandomStep
        % Move each of the bricks forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for brickIndex = 1:self.brickCount
                % Move Forward
                self.brick{brickIndex}.base = self.brick{brickIndex}.base * se3(se2(0.2, 0, 0));
                animate(self.brick{brickIndex},0);
                % Turn randomly
                self.brick{brickIndex}.base(1:3,1:3) = self.brick{brickIndex}.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
                animate(self.brick{brickIndex},0);                

                % If outside workspace rotate back around
                if self.brick{brickIndex}.base(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < self.brick{brickIndex}.base(1,4) ...
                || self.brick{brickIndex}.base(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < self.brick{brickIndex}.base(2,4)
                    self.brick{brickIndex}.base = self.brick{brickIndex}.base * se3(se2(-0.2, 0, 0)) * se3(se2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetBrickModel
        function model = GetBrickModel(name)
            if nargin < 1
                name = 'brick';
            end
            [faceData,vertexData] = plyread('HalfSizedRedGreenBrick.ply','tri');
            L1 = Link('alpha',-pi/2,'a',0,'d',0.3,'offset',0); 
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
    end    
end
