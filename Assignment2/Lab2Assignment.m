%% Lab Assignment 2 - RoBarista

%% Initialise and load the environment and safety features
% Clear and close all previous and current figures, variables, and the
% command window
close all
clear all
clc
clf

kuka = Kuka;
hold on

centerPoint = [0,0,0];
radii = [0.1,0.1,0.2];
% [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
% for i = 1:6
%     kuka.model.points{i} = [X(:),Y(:),Z(:)];
%     warning off
%     kuka.model.faces{i} = delaunay(kuka.model.points{i});     
%     warning on;
% end

kuka.model.plot3d([0,0,0,0,0,0,0]);
axis equal
camlight

%%
hold on

% Plot surface
% surf([-4,-4;4,4],[-4,4;-4,4],[-1.49,-1.49;-1.49,-1.49],'CData',...
   % imread('!!!!!!.jpg'),'FaceColor','texturemap');

% Read vertex and function data of estop ply file and plot with trisurf with
% added eStopPosition offset
[f,v,data] = plyread('estop.ply', 'tri');

vertexColoursEStop = [data.vertex.red, data.vertex.green, data.vertex.blue]/255; % Colour scaling
eStopPosition = [3,-3.9,1];
estop = trisurf(f, v(:,1) + eStopPosition(1,1), v(:,2) + eStopPosition(1,2), v(:,3) + eStopPosition(1,3) ...
    ,'FaceVertexCData', vertexColoursEStop, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

% Read vertex and function data of fire extinguisher ply file and plot with trisurf with
% added fireExPosition offset
[f,v,data] = plyread('fireextinguisher.ply', 'tri');

vertexColoursFireEx = [data.vertex.red, data.vertex.green, data.vertex.blue]/255; % Colour scaling
fireExPosition = [3,-3.8,-1];
FireEx = trisurf(f, v(:,1) + fireExPosition(1,1), v(:,2) + fireExPosition(1,2), v(:,3) + fireExPosition(1,3) ...
    ,'FaceVertexCData', vertexColoursFireEx, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

% Read vertex and function data of first aid kit ply file and plot with trisurf with
% added firstAidPosition offset
[f,v,data] = plyread('verbandtrommel.ply', 'tri');

vertexColoursFirstAid = [data.vertex.red, data.vertex.green, data.vertex.blue]/255; % Colour scaling
firstAidPosition = [2,-4,0.25];
firstAid = trisurf(f, v(:,1) + firstAidPosition(1,1), v(:,2) + firstAidPosition(1,2), v(:,3) + firstAidPosition(1,3) ...
    ,'FaceVertexCData', vertexColoursFirstAid, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');


% Initialise 9 bricks in the workspace at different positions on the XY plane
totalBricks = Bricks(9);

% Assign the bricks positions within range of the LinearUR3 on either side
% of it
totalBricks.brick{1}.base = eye(4)*transl(0.52, 0.5, 0);
totalBricks.brick{2}.base = eye(4)*transl(0.52, 0.7, 0);
totalBricks.brick{3}.base = eye(4)*transl(0.5, 0.9, 0);
totalBricks.brick{4}.base = eye(4)*transl(0.5, 1.1, 0);
totalBricks.brick{5}.base = eye(4)*transl(0.5, 1.3, 0);
totalBricks.brick{6}.base = eye(4)*transl(-0.3, 0.5, 0);
totalBricks.brick{7}.base = eye(4)*transl(-0.3, 0.7, 0);
totalBricks.brick{8}.base = eye(4)*transl(-0.3, 0.9, 0);
totalBricks.brick{9}.base = eye(4)*transl(-0.32, 1.1, 0);

% Update the bricks in the workspace to visually be present at above
% positions
totalBricks.brick{1}.animate(totalBricks.brick{1}.base)
totalBricks.brick{2}.animate(totalBricks.brick{2}.base)
totalBricks.brick{3}.animate(totalBricks.brick{3}.base)
totalBricks.brick{4}.animate(totalBricks.brick{4}.base)
totalBricks.brick{5}.animate(totalBricks.brick{5}.base)
totalBricks.brick{6}.animate(totalBricks.brick{6}.base)
totalBricks.brick{7}.animate(totalBricks.brick{7}.base)
totalBricks.brick{8}.animate(totalBricks.brick{8}.base)
totalBricks.brick{9}.animate(totalBricks.brick{9}.base)


hold on

% Load the LinearUR3 into the workspace
LinUr3 = LinearUR3;
%% Workspace Volume Calculation

% Point Cloud variables and parameteres initialised
steps = deg2rad(60);
rail_offset = 0.5;
qlim = LinUr3.model.qlim;

% calculate point cloud size based range of qlims for each joint and
% generate a point cloud of zeros that is size appropriate
pointCloudeSize1 = prod(floor((qlim(1:8,2)-qlim(1:8,1))/steps + 1)); 
pointCloud1 = zeros(pointCloudeSize1,3);
counter = 1; % Used to keep track of progress
tic % elapsed time for finding the workspace point cloud

for q1 = qlim(1,1):rail_offset:qlim(1,2)  % increment throught the joint limits of all joints
    for q2 = qlim(2,1):steps:qlim(2,2)
        for q3 = qlim(3,1):steps:qlim(3,2)
            for q4 = qlim(4,1):steps:qlim(4,2)
                for q5 = qlim(5,1):steps:qlim(5,2)
                    for q6 = qlim(6,1):steps:qlim(6,2)
                        for q7 = qlim(7,1):steps:qlim(7,2)
                            q8=0; % last joint not relevant to the point cloud
                            q = [q1,q2,q3,q4,q5,q6,q7,q8];
                            tr1 = LinUr3.model.fkine(q); % find end-effector pose
                            pointCloud1(counter,:) = tr1(1:3,4)'; % assign the x,y,z of each point to pointCloud1 
                            counter = counter + 1;
                            if mod(counter/pointCloudeSize1 * 100,1) == 0 
                                display(['After ',num2str(toc),' seconds, completed',...
                                    num2str(counter/pointCloudeSize1 * 100),'% of poses']); % display code progress
                            end
                        end
                    end
                end
            end
        end
    end
end

% 2.6 Plot3D model showing where the end effector can be over all points in the robot workspace.
plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'r.');

%% Calculate the volume of the robot workspace
% Initialise as -100 the maximum values in x,y,z to find the greatest value in
% pointCloud1
xMax = -100;
yMax = -100;
zMax = -100;

% Initialise as 100 the minimum values in x,y,z to find the lowest value in
% pointCloud1
xMin = 100;
yMin = 100;
zMin = 100;

% Find the largest x coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,1) > xMax
        xMax = pointCloud1(i,1);
    end
end

% Find the largest y coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,2) > yMax
        yMax = pointCloud1(i,2);
    end
end

% Find the largest z coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,3) > zMax
        zMax = pointCloud1(i,3);
    end
end

% Find the lowest x coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,1) < xMin
        xMin = pointCloud1(i,1);
    end
end

% Find the lowest y coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,2) < yMin
        yMin = pointCloud1(i,2);
    end
end

% Find the lowest z coordinate in pointCloud1
for i=1:1:pointCloudeSize1
    if pointCloud1(i,3) < zMin
        zMin = pointCloud1(i,3);
    end
end

% Find the radius of pointCloud1 in x,y,z axis and find the volume(approx.
% an ellipsoid = (4/3)*pi*xRadius*yRadius*zRadius)
xRadius = abs((xMax - xMin)/2);
yRadius = abs((yMax - yMin)/2);
zRadius = abs((zMax - zMin)/2);

V = (4/3)*pi*(xRadius*yRadius*zRadius)

%% Move robot to pick up object
% find robot joint angles for each object's position (offset is present to
% account for the gripper)
qToBrick1 = LinUr3.model.ikcon(totalBricks.brick{1}.base*transl(0,0,0.3)*trotx(pi));
qToBrick2 = LinUr3.model.ikcon(totalBricks.brick{2}.base*transl(0,0,0.3)*trotx(pi));


% calculate the corresponding the poses of each brick
poseToBrick1 = LinUr3.model.fkine(qToBrick1);
poseToBrick2 = LinUr3.model.fkine(qToBrick2);


% Find the end-effector joint configuration at initial joint states
startLinUr3 = LinUr3.model.fkine(zeros(8));
qStartLinUr3 = LinUr3.model.ikcon(startLinUr3);

steps = 100; % for quintic polynomial method

% Find trajectory of q from starting position to Brick1
startToB1 = jtraj(qStartLinUr3(1,:),qToBrick1,steps);

% Find the end-effector joint configuration of Brick1 dropoff
wallPosB1 = eye(4)*transl(-0.1,0.2,0)*trotx(pi)*trotz(pi/2);
qB1ToWall = LinUr3.model.ikcon(wallPosB1);

% Find trajectory of q from Brick1 to Brick1 dropoff
stackB1 = jtraj(qToBrick1,qB1ToWall,steps);


% Animate trajectory of robot from start to Brick1
for i=1:1:steps
    LinUr3.model.animate(startToB1(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick1
for i=1:1:steps
    LinUr3.model.animate(stackB1(i,:))
    totalBricks.brick{1}.base = LinUr3.model.fkine(...
        stackB1(i,:))
    totalBricks.brick{1}.animate(totalBricks.brick{1}.base)
    pause(0.01)
end



