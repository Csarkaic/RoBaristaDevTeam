%% Lab Assignment 1 - 12578833
%% Initialise and load the environment and safety features
% Clear and close all previous and current figures, variables, and the
% command window
close all
clear all
clc
clf

hold on

% Plot the concrete floor and wall surfaces
surf([-4,-4;4,4],[-4,4;-4,4],[-1.49,-1.49;-1.49,-1.49],'CData',...
    imread('concrete.jpg'),'FaceColor','texturemap');

surf([-3.9,-3.9;3.9,3.9],[-3.9,-3.9;-3.9,-3.9],[-1.49,1.49;-1.49,1.49],...
    'CData',imread('wall.jpeg'),'FaceColor','texturemap');

surf([-3.9,-3.9;-3.9,-3.9],[-3.9,-3.9;3.9,3.9],[-1.49,1.49;-1.49,1.49],...
    'CData',imread('wall.jpeg'),'FaceColor','texturemap');

% Read vertex and function data of table ply file and plot with trisurf with
% added tablePosition offset
[f,v,data] = plyread('cameron_table.ply', 'tri');

vertexColoursTable = [data.vertex.red, data.vertex.green, data.vertex.blue]/255; % Colour scaling
tablePosition = [-1.5,1.5,-1.64];
table = trisurf(f, v(:,1) + tablePosition(1,1), v(:,2) + tablePosition(1,2), v(:,3) + tablePosition(1,3) ...
    ,'FaceVertexCData', vertexColoursTable, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

% Read vertex and function data of fence ply file and plot with trisurf with
% added fencePosition offset
[f,v,data] = plyread('fence1.ply', 'tri');

vertexColoursFence = [data.vertex.red, data.vertex.green, data.vertex.blue]/255; % Colour scaling
fencePosition = [-0.7,0.5,-1.64];
fence = trisurf(f, v(:,1) + fencePosition(1,1), v(:,2) + fencePosition(1,2), v(:,3) + fencePosition(1,3) ...
    ,'FaceVertexCData', vertexColoursFence, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

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

%% Move LinearUR3 to pick up and place 9 bricks in a 3x3 wall
% find robot joint angles for each brick's position (offset is present to
% account for the gripper)
qToBrick1 = LinUr3.model.ikcon(totalBricks.brick{1}.base*transl(0,0,0.3)*trotx(pi));
qToBrick2 = LinUr3.model.ikcon(totalBricks.brick{2}.base*transl(0,0,0.3)*trotx(pi));
qToBrick3 = LinUr3.model.ikcon(totalBricks.brick{3}.base*transl(0,0,0.3)*trotx(pi));
qToBrick4 = LinUr3.model.ikcon(totalBricks.brick{4}.base*transl(0,0,0.3)*trotx(pi));
qToBrick5 = LinUr3.model.ikcon(totalBricks.brick{5}.base*transl(0,0,0.3)*trotx(pi));
qToBrick6 = LinUr3.model.ikcon(totalBricks.brick{6}.base*transl(0,0,0.3)*trotx(pi));
qToBrick7 = LinUr3.model.ikcon(totalBricks.brick{7}.base*transl(0,0,0.3)*trotx(pi));
qToBrick8 = LinUr3.model.ikcon(totalBricks.brick{8}.base*transl(0,0,0.3)*trotx(pi));
qToBrick9 = LinUr3.model.ikcon(totalBricks.brick{9}.base*transl(0,0,0.3)*trotx(pi));

% calculate the corresponding the poses of each brick
poseToBrick1 = LinUr3.model.fkine(qToBrick1);
poseToBrick2 = LinUr3.model.fkine(qToBrick2);
poseToBrick3 = LinUr3.model.fkine(qToBrick3);
poseToBrick4 = LinUr3.model.fkine(qToBrick4);
poseToBrick5 = LinUr3.model.fkine(qToBrick5);
poseToBrick6 = LinUr3.model.fkine(qToBrick6);
poseToBrick7 = LinUr3.model.fkine(qToBrick7);
poseToBrick8 = LinUr3.model.fkine(qToBrick8);
poseToBrick9 = LinUr3.model.fkine(qToBrick9);

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

% Find trajectory of q from starting Brick1 dropoff to Brick2
stackToB2 = jtraj(qB1ToWall,qToBrick2,steps);

% Find the end-effector joint configuration of Brick2 dropoff
wallPosB2 = eye(4)*transl(0.04,0.2,0)*trotx(pi)*trotz(pi/2);
qB2ToWall = LinUr3.model.ikcon(wallPosB2);

% Find trajectory of q from starting Brick2 to Brick2 dropoff
stackB2 = jtraj(qToBrick2,qB2ToWall,steps);

% Find trajectory of q from starting Brick2 dropoff to Brick3
stackToB3 = jtraj(qB2ToWall,qToBrick3,steps);

% Find the end-effector joint configuration of Brick3 dropoff
wallPosB3 = eye(4)*transl(0.18,0.2,0)*trotx(pi)*trotz(pi/2);
qB3ToWall = LinUr3.model.ikcon(wallPosB3);

% Find trajectory of q from starting Brick3 to Brick3 dropoff
stackB3 = jtraj(qToBrick3,qB3ToWall,steps);

% Find trajectory of q from starting Brick3 dropoff to Brick4
stackToB4 = jtraj(qB3ToWall,qToBrick4,steps);

% Find the end-effector joint configuration of Brick4 dropoff
wallPosB4 = eye(4)*transl(-0.1,0.2,0.05)*trotx(pi)*trotz(pi/2);
qB4ToWall = LinUr3.model.ikcon(wallPosB4);

% Find trajectory of q from starting Brick4 to Brick4 dropoff
stackB4 = jtraj(qToBrick4,qB4ToWall,steps);

% Find trajectory of q from starting Brick4 dropoff to Brick5
stackToB5 = jtraj(qB4ToWall,qToBrick5,steps);

% Find the end-effector joint configuration of Brick5 dropoff
wallPosB5 = eye(4)*transl(0.04,0.2,0.05)*trotx(pi)*trotz(pi/2);
qB5ToWall = LinUr3.model.ikcon(wallPosB5);

% Find trajectory of q from starting Brick5 to Brick5 dropoff
stackB5 = jtraj(qToBrick5,qB5ToWall,steps);

% Find trajectory of q from starting Brick5 dropoff to Brick6
stackToB6 = jtraj(qB5ToWall,qToBrick6,steps);

% Find the end-effector joint configuration of Brick6 dropoff
wallPosB6 = eye(4)*transl(0.18,0.2,0.05)*trotx(pi)*trotz(pi/2);
qB6ToWall = LinUr3.model.ikcon(wallPosB6);

% Find trajectory of q from starting Brick6 to Brick6 dropoff
stackB6 = jtraj(qToBrick6,qB6ToWall,steps);

% Find trajectory of q from starting Brick6 dropoff to Brick7
stackToB7 = jtraj(qB6ToWall,qToBrick7,steps);

% Find the end-effector joint configuration of Brick7 dropoff
wallPosB7 = eye(4)*transl(-0.1,0.2,0.1)*trotx(pi)*trotz(pi/2);
qB7ToWall = LinUr3.model.ikcon(wallPosB7);

% Find trajectory of q from starting Brick7 to Brick7 dropoff
stackB7 = jtraj(qToBrick7,qB7ToWall,steps);

% Find trajectory of q from starting Brick7 dropoff to Brick8
stackToB8 = jtraj(qB7ToWall,qToBrick8,steps);

% Find the end-effector joint configuration of Brick8 dropoff
wallPosB8 = eye(4)*transl(0.04,0.2,0.1)*trotx(pi)*trotz(pi/2);
qB8ToWall = LinUr3.model.ikcon(wallPosB8);

% Find trajectory of q from starting Brick8 to Brick8 dropoff
stackB8 = jtraj(qToBrick8,qB8ToWall,steps);

% Find trajectory of q from starting Brick8 dropoff to Brick9
stackToB9 = jtraj(qB8ToWall,qToBrick9,steps);

% Find the end-effector joint configuration of Brick9 dropoff
wallPosB9 = eye(4)*transl(0.18,0.2,0.1)*trotx(pi)*trotz(pi/2);
qB9ToWall = LinUr3.model.ikcon(wallPosB9);

% Find trajectory of q from starting Brick9 to Brick9 dropoff
stackB9 = jtraj(qToBrick9,qB9ToWall,steps);

% Find trajectory of q from starting Brick 9 dropoff to starting position
backToStart = jtraj(qB9ToWall,qStartLinUr3(1,:),steps);


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

% Animate trajectory of robot from brick wall to Brick2
for i=1:1:steps
    LinUr3.model.animate(stackToB2(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick2
for i=1:1:steps
    LinUr3.model.animate(stackB2(i,:))
    totalBricks.brick{2}.base = LinUr3.model.fkine(...
        stackB2(i,:))
    totalBricks.brick{2}.animate(totalBricks.brick{2}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick3
for i=1:1:steps
    LinUr3.model.animate(stackToB3(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick3
for i=1:1:steps
    LinUr3.model.animate(stackB3(i,:))
    totalBricks.brick{3}.base = LinUr3.model.fkine(...
        stackB3(i,:))
    totalBricks.brick{3}.animate(totalBricks.brick{3}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick4
for i=1:1:steps
    LinUr3.model.animate(stackToB4(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick4
for i=1:1:steps
    LinUr3.model.animate(stackB4(i,:))
    totalBricks.brick{4}.base = LinUr3.model.fkine(...
        stackB4(i,:))
    totalBricks.brick{4}.animate(totalBricks.brick{4}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick5
for i=1:1:steps
    LinUr3.model.animate(stackToB5(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick5
for i=1:1:steps
    LinUr3.model.animate(stackB5(i,:))
    totalBricks.brick{5}.base = LinUr3.model.fkine(...
        stackB5(i,:))
    totalBricks.brick{5}.animate(totalBricks.brick{5}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick6
for i=1:1:steps
    LinUr3.model.animate(stackToB6(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick6
for i=1:1:steps
    LinUr3.model.animate(stackB6(i,:))
    totalBricks.brick{6}.base = LinUr3.model.fkine(...
        stackB6(i,:))
    totalBricks.brick{6}.animate(totalBricks.brick{6}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick7
for i=1:1:steps
    LinUr3.model.animate(stackToB7(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick7
for i=1:1:steps
    LinUr3.model.animate(stackB7(i,:))
    totalBricks.brick{7}.base = LinUr3.model.fkine(...
        stackB7(i,:))
    totalBricks.brick{7}.animate(totalBricks.brick{7}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick8
for i=1:1:steps
    LinUr3.model.animate(stackToB8(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick8
for i=1:1:steps
    LinUr3.model.animate(stackB8(i,:))
    totalBricks.brick{8}.base = LinUr3.model.fkine(...
        stackB8(i,:))
    totalBricks.brick{8}.animate(totalBricks.brick{8}.base)
    pause(0.01)
end

% Animate trajectory of robot from brick wall to Brick9
for i=1:1:steps
    LinUr3.model.animate(stackToB9(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick9
for i=1:1:steps
    LinUr3.model.animate(stackB9(i,:))
    totalBricks.brick{9}.base = LinUr3.model.fkine(...
        stackB9(i,:))
    totalBricks.brick{9}.animate(totalBricks.brick{9}.base)
    pause(0.01)
end

% Animate trajectory of robot from Brick9 dropoff to starting position
for i=1:1:steps
    LinUr3.model.animate(backToStart(i,:))
    pause(0.01)
end

%% Slight movement of brick (demonstration)

qToBrick9 = LinUr3.model.ikcon(totalBricks.brick{9}.base*trotx(pi)); %*transl(0,0,0.3)
poseToBrick9 = LinUr3.model.fkine(qToBrick9);

startLinUr3 = LinUr3.model.fkine(zeros(8));
qStartLinUr3 = LinUr3.model.ikcon(startLinUr3);

steps = 100; % for quintic polynomial method

% Find trajectory of q from starting position to Brick1
startToB9 = jtraj(qStartLinUr3(1,:),qToBrick9,steps);

slightPosB1 = poseToBrick9*transl(0,-0.2,0); %*trotx(pi)
qB9ToPos = LinUr3.model.ikcon(slightPosB1);

moveB9 = jtraj(qToBrick9,qB9ToPos,steps);

for i=1:1:steps
    LinUr3.model.animate(startToB9(i,:))
    pause(0.01)
end

% Animate trajectory of robot stacking Brick1
for i=1:1:steps
    LinUr3.model.animate(moveB9(i,:))
    totalBricks.brick{9}.base = LinUr3.model.fkine(...
        moveB9(i,:))
    totalBricks.brick{9}.animate(totalBricks.brick{9}.base)
    pause(0.01)
end


