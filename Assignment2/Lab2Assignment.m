%% Lab Assignment 2 - RoBarista
clear all
close all
clc
clf

kuka = Kuka;

%% Kuka joint ellipsoids

centerPoint = [0,0,0];
radii = [0.1,0.1,0.2];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:7
    kuka.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{i} = delaunay(kuka.model.points{i});     
    warning on;
end

%kuka.model.plot3d([0,0,0,0,0,0,0]);
axis equal

%% COllision Detection
%% Question 2: Ellipsoid and Point collision checking
% 2.1
clf;
centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
view(3);
hold on;

% 2.2
ellipsoidAtOrigin_h = surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);

% 2.3
% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
%cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal


%% Workspace Volume Calculation

% Point Cloud variables and parameteres initialised
steps = deg2rad(60);
qlim = kuka.model.qlim;

% calculate point cloud size based range of qlims for each joint and
% generate a point cloud of zeros that is size appropriate
pointCloudeSize1 = prod(floor((qlim(1:7,2)-qlim(1:7,1))/steps + 1)); 
pointCloud1 = zeros(pointCloudeSize1,3);
counter = 1; % Used to keep track of progress
tic % elapsed time for finding the workspace point cloud

for q1 = qlim(1,1):steps:qlim(1,2)  % increment throught the joint limits of all joints
    for q2 = qlim(2,1):steps:qlim(2,2)
        for q3 = qlim(3,1):steps:qlim(3,2)
            for q4 = qlim(4,1):steps:qlim(4,2)
                for q5 = qlim(5,1):steps:qlim(5,2)
                    for q6 = qlim(6,1):steps:qlim(6,2)
                        for q7 = qlim(7,1):steps:qlim(7,2)
                            % last joint not relevant to the point cloud
                            q = [q1,q2,q3,q4,q5,q6,q7];
                            tr1 = kuka.model.fkine(q); % find end-effector pose
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



