%% Lab Assignment 2 - RoBarista
clear all
close all
clc
clf

kuka = Kuka;

%% Projectile test and collision check (Light Curtain)
% Spawn and set base position of foreign object (cup)
totalCups = Cups(1);
totalCups.cup{1}.base = eye(4)*transl(-1,0,1);
totalCups.cup{1}.animate(totalCups.cup{1}.base); % update position
cupQ1 = totalCups.cup{1}.base;

projectileDetected = 0; % light curtain signal indicator

for i=-1:-0.05:-3
    
    cupQ1(1,4) = i;
    totalCups.cup{1}.base = cupQ1;
    totalCups.cup{1}.animate(cupQ1);
    pause(0.01)
    %         totalCups.cup{1}.base = totalCups.cup{1}.fkine(projectileTraj(j,:));
    %         totalCups.cup{1}.animate(projectileTraj)
    
    if totalCups.cup{1}.base(1,4) >= 2
        projectileDetected = 1;
    end
    
    if projectileDetected == 1
        display('Foreign object deteced by light curtain');
        break
    end
    
end
  
    
%% Kuka joint ellipsoids

% Joint ellipsoid paramters
centerPoint = [0,0,0];
radii = [0.1,0.1,0.12];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

% Increment through each robot joint and create an ellipsoid centred at
% each joint
for i = 1:7
    kuka.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    kuka.model.faces{i} = delaunay(kuka.model.points{i});     
    warning on;
end

% Uncomment below to plot the ellipsoid at each joint

% kuka.model.plot3d([0,0,0,0,0,0,0]);

%% Load in collision environment point cloud
centerPoint = [0,0,0];
view(3);
hold on;

% Light Curtain

% One side of the cube
[Y,Z] = meshgrid(-1:0.05:1,-1:0.05:1);

sizeMat = size(Y);
X = repmat(1,sizeMat(1),sizeMat(2));
oneSideOfLightCurt = surf(X,Y,Z);

% Combine one surface as a point cloud
lightCurtPoints = [X(:),Y(:),Z(:)];

try delete(X); end 
try delete(oneSideOfLightCurt); end 
         
% Plot the cube's point cloud         
lightCurtPoints = lightCurtPoints + repmat([-3,0,0.75],size(lightCurtPoints,1),1);

% Coffee grinder

% One side of the cube
[Y,Z] = meshgrid(-0.1:0.05:0.1,-0.1:0.05:0.1);

sizeMat = size(Y);
X = repmat(0.1,sizeMat(1),sizeMat(2));
oneSideOfCoffeeGr = surf(X,Y,Z);

% Combine one surface as a point cloud
coffeeGrPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
coffeeGrPoints = [ coffeeGrPoints ...
             ; coffeeGrPoints * rotz(pi/2)...
             ; coffeeGrPoints * rotz(pi) ...
             ; coffeeGrPoints * rotz(3*pi/2) ...
             ; coffeeGrPoints * roty(pi/2) ...
             ; coffeeGrPoints * roty(-pi/2)]; 
         
try delete(X); end
try delete(oneSideOfCoffeeGr); end 
         
% Plot the cube's point cloud         
coffeeGrPoints = coffeeGrPoints + repmat([-0.9,1,0.8],size(coffeeGrPoints,1),1);


% Tamper(front)
[Y,Z] = meshgrid(-0.04:0.05:0.04,-0.04:0.05:0.04);

sizeMat = size(Y);
X = repmat(0.04,sizeMat(1),sizeMat(2));
oneSideOfTamperF = surf(X,Y,Z);

% Combine one surface as a point cloud
tamperFPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
tamperFPoints = [ tamperFPoints ...
             ; tamperFPoints * rotz(pi/2)...
             ; tamperFPoints * rotz(pi) ...
             ; tamperFPoints * rotz(3*pi/2) ...
             ; tamperFPoints * roty(pi/2) ...
             ; tamperFPoints * roty(-pi/2)];         
         
         
try delete(X); end
try delete(oneSideOfTamperF); end 

% Plot the cube's point cloud         
tamperFPoints = tamperFPoints + repmat([-0.64,1.1,0.85],size(tamperFPoints,1),1);


% Tamper(back)
[Y,Z] = meshgrid(-0.1:0.05:0.1,-0.1:0.05:0.1);

sizeMat = size(Y);
X = repmat(0.1,sizeMat(1),sizeMat(2));
oneSideOfTamperB = surf(X,Y,Z);

% Combine one surface as a point cloud
tamperBPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
tamperBPoints = [ tamperBPoints ...
             ; tamperBPoints * rotz(pi/2)...
             ; tamperBPoints * rotz(pi) ...
             ; tamperBPoints * rotz(3*pi/2) ...
             ; tamperBPoints * roty(pi/2) ...
             ; tamperBPoints * roty(-pi/2)]; 
         
try delete(X); end
try delete(oneSideOfTamperB); end 
         
% Plot the cube's point cloud         
tamperBPoints = tamperBPoints + repmat([-0.64,1.23,0.7],size(tamperBPoints,1),1);


% Heated Food Unit
[Y,Z] = meshgrid(-0.375:0.05:0.375,-0.375:0.05:0.375);

sizeMat = size(Y);
X = repmat(0.375,sizeMat(1),sizeMat(2));
oneSideOfHFPoints = surf(X,Y,Z);

% Combine one surface as a point cloud
heatedFoodPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
heatedFoodPoints = [ heatedFoodPoints ...
             ; heatedFoodPoints * rotz(pi/2)...
             ; heatedFoodPoints * rotz(pi) ...
             ; heatedFoodPoints * rotz(3*pi/2) ...
             ; heatedFoodPoints * roty(pi/2) ...
             ; heatedFoodPoints * roty(-pi/2)];  

try delete(X); end
try delete(oneSideOfHFPoints); end 
         
% Plot the cube's point cloud         
heatedFoodPoints = heatedFoodPoints + repmat([0.75,1.27,0.65],size(heatedFoodPoints,1),1);


% Shelf Unit
[Y,Z] = meshgrid(-0.65:0.05:0.65,-0.65:0.05:0.65);

sizeMat = size(Y);
X = repmat(0.65,sizeMat(1),sizeMat(2));
oneSideOfShelf = surf(X,Y,Z);

% Combine one surface as a point cloud
shelfPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
shelfPoints = [ shelfPoints ...
             ; shelfPoints * rotz(pi/2)...
             ; shelfPoints * rotz(pi) ...
             ; shelfPoints * rotz(3*pi/2) ...
             ; shelfPoints * roty(pi/2) ...
             ; shelfPoints * roty(-pi/2)]; 
         
try delete(X); end
try delete(oneSideOfShelf); end 

% Plot the cube's point cloud         
shelfPoints = shelfPoints + repmat([1,-1,0.6],size(shelfPoints,1),1);


% Front Counter 
[Y,Z] = meshgrid(-1.4:0.05:1.4,-1.4:0.05:1.4);
sizeMat = size(Y);
X = repmat(1.4,sizeMat(1),sizeMat(2));
oneSideOfFrontC = surf(X,Y,Z);

% Combine one surface as a point cloud
frontCPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
frontCPoints = [ frontCPoints ...
             ; frontCPoints * rotz(pi/2)...
             ; frontCPoints * rotz(pi) ...
             ; frontCPoints * rotz(3*pi/2) ...
             ; frontCPoints * roty(pi/2) ...
             ; frontCPoints * roty(-pi/2)];       
         
try delete(X); end
try delete(oneSideOfFrontC); end   

% Plot the cube's point cloud         
frontCPoints = frontCPoints + repmat([0,2.4,-0.9],size(frontCPoints,1),1);


% Coffee Dispenser

[Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
sizeMat = size(Y);
X = repmat(0.5,sizeMat(1),sizeMat(2));
oneSideOfCoffeeD = surf(X,Y,Z);

% Combine one surface as a point cloud
coffeeDPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
coffeeDPoints = [ coffeeDPoints ...
             ; coffeeDPoints * rotz(pi/2)...
             ; coffeeDPoints * rotz(pi) ...
             ; coffeeDPoints * rotz(3*pi/2) ...
             ; coffeeDPoints * roty(pi/2) ...
             ; coffeeDPoints * roty(-pi/2)];         
         
try delete(X); end
try delete(oneSideOfCoffeeD); end   

% Plot the cube's point cloud         
coffeeDPoints = coffeeDPoints + repmat([-0.7,-0.95,0.7],size(coffeeDPoints,1),1);

% Back Counter

[Y,Z] = meshgrid(-1.8:0.05:1.8,-1.8:0.05:1.8);
sizeMat = size(Y);
X = repmat(1.8,sizeMat(1),sizeMat(2));
oneSideOfBackC = surf(X,Y,Z);

% Combine one surface as a point cloud
backCPoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
backCPoints = [ backCPoints ...
             ; backCPoints * rotz(pi/2)...
             ; backCPoints * rotz(pi) ...
             ; backCPoints * rotz(3*pi/2) ...
             ; backCPoints * roty(pi/2) ...
             ; backCPoints * roty(-pi/2)];         
 
try delete(X); end
try delete(oneSideOfBackC); end   

% Plot the cube's point cloud         
backCPoints = backCPoints + repmat([0,-2,-1.3],size(backCPoints,1),1);

%% Collision checking of Kuka wih environment

kukaPose = kuka.model.getpos;
tr = kuka.model.fkine(kukaPose);

% Concatenate all environment points into 1 array
allPoints = [backCPoints;
    coffeeDPoints;
    coffeeGrPoints;
    frontCPoints;
    heatedFoodPoints;
    lightCurtPoints;
    tamperBPoints;
    tamperFPoints];

% transform the environment points into the local robot coordinate frame
allPointsAndOnes = [inv(tr) * [allPoints,ones(size(allPoints,1),1)]']';

allPointsUpdated = allPointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(allPointsUpdated, centerPoint, radii);
pointsInside = find(algebraicDist < 0.05);
display(['There are now ', num2str(size(pointsInside,1)),' points inside']);


q = [0,0,0,0,0,0,0]
tr = zeros(4,4,kuka.model.n+1);
tr(:,:,1) = kuka.model.base;
L = kuka.model.links;
for i = 1 : kuka.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each ellipsoid
for i = 1: size(tr,3)
    allPointsAndOnes = [inv(tr(:,:,i)) * [allPoints,ones(size(allPoints,1),1)]']';
    allPointsUpdated = allPointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(allPointsUpdated, centerPoint, radii);
    pointsInside = find(algebraicDist < 0.05);
    display(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end

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
% Plot3D model showing where the end effector can be over all points in the robot workspace.
plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'r.');

% Calculate the volume of the robot workspace
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
% find robot joint angles for each object's position

kuka = Kuka;
%dobot = DobotBarista;

% dobotQStart = dobot.model.getpos;
% dobotPose = dobot.model.fkine(dobotQStart);
% dobotPose = dobotPose*transl(0,0,0.1);
% dobotQStart = dobot.model.ikcon(dobotPose, [0,0,0,pi,0]);
% dobot.model.animate(dobotQStart);


% write kuka = Kuka into cmd window
kuka = Kuka;  

pf = Portafilters(1);
pf.portafilter{1}.base = eye(4)*transl(-0.9,-0.1, 0.85)*trotz(pi);
pf.portafilter{1}.base = pf.portafilter{1}.base*trotz(pi);
pf.portafilter{1}.animate(pf.portafilter{1}.base);

coffeeCup = Cups(1);
cupOrigin = eye(4)*transl(-0.3,0.9,0.55);
coffeeCup.cup{1}.base = cupOrigin;
coffeeCup.cup{1}.animate(coffeeCup.cup{1}.base);
%%
view(90,45)

steps = 50;

%
kukaStPose = [0,0,0,0,0,0,0];
kuka.model.animate(kukaStPose);

qToPfStart = kuka.model.ikcon(pf.portafilter{1}.base); %*trotz(pi/2)
  
% Calculate the corresponding the poses of robot and portafilter
moveKukaToPf = jtraj(kukaStPose,qToPfStart, steps);

% for i=1:1:steps
%     kuka.model.animate(moveKukaToPf(i,:));
%     pause(0.01)
% end

view(90,30)


% Create poses for intermediate point between the coffee machine and grinder
pPfToGrinder = eye(4)*transl(-0.9,0.6,0.65)*trotz(pi);
qMachineToGrinder = kuka.model.ikcon(pPfToGrinder,[0,0,pi,0,pi,0,0]);

movePfToGrinder = jtraj(qToPfStart,qMachineToGrinder,steps);


% for i=1:1:steps
%     kuka.model.animate(movePfToGrinder(i,:));
%     pf.portafilter{1}.base = kuka.model.fkine(...
%         movePfToGrinder(i,:));
%     pf.portafilter{1}.animate(pf.portafilter{1}.base);
%     pause(0.01)
% end


for i=1:1:steps
    kuka.model.animate(movePfToGrinder(i,:));
    pf.portafilter{1}.base = kuka.model.fkine(...
        movePfToGrinder(i,:));
    pf.portafilter{1}.animate(pf.portafilter{1}.base);
    pause(0.01)
end
pause(5);

pTamperInter = eye(4)*transl(-0.5,0.3,0.6)*trotz(pi);
qToTamperInter = kuka.model.ikcon(pTamperInter,[0,0,pi,0,pi,pi/2,0]);

movePfToTamperInter = jtraj(qMachineToGrinder,qToTamperInter,steps);

view(90,30)

% Animate trajectory of robot from start to portafilter
% for i=1:1:steps
%     kuka.model.animate(movePfToTamperInter(i,:));
%     pf.portafilter{1}.base = kuka.model.fkine(...
%         movePfToTamperInter(i,:));
%     pf.portafilter{1}.animate(pf.portafilter{1}.base);
%     pause(0.01)
% end

view(85,15)

pTamper = eye(4)*transl(-0.66,0.6,0.52)*trotz(pi);
qToTamper = kuka.model.ikcon(pTamper,[0,0,2*pi,0,3*pi/4,3*pi/4,3*pi/4]);

moveToTamper = jtraj(qToTamperInter,qToTamper,steps);

% for i=1:1:steps
%     kuka.model.animate(moveToTamper(i,:));
%     pf.portafilter{1}.base = kuka.model.fkine(...
%         moveToTamper(i,:));
%     pf.portafilter{1}.animate(pf.portafilter{1}.base);
%     pause(0.01)
% end

% Animate trajectory of robot from start to portafilter
% for i=1:1:steps
%     kuka.model.animate(moveToTamper(i,:));
%     pf.portafilter{1}.base = kuka.model.fkine(...
%         moveToTamper(i,:));
%     pf.portafilter{1}.animate(pf.portafilter{1}.base);
%     pause(0.01)
% end

moveTamperToMachine = jtraj(qToTamper,qToPfStart,steps);

% for i=1:1:steps
%     kuka.model.animate(moveTamperToMachine(i,:));
%     pf.portafilter{1}.base = kuka.model.fkine(...
%         moveTamperToMachine(i,:));
%     pf.portafilter{1}.animate(pf.portafilter{1}.base);
%     pause(0.01)
% end

for i=1:1:steps
    kuka.model.animate(moveToTamper(i,:));
    pf.portafilter{1}.base = kuka.model.fkine(...
        moveToTamper(i,:));
    pf.portafilter{1}.animate(pf.portafilter{1}.base);
    pause(0.01)
end
pause(5);
% Animate trajectory of robot from start to portafilter
moveTamperToMachine = jtraj(qToTamper,qToPfStart,steps);


for i=1:1:steps
    kuka.model.animate(moveTamperToMachine(i,:));
    pf.portafilter{1}.base = kuka.model.fkine(...
        moveTamperToMachine(i,:));
    pf.portafilter{1}.animate(pf.portafilter{1}.base);
    pause(0.01)
end

cupOffset = transl(0,-0.07,0.06)*troty(-pi/2)...
    *trotx(-pi/2);

pGrapCup = coffeeCup.cup{1}.base*cupOffset;
qGrabCup = kuka.model.ikcon(pGrapCup);
kuka.model.animate(qGrabCup);

pCupToMachine = pf.portafilter{1}.base*transl(0,-0.1,-0.09);
pCupToMachine = pCupToMachine*troty(-pi/2)*trotx(pi/2);
qCupToMachine = kuka.model.ikcon(pCupToMachine,[0,0,2*pi,0,2*pi,0,0]);

pickupCup = jtraj(qGrabCup,qCupToMachine,steps);
%%
for i=1:1:steps
    kuka.model.animate(pickupCup(i,:));
    coffeeCup.cup{1}.base = kuka.model.fkine(...
        pickupCup(i,:))*transl(-0.05,0,0.075)*troty(pi/2);
    coffeeCup.cup{1}.animate(coffeeCup.cup{1}.base);
    pause(0.01)
end

qLocks = kuka.model.getpos;
%%
coffeeInter = eye(4)*transl(-0.9,0.2,cupOrigin(3,4))*troty(-pi/2)*trotx(pi/2);
qCoffeeInter = kuka.model.ikcon(coffeeInter,qLocks);
moveCoffeeInter = jtraj(qCupToMachine,qCoffeeInter,2*steps);
%%
for i=1:1:2*steps
    kuka.model.animate(moveCoffeeInter(i,:));
    coffeeCup.cup{1}.base = kuka.model.fkine(...
        moveCoffeeInter(i,:))*transl(-0.05,0,0.075)*troty(pi/2);
    coffeeCup.cup{1}.animate(coffeeCup.cup{1}.base);
    pause(0.01)
end
qLocks2 = kuka.model.getpos;
%%
coffeeInter2 = eye(4)*transl(-0.9,0.6,cupOrigin(3,4))*troty(-pi/2)*trotx(0);
qCoffeeInter2 = kuka.model.ikcon(coffeeInter2,[0,0,pi,pi,pi,pi,qLocks2(1,7)]);
moveCoffeeInter2 = jtraj(qCoffeeInter,qCoffeeInter2,2*steps);

for i=1:1:2*steps
    kuka.model.animate(moveCoffeeInter2(i,:));
    coffeeCup.cup{1}.base = kuka.model.fkine(...
        moveCoffeeInter2(i,:))*transl(-0.05,0,0.075)*troty(pi/2);
    coffeeCup.cup{1}.animate(coffeeCup.cup{1}.base);
    pause(0.01)
end

qLocks3 = kuka.model.getpos;
%%
cupOrigin2 = cupOrigin*transl(0,-0.1,0.1)*troty(-pi/2)*trotx(-pi/2);

qCupOrigin = kuka.model.ikcon(cupOrigin2,[0,qLocks3(1,2),0,qLocks3(1,4),-pi/8,2*pi,qLocks3(1,7)]);
moveCoffeeToDobot = jtraj(qCoffeeInter2,qCupOrigin,2*steps);

for i=1:1:2*steps
    kuka.model.animate(moveCoffeeToDobot(i,:));
    coffeeCup.cup{1}.base = kuka.model.fkine(...
        moveCoffeeToDobot(i,:))*transl(-0.05,0,0.075)*troty(pi/2);
    coffeeCup.cup{1}.animate(coffeeCup.cup{1}.base);
    pause(0.01)
end
%%

dobotStart = dobot.model.getpos;
qDobotStir = dobot.model.ikcon(transl(0,-0.2,0.25));


function dist=dist2pts(pt1,pt2)

%% Calculate distance (dist) between consecutive points
% If 2D
if size(pt1,2) == 2
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2);
% If 3D          
elseif size(pt1,2) == 3
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2+...
              (pt1(:,3)-pt2(:,3)).^2);
% If 6D like two poses
elseif size(pt1,2) == 6
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
              (pt1(:,2)-pt2(:,2)).^2+...
              (pt1(:,3)-pt2(:,3)).^2+...
              (pt1(:,4)-pt2(:,4)).^2+...
              (pt1(:,5)-pt2(:,5)).^2+...
              (pt1(:,6)-pt2(:,6)).^2);
end
end

%% GetAlgebraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:* 
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end


