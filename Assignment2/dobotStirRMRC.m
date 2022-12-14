function dobotStirRMRC()
% Set parameters for RMRC

dobot = DobotBarista;

t = 0.2;             % Total time
steps = 50;         % No. of steps
deltaT = t/steps;   % Discrete time step
delta = 2*pi/steps; % Small angle change
epsilon = 1;       % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 1 1]);    % Weighting matrix for the velocity vector

% Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint angles
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% Create trajectory
s = lspb(0,1,steps); % Trapezoidal trajectory scalar

stir = zeros(4,3); % Array (4x3) with xyz of points to articulate to for stirring

for i=1:steps
    x(1,i) = (1-s(i))*stir(1,1) + s(i)*stir(2,1); % Points in x
    x(2,i) = (1-s(i))*stir(1,2) + s(i)*stir(2,2); % Points in y
    x(3,i) = (1-s(i))*stir(1,3) + s(i)*stir(2,3); % Points in z
    theta(1,i) = 0;                 % Roll angle
    theta(2,i) = 0;              % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end

% Create transformation of first point and angle
T = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) x(:,1); zeros(1,3) 1];

q0 = dobot.model.getpos; % Initial guess for joint angles

qMatrix(1,:) = dobot.model.ikcon(T,q0);  % Solve joint angles to achieve first waypoint

% Track the trajectory with RMRC
for i = 1:steps-1
    T = dobot.model.fkine(qMatrix(i,:));                 % Get forward transformation at current joint state
    deltaX = x(:,i+1)-T(1:3,4);                          % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));  % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                     % Current end-effector rotation matrix
    
    Rdot = (1/deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
    S = Rdot*Ra';                      % Skew symmetric! S(\omega)
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)

   % deltaR =         	                                  % Calculate rotation matrix error
        deltaTheta = tr2rpy(Rd*Ra');                      % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];      % Calculate end-effector velocity to reach next waypoint.
    
    % (Try using a weighting matrix to (de)emphasize certain dimensions)
    J = dobot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon % If manipulability is less than given threshold
            
        lambda =  (1 - pow((m(i)/epsilon),2))*0.5;  % Damping coefficient (try scaling it)
        invJ = inv(J*J'                                 % Apply Damped Least Squares pseudoinverse
    else
    invJ =;  % Don't use DLS
    
    end
    invJ =
    qdot(i,:) = ... % Solve the RMRC equation (you may need to transpose the vector)
        for ... % Loop through joints 1 to 6
            if ... % If next joint angle is lower than joint limit...
                ... % Stop the motor
            elseif ... % If next joint angle is greater than joint limit ...
                ... % Stop the motor
            end
        end
        qMatrix(i+1,:) = ... % Update next joint state based on joint velocities
            m(i) = ...  % Record manipulability
            positionError(:,i) = deltaX;  % For plotting
        angleError(:,i) = deltaTheta; % For plotting
    end
    
    % Plot RMRC results
    figure(1)
    plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
    p560.plot(qMatrix,'trail','r-')
    
    for i = 1:6
        figure(2)
        subplot(3,2,i)
        plot(qMatrix(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        ylabel('Angle (rad)')
        refline(0,p560.qlim(i,1));
        refline(0,p560.qlim(i,2));
        
        figure(3)
        subplot(3,2,i)
        plot(qdot(:,i),'k','LineWidth',1)
        title(['Joint ',num2str(i)]);
        ylabel('Velocity (rad/s)')
        refline(0,0)
    end
    
    figure(4)
    subplot(2,1,1)
    plot(positionError'*1000,'LineWidth',1)
    refline(0,0)
    xlabel('Step')
    ylabel('Position Error (mm)')
    legend('X-Axis','Y-Axis','Z-Axis')
    
    subplot(2,1,2)
    plot(angleError','LineWidth',1)
    refline(0,0)
    xlabel('Step')
    ylabel('Angle Error (rad)')
    legend('Roll','Pitch','Yaw')
    figure(5)
    plot(m,'k','LineWidth',1)
    refline(0,epsilon)
    title('Manipulability')
