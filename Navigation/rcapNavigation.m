%% SETUP STEP
connectToRobot;
% Create publishers and subscribers for navigation
odomSub = rossubscriber('/odom');
[vPub,vMsg] = rospublisher('/mobile_base/commands/velocity');
% Reset the odometry to zero
resetOdometry;sh

%% Get goal point from the speech node
% TODO: Get string from ROS speech module
speechStr = 'Get the red object from the kitchen';
[goalPoint,objType] = parseCommand(speechStr)
if isempty(goalPoint)
    error('Destination could not be found.');
end
    
%% Do path planning
% First, load the presaved map
load myMaps
% Then, create a probabilistic roadmap (PRM)
prm = robotics.PRM(map);
prm.NumNodes = 300;
prm.ConnectionDistance = 2.5;
% Define a start and goal point and find a path
pose = getRobotPose(odomSub);
startPoint = pose(1:2);
myPath = findpath(prm,startPoint,goalPoint);
show(prm)

%% Perform navigation using Pure Pursuit
% First, create the controller and set its parameters
pp = robotics.PurePursuit;
pp.DesiredLinearVelocity = 0.2;
pp.LookaheadDistance = 0.5;
pp.Waypoints = myPath;

% Navigate until the goal is reached within threshold
show(prm); 
hold on;
hPose = plot(pose(1),pose(2),'gx','MarkerSize',15,'LineWidth',2);
pose = getRobotPose(odomSub);
while norm(goalPoint-pose(1:2)) > 0.1 
    % Get latest pose
    pose = getRobotPose(odomSub);     
    % Run the controller
    [v,w] = pp(pose); 
    % Assign speeds to ROS message and send
    vMsg.Linear.X = v;
    vMsg.Angular.Z = w;
    send(vPub,vMsg);   
    % Plot the robot position
    delete(hPose);
    hPose = plot(pose(1),pose(2),'gx','MarkerSize',15,'LineWidth',2);
    drawnow;
end
disp('Reached Goal!');