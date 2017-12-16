%% Import the robot
robot = importrobot('open_manipulator_chain.urdf')
% Add end effector frame, offset from the grip link frame
eeOffset = 0.12;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(robot,eeBody,'link5');
% Show the robot
show(robot);

%% Inverse kinematics
% The end effector is at gripper_link (body index 24)
targetPos = [0.25 -0.15 0.05];
targetTform = trvec2tform(targetPos);
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];
initGuess = robot.homeConfiguration;
[ikSoln,ikInfo] = ik('end_effector',targetTform,weights,initGuess);

%% Display IK solution
actualTform = getTransform(robot,ikSoln,'end_effector');
actualPos = tform2trvec(actualTform)
posError = ikInfo.PoseErrorNorm
iters = ikInfo.Iterations
show(robot,ikSoln,'Frames','off');
hold on;
plot3(targetPos(1),targetPos(2),targetPos(3),...
      'ro','MarkerSize',12,'LineWidth',2);
  
%% Repeat the process for an entire trajectory
hold off
initGuess = robot.homeConfiguration;
startPoint = tform2trvec(getTransform(robot,initGuess,'end_effector'));
waypoints = [startPoint; ...
             0.4 0.0 0.05; ...
             0.2 0.2 -0.1; ...
             -0.2 0.2 0.15; ...
             0.05 0.05 0.2];
show(robot,'Frames','off');
hold on;
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),...
      'ro','MarkerSize',12,'LineWidth',2);
numSteps = 5; % Number of steps between each set of waypoints
for ii = 1:size(waypoints,1)-1
    for jj = 1:numSteps
        targetPos = waypoints(ii,:) + ((jj-1)/numSteps)*(waypoints(ii+1,:)-waypoints(ii,:));
        targetTform = trvec2tform(targetPos);
        ikSoln = ik('end_effector',targetTform,weights,initGuess);
        initGuess = ikSoln;
        
        % Plot the solution
        show(robot,ikSoln,'Frames','off')
        hold on;
        plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),...
              'ro','MarkerSize',12,'LineWidth',2);
        hold off;
        drawnow
    end
end