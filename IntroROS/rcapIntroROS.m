%% SETUP STEP
connectToRobot;

%% View the list of topics and drill into the odometry topic
rostopic list
rostopic info /odom
% rostopic echo /odom % Ctrl+C to stop this

%% Create velocity publisher and send velocity commands
[vPub,vMsg] = rospublisher('/mobile_base/commands/velocity');
vMsg.Linear.X = 0.1;
vMsg.Angular.Z = 0.5;
send(vPub,vMsg);

% %% Create odometry subscriber and view the results
% odomSub = rossubscriber('/odom');
% odomMsg = receive(odomSub);
% posData = [odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y];
% for idx = 1:20
%     % Send velocity message
%     send(vPub,vMsg);
%     % Extract the odometry data
%     odomMsg = odomSub.LatestMessage;
%     posData = [posData; odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y];
%     pause(0.5);
% end
% % Visualize the data
% plot(posData(:,1),posData(:,2),'bo-');
% title('Robot Motion')
% axis equal
% 
% 
% %% Create scan subscriber and view the results
% scanSub = rossubscriber('/scan');
% receive(scanSub);
% for idx = 1:20
%     % Send velocity message
%     send(vPub,vMsg);
%     % Visualize the scan data
%     plot(scanSub.LatestMessage);
%     pause(0.5);
% end

%% Create camera subscriber and view the results
imgSub = rossubscriber('/camera/rgb/image_rect_color');
receive(imgSub);
for idx = 1:20
    % Send velocity message
    send(vPub,vMsg);
    % Visualize the image data
    img = readImage(imgSub.LatestMessage);
    imshow(img);
    pause(0.5);
end