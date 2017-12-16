%% SETUP STEP
connectToRobot;
% Create publishers and subscribers for navigation
mapSub = rossubscriber('/map');

%% Receive, show, and save the latest map from the /map topic
% Assumes you are using a gmapping tutorial like this one:
% http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM
mapMsg = mapSub.LatestMessage;
map = readOccupancyGrid(mapMsg);
robotRadius = 0.1;
inflate(map,robotRadius);
show(map)
% save myMaps map % Uncomment this to save the map