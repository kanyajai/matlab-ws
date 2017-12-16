function showMap

    % Create map subscriber
    persistent mapSub
    if isempty(mapSub)
        mapSub = rossubscriber('/map');
    end

    %% Receive, show, and save the latest map from the /map topic
    % Assumes you are using a gmapping tutorial like this one:
    % http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM
    mapMsg = mapSub.LatestMessage;
    if ~isempty(mapMsg)
        map = readOccupancyGrid(mapMsg);
        show(map);
        drawnow;
    end
    
end