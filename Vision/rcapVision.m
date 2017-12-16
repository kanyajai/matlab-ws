%% SETUP STEP
connectToRobot;
% Create publishers and subscribers for vision
imgSub = rossubscriber('/camera/rgb/image_rect_color');
depthSub = rossubscriber('/camera/depth_registered/image_raw');
[vPub,vMsg] = rospublisher('/mobile_base/commands/velocity');

%% GET FIRST IMAGES
imgMsg = receive(imgSub);
img = readImage(imgMsg);
depthMsg = receive(depthSub);
depthImg = readImage(depthMsg);
% colorThresholder(img) % Open up the Color Thresholder App

%% OBJECT DETECTION + TRACKING
objType = 'red';
gripCount = 0;
% Create video player for visualization
vidPlayer = vision.DeployableVideoPlayer;
while(1) %(gripCount < 20)
    % Get image data
    imgMsg = imgSub.LatestMessage;
    img = readImage(imgMsg);
    depthMsg = depthSub.LatestMessage;
    depthImg = readImage(depthMsg);
    
    % Detect object
    [objLocation,objDepth] = detectObject(img,depthImg,objType);
    
    % Visualize and track only if an object is found
    if ~isempty(objDepth) %&& (objDepth>0)
       % Visualize
       img = insertShape(img,'Circle',[objLocation(1) objLocation(2) 20],'LineWidth',2);
       img = insertText(img,[0 0],['Depth: ' num2str(objDepth) ' m']);    
       % Track object
       imgWidth = size(img,2);
       [v,w,grip] = trackObject(objLocation,objDepth,imgWidth);
       if grip
          gripCount = gripCount + 1; 
       else
          gripCount = 0; 
       end
       % Publish velocity command
       vMsg.Linear.X = v;
       vMsg.Angular.Z = w;
       send(vPub,vMsg);
    end
    
    step(vidPlayer,img);
    
end
disp('Ready to grip!')
