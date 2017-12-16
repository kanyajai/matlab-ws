function [v,w,grip] = trackObject(objLocation,objDepth,imgWidth)

   % Initialize outputs
   v = 0; 
   w = 0;
   validPos = false;
   validDepth = false;

   % Angular velocity
   posThresh = 20; % pixels
   if objLocation(1) > imgWidth/2 + posThresh
       w = -0.3;
   elseif objLocation(1) < imgWidth/2 - posThresh
       w = 0.3;
   else
       validPos = true;
   end
   
   % Linear velocity
   depthThresh = 0.025; % meters
   targetDepth = 0.7;   % meters
   if objDepth > targetDepth + depthThresh
       v = 0.05;
   elseif objDepth < targetDepth - depthThresh
       v = -0.05;
   else
      validDepth = true; 
   end

   % Find if both the depth and position are valid for gripping
   grip = validPos & validDepth;
   
end

