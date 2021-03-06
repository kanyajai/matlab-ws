function [objLocation,objDepth] = detectObject(img,depthImg,objType)

    % Create blob detector system object
    persistent blobDetector
    if isempty(blobDetector)
        blobDetector = vision.BlobAnalysis( ...
                        'MinimumBlobArea',300, ...
                        'MaximumCount',10);
    end

    % Threshold the image based on autogenerated color thresholder code
    if strcmp(objType,'blue')
        imgBW = createMaskBlue(img);
    elseif strcmp(objType,'red')
        imgBW = createMaskRed(img);
    end
    % Find blobs
    [areas,centroids] = blobDetector(imgBW);
    
    % Find the centroid and depth of the biggest blob, if any
    objLocation = [];
    objDepth = [];
    if ~isempty(areas)
       % Get centroid of biggest area blob
       [~,maxIdx] = max(areas);
       objLocation = round(centroids(maxIdx,:));
       objLocation(1) = min(objLocation(1),size(img,1));
       objLocation(2) = min(objLocation(2),size(img,2));
       % Get depth at that location
       depthData = depthImg(objLocation(1),objLocation(2));
       % Scale the depth data to actual depth based on calibration values
       objDepth = double(depthData);
    end
    
end