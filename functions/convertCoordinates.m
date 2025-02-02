function [robotPoses, srcGroundTruth] = convertCoordinates(robotPoses, srcGroundTruth, origin, resolution)
    % Convert coordinates for robotPoses and srcGroundTruth from map coordinates to image coordinates
    
    robotPoses(:,1) = (robotPoses(:,1) - origin(1)) / resolution; % x
    robotPoses(:,2) = (robotPoses(:,2) - origin(2)) / resolution; % y
    robotPoses(:,3) = (robotPoses(:,3) - origin(3)) / resolution; % z
    
    srcGroundTruth(:,1) = (srcGroundTruth(:,1) - origin(1)) / resolution;
    srcGroundTruth(:,2) = (srcGroundTruth(:,2) - origin(2)) / resolution;
    srcGroundTruth(:,3) = (srcGroundTruth(:,3) - origin(3)) / resolution;
end
