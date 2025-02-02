function [associatedParticleSet,updatedAzEstTable,updatedElEstTable]  = associateObservations_3D(clusteredParticleSet, numTimeSteps, numObservations, azEstTable,elEstTable, robotPoses, associatedRange)
    
    associatedRange = wrapToPi(associatedRange); % Association range

    % Initialize the observations array for each particle filter, only one
    for i = 1:numel(clusteredParticleSet)
        clusteredParticleSet{i}.timeStep = [];
        clusteredParticleSet{i}.azEst = [];
        clusteredParticleSet{i}.elEst = [];
    end

    % Iterate over each time step and each observation interval
    for t = 1:numTimeSteps
        for j = 1:numObservations
            minDiff = associatedRange;
            maxIndex = 0;
            angle = wrapToPi(azEstTable(t,j) + deg2rad(robotPoses(t,4)));  % Convert azimuth to the world coordinate system, in radians
            
            % Iterate over each particle filter
            for i = 1:numel(clusteredParticleSet)
                diff = clusteredParticleSet{i}.State - robotPoses(t,1:2); % Calculate the position difference between the cluster center and the robot
                particleAngles = atan2(diff(:,2), diff(:,1));
                angleDiff = abs(angle - particleAngles); % Calculate the difference between the observation ray angle and the particle angle delta_theta       
                
                % If the angle difference is not within the association range, do not associate
                if (angleDiff < minDiff)
                    minDiff = angleDiff;
                    maxIndex = i;
                end

            end
            
            % If a corresponding filter is found, store the observation in the corresponding filter's observations array and mark the corresponding element in the original observation matrix as NaN
            if maxIndex > 0
                if abs(elEstTable(t,j)) >= pi/2 - 1e-6
                    warning('Elevation angle is too close to 90 degrees, resulting in an infinite z value.'); 
                    break % do not associate
                else
                    clusteredParticleSet{maxIndex}.elEst = [clusteredParticleSet{maxIndex}.elEst; elEstTable(t,j)];
                     
                    clusteredParticleSet{maxIndex}.timeStep = [clusteredParticleSet{maxIndex}.timeStep; t]; 
                    clusteredParticleSet{maxIndex}.azEst = [clusteredParticleSet{maxIndex}.azEst; azEstTable(t,j)];
                    
                    azEstTable(t,j) = NaN;
                    elEstTable(t,j) = NaN;
                end
            end
        end
    end
    updatedAzEstTable = azEstTable;
    updatedElEstTable = elEstTable;
    associatedParticleSet = clusteredParticleSet;
end