% Filtering - Clustering - Implicit Associating cycle

if temp_N > 1
    [row, col] = initializeParticles(image, D);
    numParticles = numel(row); % Number of particles equals the number of pixels found
end


while true
    roundCount = roundCount + 1;
    
    % Particle Filtering
    disp('Filtering...')
    [particleFilter,weight_sum] = particleFiltering(beta, roundCount,sigma,numTimeSteps, temp_N, updatedAzEstTable, robotPoses, srcGroundTruth,image,row, col,resolution);
    pfResults{roundCount}.particles = particleFilter.particles;
    pfResults{roundCount}.particleWeight = particleFilter.weights;
    pfResults{roundCount}.roundCount = roundCount;

    % DBSCAN clustering
    disp('Clustering...')
    clusteredParticleSet = getMaxClusterFromDBSCAN(particleFilter, epsilon, MinPts); % cluster with most particles
   
    if ~isempty(clusteredParticleSet)
        clusterResults{roundCount}.particles = clusteredParticleSet{1}.particles;
        clusterResults{roundCount}.roundCount = roundCount;
    else
        roundCount = roundCount - 1;
        disp('Clustering failed.')
        break;
    end

    % Associate observations and update the DoA Estimates Table  
    disp('Associating...')
    [associatedParticleSet, updatedAzEstTable,updatedElEstTable] = associateObservations_3D(clusteredParticleSet, numTimeSteps, numObservations, updatedAzEstTable, updatedElEstTable, robotPoses, associatedRange);
    
    % 计算z坐标
    zEstList = calculateZ(associatedParticleSet,robotPoses);
    if isempty(zEstList)
        roundCount = roundCount - 1;
        disp('Calculation of z failed.')
        break
    end
    associatedParticleSet{1}.zEstList = zEstList;
    labels = dbscan(zEstList, 2, 2); 
    mainClusterValues = zEstList(labels > 0); % exclude noise
    zEst = mean(mainClusterValues);
    if zEst<0  % non-negative
       zEst=0;
    elseif isnan(zEst)
        continue
    end
    associatedParticleSet{1}.zEst = zEst;
    associatedParticleSet{1}.State = [associatedParticleSet{1}.State,zEst];

    % Add new estimate to the set
    detectedSourceFilters = [detectedSourceFilters, associatedParticleSet];  
    disp(['-----Round ',num2str(roundCount),' finished.-----']);
end

disp(['----------Cycle ',num2str(temp_N),' finished.----------']);

I = numel(detectedSourceFilters); % number of detected sources

% plot final map result
% figure;
% imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
% set(gca, 'YDir', 'normal');
% hold on;
% legendEntries = gobjects(2, 1);
% for i = 1:I 
%     particleFilter = detectedSourceFilters{i};
%     % plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10); 
%     scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');      
% end
% scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
% scatter(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 10, 'k', 'filled'); 
% plot(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 'k--'); 
% 
% title("2D Mapping Results, Cycle "+num2str(temp_N));
% axis equal;
% axis on;
% axis image