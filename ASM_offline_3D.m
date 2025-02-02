clc
clear
close all

addpath("functions\")

%% Load experiment data

% Source Arrangement I,  K = 71
% Source Arrangement II,  K = 40
% Source Arrangement III,  K = 76
% Source Arrangement IV, K = 28, duration = 2.82s
% Source Arrangement V, K = 62, duration = 2.82s
% Source Arrangement VI, K = 55, duration = 2.82s
arrangement = "VI"; 
SSLmethod = "MVDR"; % 'MVDR' or 'GCC-PHAT'
params = loadExperimentData_3D(arrangement, SSLmethod); 
baseDir = ".\exp_data\";

robotPoses = params.robotPoses;
azEstTable = params.azEstTable;
elEstTable = params.elEstTable;
image = params.image;
resolution = params.resolution;
origin = params.origin;
srcGroundTruth = params.srcGroundTruth;
numObservations = params.numObservations; % default N
sigma = params.sigma; % default: 5 degrees
associatedRange = params.associatedRange;
beta = 50; % default: 50, the larger the value, the less sensitive to distance
D = 16; % default: 16

 %% Initialization

numTimeSteps = size(robotPoses,1); % number of discrete time steps, K = size(robotPoses,1)
[robotPoses, srcGroundTruth] = convertCoordinates(robotPoses, srcGroundTruth, origin, resolution);

% Initialize particle filter parameters
[row, col] = initializeParticles(image, D);
numParticles = numel(row); % Number of particles equals the number of pixels found

% DBSCAN clustering parameters
epsilon = 0.1/resolution; % Distance threshold used to define neighborhoods. If the distance between two points is less than or equal to epsilon, then these two points are considered neighbors.
MinPts = numParticles*0.1; % Quantity threshold used to define core points. If an epsilon-neighborhood of a point contains at least MinPts points (including the point itself), then this point is considered a core point.

detectedSourceFilters = [];
updatedAzEstTable = azEstTable; % Initialize the updated DoA Estimates Table -azi
updatedElEstTable = elEstTable; % Initialize the updated DoA Estimates Table -ele

%% Filtering - Clustering - Implicit Associating cycle

% Start timer
tic
roundCount = 0; % iteration round count
temp_N = 0;  

while temp_N < numObservations
    temp_N = temp_N+1;  % N'
    run("cycle.m")
end

% Merge similar estimates
merge_tresh = 0.5/resolution; % 0.5m
detectedSourceFilters = mergeClusters_3D(detectedSourceFilters, merge_tresh);

elapsedTime = toc; % End timer
disp(['The mapping took ', num2str(elapsedTime), ' seconds.']);

%% Error analysis - OSPA distance

c = 1; % cutoff distance 1m
p = 1; %  first order

[OSPA, locOspa,cardOspa] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p);
mapping_result_CA = [OSPA, locOspa,cardOspa,elapsedTime]; 
fprintf('OSPA distance is %.3f m.\n', OSPA);
fprintf('OSPA localization error is %.3f m.\n', locOspa);
fprintf('OSPA cardility error is %.3f m\n', cardOspa);

%% Visualize ASM results -2D

I = numel(detectedSourceFilters); % number of detected sources

imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
set(gca, 'YDir', 'normal');
set(gcf,'Position',[-10.2,45.8,673.6,518.4])
set(gcf, 'MenuBar', 'none'); % hide menubar
hold on;
legendEntries = gobjects(2, 1);
for i = 1:I 
    particleFilter = detectedSourceFilters{i};
    if i==1
        legendEntries(2) = plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10);
        legendEntries(3) = scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');
    else
        plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10); 
        scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled'); 
    end      
end
legendEntries(1) = scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
legendEntries(4) = scatter(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 10, 'k', 'filled'); 
legendEntries(5) = plot(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 'k--'); 

% plot N DoA observations of current time step k
for i = 1:numObservations 
    azi = wrapToPi(azEstTable(numTimeSteps,i)+ deg2rad(robotPoses(numTimeSteps,4)));
    legendEntries(6) = quiver(robotPoses(numTimeSteps,1), robotPoses(numTimeSteps,2), cos(azi), sin(azi),10); 
end

legend(legendEntries, 'Source ground truth','Clustered particles','Source estimate', 'Robot position','Robot trajectory','DoA estimate','Location', 'north east');
title("2D Mapping Results, k = "+num2str(numTimeSteps));
% title("2D Mapping Results");
axis equal;
axis on;
axis image

%% Visualize ASM results -3D

figure;
set(gcf, 'MenuBar', 'none'); 
set(gcf,'Position',[-1.4,593.8,663.2,250])
hold on;

% Plot source estimate (State)
for i = 1:I
  particleFilter = detectedSourceFilters{i};
  if i==1
    legendEntries3D(2) = scatter3(particleFilter.State(:,1), particleFilter.State(:,2), particleFilter.State(:,3), 50, 'b^', 'filled');     
  else
    scatter3(particleFilter.State(:,1), particleFilter.State(:,2), particleFilter.State(:,3), 50, 'b^', 'filled');  
  end
end

% Plot source ground truth
legendEntries3D(1) = scatter3(srcGroundTruth(:,1), srcGroundTruth(:,2), srcGroundTruth(:,3), 100, 'rp', 'filled'); 

% Plot robot position and trajectory
legendEntries3D(3) = scatter3(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), robotPoses(1:numTimeSteps,3), 10, 'k', 'filled'); 
legendEntries3D(4) = plot3(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), robotPoses(1:numTimeSteps,3), 'k--'); 

for i = 1:numObservations 
    azi = wrapToPi(azEstTable(numTimeSteps,i)+ deg2rad(robotPoses(numTimeSteps,4)));
    ele = wrapToPi(elEstTable(numTimeSteps,i));
    quiver3(robotPoses(numTimeSteps,1), robotPoses(numTimeSteps,2), robotPoses(numTimeSteps,3), cos(ele)*cos(azi), cos(ele)*sin(azi), sin(ele),10); % plot N DoA observations of current time step k
end

legend(legendEntries3D, 'Source ground truth','Source estimate', 'Robot position','Robot trajectory','Location', 'north east');
title("3D Mapping Results, k = "+num2str(numTimeSteps));
% title("3D Mapping Results");
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([70,260])
zlim([-10,20])
axis equal;
grid on;
view(30,15)
% view(-25.0315,7.2761)







