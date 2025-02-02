clc
clear
close all

addpath("functions\")
addpath("mbss_locate\v2.0\localization_tools")
addpath("mbss_locate\v2.0\localization_tools\pair_angular_meths");

%% Load experiment data

arrangement = "IV"; 
SSLmethod = "GCC-PHAT"; % SSLmethod: 'MVDR' or 'GCC-PHAT'

baseDir = ".\exp_data\"; 
posePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "pose", "pose_theta.xlsx");
imagePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "map", sprintf("map_%s.pgm", arrangement));
audioPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "audio");

azimuthPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "online", sprintf("online_azEst_%s.xlsx", SSLmethod));
if exist(azimuthPath, 'file')
    delete(azimuthPath);
    disp(['File deleted: ', azimuthPath]);
end

elevationPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "online", sprintf("online_elEst_%s.xlsx", SSLmethod));
if exist(elevationPath, 'file')
    delete(elevationPath);
    disp(['File deleted: ', elevationPath]);
end

image = imread(imagePath);
image = flipud(image(1:250, 1:300));  
resolution = 0.05; % occupancy map resolution, copied from yaml
origin = [-4.000000, -5.000000, 0.000000]; % occupancy map origin, copied from yaml

if arrangement == "I"
    srcGroundTruth = [ 0, -1.2, 0;
                     2.5, -1.2, 0;
                     5.0, -1.2, 0;
                     7.5,    0, 0;
                     7.5,  3.6, 0;
                     5.0,  4.8, 0;
                     2.5,  4.8, 0;
                       0,  4.8, 0;
                   -1.25,  3.6, 0;
                   -1.25,  1.2, 0];
    numObservations = 3; % number of considered DoA per time step, N
elseif arrangement == "II"
    srcGroundTruth = [1.25,  0.6, 0;
                      1.25, -0.6, 0;
                      2.50,  0.6, 0;
                      2.50, -0.6, 0;
                      3.74,  0.6, 0;
                      3.74, -0.6, 0;
                      4.99,  0.6, 0;
                      4.99, -0.6, 0;
                      6.23,  0.6, 0;
                      6.23, -0.6, 0];
    numObservations = 4;
elseif arrangement == "III"
    srcGroundTruth = [0.54,1.09,0.775;
                      2.34,1.09,0.775;
                      4.14,1.09,0.775;
                      5.94,1.09,0.775;
                         0,-1.2,    0;
                      1.25,-1.2,    0;
                       2.5,-1.2,    0;
                      3.75,-1.2,    0;
                         5,-1.2,    0;
                      6.25,-1.2,    0];
    numObservations = 3;
 elseif arrangement == "IV" 
     srcGroundTruth = [1.25, 0.42, 0.68;
                       2.50, 0.60, 0.00;
                       3.75, 0.38, 0.67;
                       5.00, 0.60, 0.00;
                       6.24, 0.60, 0.00;
                       6.24,-1.20, 0.00;
                       5.00,-1.16, 0.68;
                       3.75,-1.20, 0.00;
                       2.50,-1.20, 0.68;
                       1.25,-1.20, 0.00];
      numObservations = 4; 
 elseif arrangement == "V" 
     srcGroundTruth = [1.25, -1.20,  0.00;
                       2.50,  0.44,  0.69; 
                       5.00, -1.20,  0.00; 
                      -2.21,  3.14,  0.00;
                       8.67,  1.20,  0.00; 
                       6.25,  3.24,  0.77; 
                       5.00,  4.80,  0.00; 
                       2.50,  3.85,  0.68;
                       0.00,  4.80,  0.00;
                       0.00,  1.05,  0.77];
     numObservations = 3;  
 elseif arrangement == "VI" 
     srcGroundTruth = [1.25, -1.20, 0.00;
                       2.50,  0.44, 0.68;
                       5.00, -1.20, 0.00;
                       6.25,  1.20, 0.00;
                       7.50,  1.20, 0.00; 
                       7.50, -1.20, 0.00;
                       2.50, -1.22, 0.68; 
                       6.25, -1.23, 0.68;
                       3.75,  0.60, 0.00;
                       0.00,  1.04, 0.77];
      numObservations = 4; %  
end

srcGroundTruth(:,1) = (srcGroundTruth(:,1)-origin(1))/resolution;srcGroundTruth(:,2) = (srcGroundTruth(:,2)-origin(2))/resolution; 
sigma = deg2rad(5);  % standard deviation of delta theta
associatedRange = 3*sigma; % observation associated range, gamma
beta = 50; % default: 50, the larger the value, the less sensitive to distance
D = 16; % default: 16

[row, col] = initializeParticles(image, D);
numParticles = numel(row); % Number of particles equals the number of pixels found

% DBSCAN clustering parameters
epsilon = 0.1/resolution; % Distance threshold used to define neighborhoods. If the distance between two points is less than or equal to epsilon, then these two points are considered neighbors.
MinPts = numParticles*0.1; % Quantity threshold used to define core points. If an epsilon-neighborhood of a point contains at least MinPts points (including the point itself), then this point is considered a core point.

numTimeSteps = 0;
hFig2D_init = figure;
imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)], 'InitialMagnification', 200);
set(gca, 'YDir', 'normal');
set(hFig2D_init, 'Position', [-10.2, 45.8, 673.6, 518.4]);
set(hFig2D_init, 'MenuBar', 'none'); 
hold on;
hGroundTruth = scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
legend(hGroundTruth, 'Source ground truth');
title("2D Mapping Results, k = " + num2str(numTimeSteps));
legend(hGroundTruth, 'Source ground truth');
axis equal;
axis on;
axis image;
pause(0.1);

hFig3D_init = figure;
set(hFig3D_init, 'MenuBar', 'none'); 
set(hFig3D_init, 'Position', [-1.4, 593.8, 663.2, 250]);
hold on
hGroundTruth = scatter3(srcGroundTruth(:,1), srcGroundTruth(:,2), srcGroundTruth(:,3), 100, 'rp', 'filled'); 
legend(hGroundTruth, 'Source ground truth')
title("3D Mapping Results, k = " + num2str(numTimeSteps));
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([60, 260]);
zlim([-10, 20]);
axis equal;
grid on;
view(30, 15);
pause(0.1);
%% 
fileCounter = 0;
last_length = 0;
while true
    disp('waiting for the audio files ...')
    % Get a list of all the wav files in the folder
    files = dir(fullfile(audioPath, '*.wav'));

    % Check if there are any new files
    while length(files) > fileCounter
        fileCounter = fileCounter + 1;
        fprintf('Calculating obervation %d:',fileCounter)
        pause(0.5)
        filename_wav = fullfile(audioPath, files(fileCounter).name);
        DOAs = SSL_3D(filename_wav, SSLmethod); % 计算DoA
        azEst = DOAs.azi;
        elEst = DOAs.ele;
        writematrix(azEst,azimuthPath, 'WriteMode', 'append'); % azimuth表
        writematrix(elEst,elevationPath, 'WriteMode', 'append'); % elevation表
    end
        if  (fileCounter >= 1) && (length(files) > last_length) 
            last_length = length(files);
            % convertQuaternion2Theta(arrangement); % 将原始pose数据转换为theta并另存
            robotPoses = readmatrix(posePath); % 读取最新的Pose Estimates Table 
            % [robotPoses, srcGroundTruth] = convertCoordinates(robotPoses, srcGroundTruth, origin, resolution);
            azEstTable = deg2rad(readmatrix(azimuthPath)); % 读取最新的DoA Estimates Table 
            elEstTable = deg2rad(readmatrix(elevationPath)); 
            numTimeSteps = last_length;  % number of discrete time steps, K
            robotPoses(1:numTimeSteps,1) = (robotPoses(1:numTimeSteps,1)-origin(1))/resolution;
            robotPoses(1:numTimeSteps,2) = (robotPoses(1:numTimeSteps,2)-origin(2))/resolution;
            robotPoses(1:numTimeSteps,3) = (robotPoses(1:numTimeSteps,3)-origin(3))/resolution;
            
            % Initialize the updated DoA Estimates Table 
            updatedAzEstTable = azEstTable; % Initialize the updated DoA Estimates Table -azi
            updatedElEstTable = elEstTable; % Initialize the updated DoA Estimates Table -ele

            % Initialize array of detected source particle filters
            detectedSourceFilters = [];

            % Filtering - Clustering - Implicit Associating cycle
            tic % Start timer
            roundCount = 0; % iteration round count
            temp_N = 0;  

            while temp_N < numObservations
                temp_N = temp_N+1;  % N'
                run("cycle.m")
            end

            % Merge similar estimates
            merge_tresh = 0.5/resolution; % 0.5m
            detectedSourceFilters = mergeClusters_3D(detectedSourceFilters, merge_tresh);
            I = numel(detectedSourceFilters); % number of detected sources 

            elapsedTime = toc; % End timer
            disp(['The mapping took ', num2str(elapsedTime), ' seconds.']);

            % Visualize ASM results
            if ~exist('hFig2D', 'var') || ~isvalid(hFig2D)
                hFig2D = figure;
            else
                figure(hFig2D);
                cla; 
            end  
            imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)], 'InitialMagnification', 200);
            set(gca, 'YDir', 'normal');
            set(hFig2D, 'MenuBar', 'none'); 
            set(hFig2D, 'Position', [-10.2, 45.8, 673.6, 518.4]);
            hold on;
            legendEntries = gobjects(2, 1);
            for i = 1:I 
                particleFilter = detectedSourceFilters{i}; 
                plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.', 'MarkerSize', 10);
                scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');     
            end
            hGroundTruth = scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
            scatter(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 10, 'k', 'filled'); 
            plot(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 'k--'); 
            
            % plot N DoA observations of current time step k
            for i = 1:numObservations 
                azi = wrapToPi(azEstTable(numTimeSteps,i) + deg2rad(robotPoses(numTimeSteps,4)));
                quiver(robotPoses(numTimeSteps,1), robotPoses(numTimeSteps,2), cos(azi), sin(azi), 10); 
            end
            title("2D Mapping Results, k = " + num2str(numTimeSteps));
            legend(hGroundTruth, 'Source ground truth');
            axis equal;
            axis on;
            axis image;
            pause(0.1);
            
            if ~exist('hFig3D', 'var') || ~isvalid(hFig3D)
                hFig3D = figure;
                set(hFig3D, 'MenuBar', 'none'); 
                set(hFig3D, 'Position', [-1.4, 593.8, 663.2, 250]);
            else
                figure(hFig3D);
                clf; 
            end
            hold on;
            for i = 1:I
                particleFilter = detectedSourceFilters{i};
                % Plot source estimate (State)
                scatter3(particleFilter.State(:,1), particleFilter.State(:,2), particleFilter.State(:,3), 50, 'b^', 'filled');     
            end
            scatter3(srcGroundTruth(:,1), srcGroundTruth(:,2), srcGroundTruth(:,3), 100, 'rp', 'filled'); 
            scatter3(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), robotPoses(1:numTimeSteps,3), 10, 'k', 'filled'); 
            plot3(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), robotPoses(1:numTimeSteps,3), 'k--'); 
            for i = 1:numObservations 
                angle = wrapToPi(azEstTable(numTimeSteps,i) + deg2rad(robotPoses(numTimeSteps,4)));
                azi = wrapToPi(azEstTable(numTimeSteps,i) + deg2rad(robotPoses(numTimeSteps,4)));
                ele = wrapToPi(elEstTable(numTimeSteps,i));
                quiver3(robotPoses(numTimeSteps,1), robotPoses(numTimeSteps,2), robotPoses(numTimeSteps,3), cos(ele)*cos(azi), cos(ele)*sin(azi), sin(ele), 10); % plot N DoA observations of current time step k
            end
            title("3D Mapping Results, k = " + num2str(numTimeSteps));
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            xlim([60, 260]);
            zlim([-10, 20]);
            axis equal;
            grid on;
            view(30, 15);
            pause(0.1);
        end          
end








