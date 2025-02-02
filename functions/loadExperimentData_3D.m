function params = loadExperimentData_3D(arrangement,  SSLmethod)
    % Load experiment data based on specified arrangement (e.g., 'I' or 'II')
    % and SSL method (e.g., 'GCC-PHAT' or 'MVDR')
    % Input:
    %   arrangement - character string ('I' or 'II') specifying the source arrangement
    %    SSLmethod - character string ('GCC-PHAT' or 'MVDR') specifying the SSL method
    % Output:
    %   params - structure containing all required parameters for the experiment

    baseDir = ".\exp_data\"; 
    posePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "pose", "pose_theta.xlsx");
    azimuthPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "DoA", sprintf("online_azEst_%s.xlsx",  SSLmethod));
    elevationPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "DoA", sprintf("online_elEst_%s.xlsx", SSLmethod));
    imagePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "map", sprintf("map_%s.pgm", arrangement));

    params.robotPoses = readmatrix(posePath);                % Pose Estimates Table
    params.azEstTable = deg2rad(readmatrix(azimuthPath));  % DoA Estimates Table - azi
    params.elEstTable = deg2rad(readmatrix(elevationPath));  % DoA Estimates Table - ele

    image = imread(imagePath);                 
    params.image = flipud(image(1:250, 1:300));              % Occupancy map
    params.resolution = 0.05;                                % Occupancy map resolution
    params.origin = [-4.000000, -5.000000, 0.000000];        % Occupancy map origin
    params.sigma = deg2rad(5);                               % Standard deviation of delta theta
    params.associatedRange = 3 * params.sigma;               % Observation associated range
    

    % World coordinate system
    if arrangement == "I"
        params.srcGroundTruth = [ 0, -1.2, 0;
                                2.5, -1.2, 0;
                                5.0, -1.2, 0;
                                7.5,    0, 0;
                                7.5,  3.6, 0;
                                5.0,  4.8, 0;
                                2.5,  4.8, 0;
                                  0,  4.8, 0;
                              -1.25,  3.6, 0;
                              -1.25,  1.2, 0];
        params.numObservations = 3; % defualt:3, Number of considered DoA observations per time step
    elseif arrangement == "II"
        params.srcGroundTruth = [1.25,  0.6, 0;
                                 1.25, -0.6, 0;
                                 2.50,  0.6, 0;
                                 2.50, -0.6, 0;
                                 3.74,  0.6, 0;
                                 3.74, -0.6, 0;
                                 4.99,  0.6, 0;
                                 4.99, -0.6, 0;
                                 6.23,  0.6, 0;
                                 6.23, -0.6, 0];
        params.numObservations = 4;  % defualt:4
     elseif arrangement == "III"
        params.srcGroundTruth = [0.54,1.09,0.775;
                                 2.34,1.09,0.775;
                                 4.14,1.09,0.775;
                                 5.94,1.09,0.775;
                                    0,-1.2,    0;
                                 1.25,-1.2,    0;
                                  2.5,-1.2,    0;
                                 3.75,-1.2,    0;
                                    5,-1.2,    0;
                                 6.25,-1.2,    0];
        params.numObservations = 3; % defualt:3
    elseif arrangement == "IV" 
        params.srcGroundTruth = [1.25, 0.42, 0.68;
                                 2.50, 0.60, 0.00;
                                 3.75, 0.38, 0.67;
                                 5.00, 0.60, 0.00;
                                 6.24, 0.60, 0.00;
                                 6.24,-1.20, 0.00;
                                 5.00,-1.16, 0.68;
                                 3.75,-1.20, 0.00;
                                 2.50,-1.20, 0.68;
                                 1.25,-1.20, 0.00];
        params.numObservations = 4; % defualt:4
    elseif arrangement == "V" 
        params.srcGroundTruth = [1.25, -1.20,  0.00;
                                 2.50,  0.44,  0.69; 
                                 5.00, -1.20,  0.00; 
                                -2.21,  3.14,  0.00;
                                 8.67,  1.20,  0.00; 
                                 6.25,  3.24,  0.77; 
                                 5.00,  4.80,  0.00; 
                                 2.50,  3.85,  0.68;
                                 0.00,  4.80,  0.00;
                                 0.00,  1.05,  0.77];
        params.numObservations = 3; % defualt:3 
    elseif arrangement == "VI" 
        params.srcGroundTruth = [1.25, -1.20, 0.00;
                                 2.50,  0.44, 0.68;
                                 5.00, -1.20, 0.00;
                                 6.25,  1.20, 0.00;
                                 7.50,  1.20, 0.00; 
                                 7.50, -1.20, 0.00;
                                 2.50, -1.22, 0.68; 
                                 6.25, -1.23, 0.68;
                                 3.75,  0.60, 0.00;
                                 0.00,  1.04, 0.77];
        params.numObservations = 4; % defualt:4
    else
        error('Invalid arrangement specified.');
    end
end
