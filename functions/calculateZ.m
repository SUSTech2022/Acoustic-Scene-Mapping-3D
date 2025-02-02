
function zEstList = calculateZ(associatedParticleSet,robotPoses)

    numAssoObservations = numel(associatedParticleSet{1}.timeStep);
    timeStep = associatedParticleSet{1}.timeStep;
    % azEst = associatedParticleSet{1}.azEst;
    elEst = associatedParticleSet{1}.elEst;
    
    xEst = associatedParticleSet{1}.State(1);
    yEst = associatedParticleSet{1}.State(2);
    % zEstList = [];
    zEstList = zeros(numAssoObservations, 1); 

    % compute 2D Euclidean distance
    dx = xEst - robotPoses(timeStep, 1);
    dy = yEst - robotPoses(timeStep, 2);
    d_xy = sqrt(dx.^2 + dy.^2);   

    % compute z coordinate
    zEstList = d_xy .* tan(elEst) + robotPoses(timeStep, 3);
    
end