
function convertQuaternion2Theta(arrangement)
    % convert_quaternion_to_theta - Converts quaternion data to theta (Euler angle)
    %
    % Syntax: convert_quaternion_to_theta(arrangement)
    %
    % Inputs:
    %   arrangement - A string representing the arrangement identifier (e.g., '1125c')
    %
    % Outputs:
    %   None (Results are written to an output file)

    data = readmatrix(sprintf(".\exp_data\arrangement_%s\pose\pose_theta_ori.xlsx", arrangement));
    theta = zeros(size(data, 1), 1);

    for i = 1:size(data, 1)
        % qw qx qy qz
        quat = [data(i, 7), data(i, 4:6)];

        % convert to eular angles
        eul = quat2eul(quat, 'ZYX');
 
        theta(i) = rad2deg(eul(1));
    end

    new_data = [data(:, 1:2), theta];
    writematrix(new_data, sprintf(".\exp_data\arrangement_%s\pose\pose_theta.xlsx", arrangement));
end







