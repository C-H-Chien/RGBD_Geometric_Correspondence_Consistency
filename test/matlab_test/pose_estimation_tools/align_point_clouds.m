function [R, t] = align_point_clouds(points1, points2)
% Aligns two point clouds using feature matches
%
% Inputs:
%   point_cloud1: Nx3 matrix of 3D points in the first point cloud
%   point_cloud2: Mx3 matrix of 3D points in the second point cloud
%   matches: Px2 matrix of indices representing feature matches between the two point clouds
%
% Outputs:
%   R: 3x3 rotation matrix to align point_cloud2 with point_cloud1
%   t: 3x1 translation vector to align point_cloud2 with point_cloud1

% Extract the matching points from the two point clouds
%points1 = point_cloud1(matches(:, 1), :);
%points2 = point_cloud2(matches(:, 2), :);

% Compute the centroids of the matching points in each point cloud
centroid1 = mean(points1);
centroid2 = mean(points2);

% -0.128738158692995   0.188985781154671   1.120163380809315
%    0.406403095034157   0.365184359243770   0.890583764648437
%   -0.520822980142110   0.054673591768922   1.326050474628387

% 0.321155165878628   0.331102898915362   0.780360092319548
%    0.085091192248212  -0.175343654707164   1.203200000000000
%    0.433489502281350   0.169996750038386   0.933266178009287

% Compute the covariance matrix of the matching points in each point cloud
covariance_matrix = (points2 - centroid2)' * (points1 - centroid1);

% Use singular value decomposition to compute the optimal rotation matrix
[U, ~, V] = svd(covariance_matrix);
R = V * U';

% Ensure that R is a valid rotation matrix (det(R) = 1)
if det(R) < 0
    V(:, 3) = -V(:, 3);
    R = V * U';
end

% Compute the translation vector
t = centroid1' - R * centroid2';


%rmse_error = compute_alignment_rmse(points1, points2, R, t);
end

function transformed_points = transform_point_cloud(points, R, t)
% Transforms a point cloud using a rotation matrix and translation vector
%
% Inputs:
%   points: Nx3 matrix of 3D points to be transformed
%   R: 3x3 rotation matrix
%   t: 3x1 translation vector
%
% Outputs:
%   transformed_points: Nx3 matrix of transformed 3D points

% Apply the rotation matrix and translation vector to the points
transformed_points = (R * points')' + t';

end


function rmse_error = compute_alignment_rmse(point_cloud1, point_cloud2, R, t)
% Computes the root-mean-square error (RMSE) between two point clouds after alignment
%
% Inputs:
%   point_cloud1: Nx3 matrix of 3D points in the first point cloud
%   point_cloud2: Mx3 matrix of 3D points in the second point cloud
%   R: 3x3 rotation matrix to align point_cloud2 with point_cloud1
%   t: 3x1 translation vector to align point_cloud2 with point_cloud1
%
% Outputs:
%   rmse_error: scalar value of the RMSE between the two aligned point clouds

% Transform point_cloud2 to align with point_cloud1
transformed_point_cloud2 = transform_point_cloud(point_cloud2, R, t);
size(transformed_point_cloud2);

% Compute the distances between each corresponding point in the two point clouds
distances = sqrt(sum((point_cloud1 - transformed_point_cloud2).^2, 2));

% Compute the root-mean-square error between the two point clouds
rmse_error = sqrt(mean(distances.^2));

end
