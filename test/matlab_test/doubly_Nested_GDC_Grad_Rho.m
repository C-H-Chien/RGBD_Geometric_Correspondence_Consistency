function [Times, Final_R, Final_T] = doubly_Nested_GDC_Grad_Rho ...
                 (PARAMS, X, Y, depthMap1, depthMap2, ...
                  R12, T12, K, invK, fx, fy, ...
                  match_points3D_v1, match_points2D_v1, match_points3D_v2, match_points2D_v2, ...
                  all_Points3D_View1, all_Points3D_View2)

    [rows, cols] = size(depthMap2);
    Times.orientation_err_deg_when_success = 0;
    Times.transl_err_when_success = 0;
    Times.success_flag = 0;
    
    %> Small patch meshgrid for each point
    window_half_sz = PARAMS.WINDOW_HALF_SIZE;
    window_length = 2*window_half_sz + 1;
    [xm, ym] = meshgrid(-window_half_sz:window_half_sz, -window_half_sz:window_half_sz);
    
    %> Get gradient of Depth Maps here
    gradDepth1_xi  = filter_2d(depthMap1, @Gx_2d_op, PARAMS.GAUSSIAN_SIGMA, 1);
    gradDepth1_eta = filter_2d(depthMap1, @Gy_2d_op, PARAMS.GAUSSIAN_SIGMA, 1);
    gradDepth2_xi  = filter_2d(depthMap2, @Gx_2d_op, PARAMS.GAUSSIAN_SIGMA, 1);
    gradDepth2_eta = filter_2d(depthMap2, @Gy_2d_op, PARAMS.GAUSSIAN_SIGMA, 1);
    
    %> Get the gradient of Phi in the pixel space and also at (x0, y0),
    %  i.e., the points in image 2. I am doing it for all points at once.
    grad_Rho2_xi  = interp2(X, Y, gradDepth2_xi,  match_points2D_v2(:,1), match_points2D_v2(:,2));
    grad_Rho2_eta = interp2(X, Y, gradDepth2_eta, match_points2D_v2(:,1), match_points2D_v2(:,2));
    grad_Rho1_xi  = interp2(X, Y, gradDepth1_xi,  match_points2D_v1(:,1), match_points2D_v1(:,2));
    grad_Rho1_eta = interp2(X, Y, gradDepth1_eta, match_points2D_v1(:,1), match_points2D_v1(:,2));
    
    %> Get all gammas and rhos
    gammas1  = invK * [match_points2D_v1, ones(size(match_points2D_v1, 1), 1)]';
    gammas2  = invK * [match_points2D_v2, ones(size(match_points2D_v2, 1), 1)]';
    rho1  = match_points3D_v1(:,3);
    rho2  = match_points3D_v2(:,3);
    max_inliers = 0;
    tic;
    for iter1 = 1:PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS

        id1 = randi(min(PARAMS.TOP_N_FOR_FIRST_PAIR,size(match_points3D_v2,1)));
        anchor_Indx = id1;
        picked_Indx = zeros(2,1);

        id2 = randi(min(PARAMS.TOP_N_FOR_SECOND_PAIR,size(match_points3D_v2,1)));
        while id1==id2
            id2 = randi(min(PARAMS.TOP_N_FOR_SECOND_PAIR,size(match_points3D_v2,1)));
        end

        id3 = randi(min(PARAMS.TOP_N_FOR_THIRD_PAIR,size(match_points3D_v2,1)));
        while id1==id3 || id2==id3
            id3 = randi(min(PARAMS.TOP_N_FOR_THIRD_PAIR,size(match_points3D_v2,1)));
        end

        picked_Indx(1) = id2;
        picked_Indx(2) = id3;

        %> From image 1 to image 2
        dist_1to2 = GDC_Grad_Rho_One_Point(fx, fy, xm, ym, rows, cols, ...
                          rho2, gammas2, match_points2D_v2, match_points3D_v2, match_points3D_v1, ...
                          all_Points3D_View2, window_length, anchor_Indx, picked_Indx(1), ...
                          grad_Rho2_xi, grad_Rho2_eta);

        %> From image 2 to image 1
        dist_2to1 = GDC_Grad_Rho_One_Point(fx, fy, xm, ym, rows, cols, ...
                          rho1, gammas1, match_points2D_v1, match_points3D_v1, match_points3D_v2, ...
                          all_Points3D_View1, window_length, anchor_Indx, picked_Indx(1), ...
                          grad_Rho1_xi, grad_Rho1_eta);

        if (dist_1to2 < 1) && (dist_2to1 < 1)
            %> From image 1 to image 2
            dist_1to2_ = GDC_Grad_Rho_One_Point(fx, fy, xm, ym, rows, cols, ...
                              rho2, gammas2, match_points2D_v2, match_points3D_v2, match_points3D_v1, ...
                              all_Points3D_View2, window_length, anchor_Indx, picked_Indx(2), ...
                              grad_Rho2_xi, grad_Rho2_eta);

            %> From image 2 to image 1
            dist_2to1_ = GDC_Grad_Rho_One_Point(fx, fy, xm, ym, rows, cols, ...
                              rho1, gammas1, match_points2D_v1, match_points3D_v1, match_points3D_v2, ...
                              all_Points3D_View1, window_length, anchor_Indx, picked_Indx(2), ...
                              grad_Rho1_xi, grad_Rho1_eta);


            if (dist_1to2_ < 1) && (dist_2to1_ < 1)

                %> Estimate the pose
                xyz1  = match_points3D_v1(anchor_Indx, :);
                xyz1_ = match_points3D_v2(anchor_Indx, :);
                xyz2  = match_points3D_v1(picked_Indx(1), :);
                xyz2_ = match_points3D_v2(picked_Indx(1), :);
                xyz3  = match_points3D_v1(picked_Indx(2), :);
                xyz3_ = match_points3D_v2(picked_Indx(2), :);
                points3D_set1 = [xyz1; xyz2; xyz3];
                points3D_set2 = [xyz1_; xyz2_; xyz3_];
                [R, T] = align_point_clouds(points3D_set2, points3D_set1);

                %> Angular error for rotation matrix
                orientation_err_rad = abs(acos((trace(R'*R12)-1)/2));
                orientation_err_deg = rad2deg(orientation_err_rad);

                %> Translation error
                transl_err = norm(T - T12);
                [inlier_count_alignment, ~] = compute_alignment_rmse( match_points3D_v1,  match_points3D_v2, R, T);
                
                [inlier_count_sampson, ~] = count_inliers_from_Sampson_error ...
                (PARAMS, match_points3D_v1', match_points2D_v2', R, T, K);
                
                inlier_count = inlier_count_sampson;
                %inlier_count = inlier_count_sampson;
                if inlier_count > max_inliers
                    max_inliers = inlier_count;
                    Times.transl_err_when_success = transl_err;
                    Times.orientation_err_deg_when_success = orientation_err_deg;
                    Times.success_flag = 1;
                    Final_R = R;
                    Final_T = T;
                end
            end
        end
    end
    toc;
    disp(toc - tic);
end

function [inlier_count,rmse_error] = compute_alignment_rmse(point_cloud2, point_cloud1, R, t)
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

inlier_count=sum(distances<0.01);
% Compute the root-mean-square error between the two point clouds
rmse_error = sqrt(mean(distances.^2));

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

function [inlier_count, inlier_indices] = count_inliers_from_Sampson_error(PARAMS, point3D_v1, point2D_v2, R, T, K)
    
    transformed_xyz1 = K * (R * point3D_v1 + T);
    projected_pt = transformed_xyz1 ./ transformed_xyz1(3,:);
    reproj_errors = point2D_v2 - projected_pt(1:2, :);
    reproj_errors_norm = vecnorm(reproj_errors, 2);

    %> Find indices where the reprojection error is less than some threshold
    indx_Pass_Reproj_Err_Thresh = find(reproj_errors_norm <= PARAMS.REPROJECTION_ERR_PIXEL_THRESH);
    inlier_indices = indx_Pass_Reproj_Err_Thresh;
    inlier_count = size(indx_Pass_Reproj_Err_Thresh, 2);
end
