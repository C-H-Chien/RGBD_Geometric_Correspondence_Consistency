function Times = est_rel_pose_3Loops( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
                                      all_points2D_v1, all_points2D_v2, ...
                                      all_points3D_v1, all_points3D_v2)

    
    max_count = 0;
    Times.profile_time1 = 0;
    Times.profile_time2 = 0;
    Times.profile_time3 = 0;
    Times.profile_time4 = 0;
    Times.profile_time5 = 0;
    Times.profile_time6 = 0;
    Times.profile_time7 = 0;
    Times.profile_time8 = 0;
    Times.profile_time9 = 0;
    Times.profile_pass_geometric_nums_12 = 0;
    Times.profile_pass_photometric_nums_2 = 0;
    Times.profile_pass_geometric_nums_123 = 0;
    Times.profile_pass_photometric_nums_3 = 0;
    Times.profile_pass_combined_nums_2nd_loop = 0;
    Times.profile_pass_combined_nums_3rd_loop = 0;
    Times.Min_Iterations1 = PARAMS.NUM_OF_FIRST_ITERATIONS;
    Times.Min_Iterations2 = PARAMS.NUM_OF_SECOND_ITERATIONS;
    Times.Min_Iterations3 = PARAMS.NUM_OF_THIRD_ITERATIONS;
    
    flag = 1;
    
    if PARAMS.DO_TIME_PROFILING
        profiling_iterations = 1000; 
        num_of_1st_loop = PARAMS.NUM_OF_FIRST_ITERATIONS;
        num_of_2nd_loop = PARAMS.NUM_OF_SECOND_ITERATIONS;
        num_of_3rd_loop = PARAMS.NUM_OF_THIRD_ITERATIONS;
    else
        profiling_iterations = 1;
        num_of_1st_loop = 50;
        num_of_2nd_loop = 50;
        num_of_3rd_loop = 100;
    end
    
    if PARAMS.TOP_N_FOR_THIRD_PAIR > size(all_points3D_v1, 1)
        num_of_matches = size(all_points3D_v1, 1);
    else
        num_of_matches = PARAMS.TOP_N_FOR_THIRD_PAIR;
    end
    
    %> 1ST LOOP
    for iter_Loop1 = 1:num_of_1st_loop
        
        %> Profiling 1: Access 3D data from the first point correspondence
        tic;
        for ti = 1:profiling_iterations
            id1 = randi(PARAMS.TOP_N_FOR_FIRST_PAIR);
            xyz1    = all_points3D_v1(id1, :);
            xyz1_   = all_points3D_v2(id1, :);
            pt1_2d  = all_points2D_v1(id1, :);
            pt1_2d_ = all_points2D_v2(id1, :);
        end
        time_ = toc;
        Times.profile_time1 = Times.profile_time1 + (time_/profiling_iterations);

        for iter_Loop2 = 1:num_of_2nd_loop
            
            %> Profiling 2: Randomly pick the second point other than the
            %               first pair and get its 3D + RGB data
            tic;
            for ti = 1:profiling_iterations
                id2 = randi(PARAMS.TOP_N_FOR_SECOND_PAIR);

                while id1==id2
                    id2 = randi(PARAMS.TOP_N_FOR_SECOND_PAIR);
                end
                
                xyz2  = all_points3D_v1(id2, :);
                xyz2_ = all_points3D_v2(id2, :);
                pt2_2d  = all_points2D_v1(id2, :);
                pt2_2d_ = all_points2D_v2(id2, :);
                rgb2  = all_pointsRGB_v1(id2, :);
                rgb2_ = all_pointsRGB_v2(id2, :);
            end
            time_ = toc;
            Times.profile_time2 = Times.profile_time2 + (time_/profiling_iterations);
            
            %> Profiling 3: geometric constraint for 1st + 2nd pairs
            tic;
            for ti = 1:profiling_iterations
                distance1  = norm(xyz1  - xyz2);
                distance1_ = norm(xyz1_ - xyz2_);
                err_geom_12 = abs(distance1-distance1_);
            end
            time_ = toc;
            Times.profile_time3 = Times.profile_time3 + (time_/profiling_iterations);
            
            %> Count # of passes in geometric constraint for 1st+2nd pairs
            if err_geom_12 < PARAMS.GEOMETRY_ERR_THRESH
                Times.profile_pass_geometric_nums_12 = Times.profile_pass_geometric_nums_12 + 1;
            end
            
            %> Profiling 4: photometric constraint for the 2nd pair
            tic;
            for ti = 1:profiling_iterations
                err_phto_2 = norm(rgb2(:) - rgb2_(:));
            end
            time_ = toc;
            Times.profile_time4 = Times.profile_time4 + (time_/profiling_iterations);
            
            %> Count # of passes in photometric constraint for the 2nd pair
            if err_phto_2 < PARAMS.PHOTOMETRIC_ERR_THRESH
                Times.profile_pass_photometric_nums_2 = Times.profile_pass_photometric_nums_2 + 1;
            end
            
            if err_geom_12 < PARAMS.GEOMETRY_ERR_THRESH %&& err_phto_2 < PARAMS.PHOTOMETRIC_ERR_THRESH 
                
                Times.profile_pass_combined_nums_2nd_loop = Times.profile_pass_combined_nums_2nd_loop + 1;
                
                for iter_Loop3 = 1:num_of_3rd_loop
                    
                    %> Profiling 5: pick and get the third pair
                    tic;
                    for ti = 1:profiling_iterations
                        id3 = randi(num_of_matches);

                        while id1==id3 || id2==id3
                            id3 = randi(num_of_matches);
                        end

                        xyz3    = all_points3D_v1(id3, :);
                        xyz3_   = all_points3D_v2(id3, :);
                        pt3_2d  = all_points2D_v1(id3, :);
                        pt3_2d_ = all_points2D_v2(id3, :);
                        rgb3    = all_pointsRGB_v1(id3, :);
                        rgb3_   = all_pointsRGB_v2(id3, :);
                    end
                    time_ = toc;
                    Times.profile_time5 = Times.profile_time5 + (time_/profiling_iterations);
                   
                    %> Profiling 6: geometric constraint for 1st+3rd and 2nd+3rd pairs
                    tic;
                    for ti = 1:profiling_iterations
                        distance2  = norm(xyz3  - xyz1);
                        distance2_ = norm(xyz3_ - xyz1_);
                        distance3  = norm(xyz3  - xyz2);
                        distance3_ = norm(xyz3_ - xyz2_);
                        err_geom_13 = abs(distance2-distance2_);
                        err_geom_23 = abs(distance3-distance3_);
                    end
                    time_ = toc;
                    Times.profile_time6 = Times.profile_time6 + (time_/profiling_iterations);
                
                    %> Count # of passes in geometric constraint for 1st+3rd  and 2nd+3rd pairs
                    if err_geom_13 < PARAMS.GEOMETRY_ERR_THRESH && err_geom_23 < PARAMS.GEOMETRY_ERR_THRESH
                        Times.profile_pass_geometric_nums_123 = Times.profile_pass_geometric_nums_123 + 1;
                    end
                    
                    %> Profiling 7: photometric constraint for the 3rd pair
                    tic;
                    for ti = 1:profiling_iterations
                        err_photo_3 = norm(rgb3(:) - rgb3_(:));
                    end
                    time_ = toc;
                    Times.profile_time7 = Times.profile_time7 + (time_/profiling_iterations);
                    
                    %> Count # of passes in photometric constraint for 3rd pair
                    if err_photo_3 < PARAMS.PHOTOMETRIC_ERR_THRESH
                        Times.profile_pass_photometric_nums_3 = Times.profile_pass_photometric_nums_3 + 1;
                    end
                
                    if err_geom_13 < PARAMS.GEOMETRY_ERR_THRESH && ...
                       err_geom_23 < PARAMS.GEOMETRY_ERR_THRESH %&& ...
                       %err_photo_3 < PARAMS.PHOTOMETRIC_ERR_THRESH
                        
                        Times.profile_pass_combined_nums_3rd_loop = Times.profile_pass_combined_nums_3rd_loop + 1;
                        
                        if flag == 1
                            points3D_set1 = [xyz1; xyz2; xyz3];
                            points3D_set2 = [xyz1_; xyz2_; xyz3_];
                            [R_, t_] = align_point_clouds(points3D_set2, points3D_set1);
                            %time_ = toc;
                            %Times.profile_time6 = Times.profile_time6 + time_;

                            %> Profiling 7: find final solution corresponds to maximal inliers
                            %> TODO: use Sampson error???
                            %tic;
                            [inlier_count, ~] = compute_alignment_rmse(all_points3D_v1, all_points3D_v2, R_, t_);
                            %if max_count <= inlier_count
                                R = R_;
                                T = t_;
                                max_count = inlier_count;

                                %> Angular error for rotation matrix
                                orientation_err_rad = abs(acos((trace(R'*R12)-1)/2));
                                orientation_err_deg = rad2deg(orientation_err_rad);

                                %> Translation error
                                transl_err = norm(T - T12);

                                if transl_err < PARAMS.TRANSLATION_DIST_TOL && ...
                                   orientation_err_deg < PARAMS.ORIENTATION_DEG_TOL
                                    Times.Min_Iterations1 = iter_Loop1;
                                    Times.Min_Iterations2 = iter_Loop2;
                                    Times.Min_Iterations3 = iter_Loop3;
                                    Times.orientation_err_deg = orientation_err_deg;
                                    Times.transl_err = transl_err;
                                    Times.success_flag = 1;
                                    flag = 0;
                                    break;
                                else
                                    Times.orientation_err_deg = orientation_err_deg;
                                    Times.transl_err = transl_err;
                                    Times.success_flag = 0;
                                end
                            %end
                        end
                    end

%                     %> Compute the reprojection errors to see how many loops we need to get very good 3 pairs
%                     transformed_xyz1 = K * (R12 * xyz1' + T12);
%                     projected_pt = transformed_xyz1 ./ transformed_xyz1(3,1);
%                     reprojection_error_pt1 = norm(pt1_2d_' - projected_pt(1:2, 1));
% 
%                     transformed_xyz2 = K * (R12 * xyz2' + T12);
%                     projected_pt = transformed_xyz2 ./ transformed_xyz2(3,1);
%                     reprojection_error_pt2 = norm(pt2_2d_' - projected_pt(1:2, 1));
% 
%                     transformed_xyz3 = K * (R12 * xyz3' + T12);
%                     projected_pt = transformed_xyz3 ./ transformed_xyz3(3,1);
%                     reprojection_error_pt3 = norm(pt3_2d_' - projected_pt(1:2, 1));
% 
%                     if flag == 1 && reprojection_error_pt1 < 1.5 && reprojection_error_pt2 < 1.5 && reprojection_error_pt3 < 1.5
%                         Times.Min_Iterations1 = ind;
%                         Times.Min_Iterations2 = iter1;
%                         Times.Min_Iterations2 = iter2;
%                         flag = 0;
%                     end
                end
            end
            
        if flag == 0
            break;
        end
            
        end
        
        if flag == 0
            break;
        end
    end 

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