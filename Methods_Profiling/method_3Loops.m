function Times = method_3Loops( PARAMS, K, R12, T12, ...
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
    Times.profile_pass_geometric_nums_123 = 0;
    Times.Min_Iterations1 = PARAMS.NUM_OF_FIRST_ITERATIONS;
    Times.Min_Iterations2 = PARAMS.NUM_OF_SECOND_ITERATIONS;
    Times.Min_Iterations3 = PARAMS.NUM_OF_THIRD_ITERATIONS;
    
    Times.orientation_err_deg_when_success = 0;
    Times.transl_err_when_success = 0;
    Times.max_inlier_ratio_for_accurate_pose_when_success = 0;
    Times.success_flag = 0;
    
    flag = 1;
    max_count_sampson = 0;
    
    if PARAMS.DO_TIME_PROFILING
        profiling_iterations = 1000; 
        num_of_1st_loop = PARAMS.NUM_OF_FIRST_ITERATIONS;
        num_of_2nd_loop = PARAMS.NUM_OF_SECOND_ITERATIONS;
        num_of_3rd_loop = PARAMS.NUM_OF_THIRD_ITERATIONS;
    else
        profiling_iterations = 1;
        num_of_1st_loop = 25;
        num_of_2nd_loop = 25;
        num_of_3rd_loop = 25;
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
            end
            time_ = toc;
            Times.profile_time2 = Times.profile_time2 + (time_/profiling_iterations);
            
%             %> Profiling 3: geometric constraint for 1st + 2nd pairs
%             tic;
%             for ti = 1:profiling_iterations
%                 distance1  = norm(xyz1  - xyz2);
%                 distance1_ = norm(xyz1_ - xyz2_);
%                 err_geom_12 = abs(distance1-distance1_);
%             end
%             time_ = toc;
%             Times.profile_time3 = Times.profile_time3 + (time_/profiling_iterations);
%             
%             %> Count # of passes in geometric constraint for 1st+2nd pairs
%             if err_geom_12 < PARAMS.GEOMETRY_ERR_THRESH
%                 Times.profile_pass_geometric_nums_12 = Times.profile_pass_geometric_nums_12 + 1;
%             end
 
%             %> Count # of passes in photometric constraint for the 2nd pair
%             if err_phto_2 < PARAMS.PHOTOMETRIC_ERR_THRESH
%                 Times.profile_pass_photometric_nums_2 = Times.profile_pass_photometric_nums_2 + 1;
%             end
            
            pass_constraint_flag = 0;
            if PARAMS.APPLY_CONSTRAINTS == 1
                if err_geom_12 < PARAMS.GEOMETRY_ERR_THRESH
                    pass_constraint_flag = 1;
                end
            else
                %> If not applyong any constraint, the constraint flag is always 1
                pass_constraint_flag = 1;
            end

            if pass_constraint_flag == 1
                
                %Times.profile_pass_combined_nums_2nd_loop = Times.profile_pass_combined_nums_2nd_loop + 1;
                
                for iter_Loop3 = 1:num_of_3rd_loop
                    
                    %> Profiling 5: pick and get the third pair
                    tic;
                    for ti = 1:profiling_iterations
                        id3 = randi(PARAMS.TOP_N_FOR_THIRD_PAIR);

                        while id1==id3 || id2==id3
                            id3 = randi(PARAMS.TOP_N_FOR_THIRD_PAIR);
                        end

                        xyz3    = all_points3D_v1(id3, :);
                        xyz3_   = all_points3D_v2(id3, :);
                        pt3_2d  = all_points2D_v1(id3, :);
                        pt3_2d_ = all_points2D_v2(id3, :);
                    end
                    time_ = toc;
                    Times.profile_time5 = Times.profile_time5 + (time_/profiling_iterations);
                   
%                     %> Profiling 6: geometric constraint for 1st+3rd and 2nd+3rd pairs
%                     tic;
%                     for ti = 1:profiling_iterations
%                         distance2  = norm(xyz3  - xyz1);
%                         distance2_ = norm(xyz3_ - xyz1_);
%                         distance3  = norm(xyz3  - xyz2);
%                         distance3_ = norm(xyz3_ - xyz2_);
%                         err_geom_13 = abs(distance2-distance2_);
%                         err_geom_23 = abs(distance3-distance3_);
%                     end
%                     time_ = toc;
%                     Times.profile_time6 = Times.profile_time6 + (time_/profiling_iterations);
%                 
%                     %> Count # of passes in geometric constraint for 1st+3rd  and 2nd+3rd pairs
%                     if err_geom_13 < PARAMS.GEOMETRY_ERR_THRESH && err_geom_23 < PARAMS.GEOMETRY_ERR_THRESH
%                         Times.profile_pass_geometric_nums_123 = Times.profile_pass_geometric_nums_123 + 1;
%                     end
  
                    pass_constraint_flag_inner = 0;
                    if PARAMS.APPLY_CONSTRAINTS == 1
                        if err_geom_13 < PARAMS.GEOMETRY_ERR_THRESH && ...
                           err_geom_23 < PARAMS.GEOMETRY_ERR_THRESH
                            pass_constraint_flag_inner = 1;
                        end
                    else
                        %> If not applyong any constraint, the constraint flag is always 1
                        pass_constraint_flag_inner = 1;
                    end

                    if pass_constraint_flag_inner == 1
                        
                        points3D_set1 = [xyz1; xyz2; xyz3];
                        points3D_set2 = [xyz1_; xyz2_; xyz3_];
                        [R_, t_] = align_point_clouds(points3D_set2, points3D_set1);

                        [inlier_count, ~] = compute_alignment_rmse(all_points3D_v1, all_points3D_v2, R_, t_);

                        inlier_count_sampson = count_inliers_from_Sampson_error(PARAMS, all_points3D_v1', all_points2D_v2', R_, t_, K);

                        %> 1) If pose estimation is very accurate, what is the max inlier ratio
                        R = R_;
                        T = t_;
                        %> Angular error for rotation matrix
                        orientation_err_rad = abs(acos((trace(R'*R12)-1)/2));
                        orientation_err_deg = rad2deg(orientation_err_rad);

                        %> Translation error
                        transl_err = norm(T - T12);

                        if flag == 1 && transl_err < PARAMS.TRANSLATION_DIST_TOL && ...
                           orientation_err_deg < PARAMS.ORIENTATION_DEG_TOL
                            Times.Min_Iterations1 = iter_Loop1;
                            Times.Min_Iterations2 = iter_Loop2;
                            Times.Min_Iterations3 = iter_Loop3;
                            Times.orientation_err_deg_when_success = orientation_err_deg;
                            Times.transl_err_when_success = transl_err;
                            Times.max_inlier_ratio_for_accurate_pose_when_success = inlier_count / size(all_points3D_v1, 1);
                            Times.success_flag = 1;
                            flag = 0;
                        end

                        %> 2) For max inlier ratio, how accurate is the pose estimation
                        %> This statement will always be entered...
                        if max_count < inlier_count
                            max_count = inlier_count;
                            Times.orientation_err_deg = orientation_err_deg;
                            Times.transl_err = transl_err;
                            Times.max_inlier_ratio_for_accurate_pose = max_count / size(all_points3D_v1, 1);
                        end

                        if max_count_sampson < inlier_count_sampson
                            max_count_sampson = inlier_count_sampson;
                            Times.orientation_err_deg_from_sampson = orientation_err_deg;
                            Times.transl_err_from_sampson = transl_err;
                            Times.max_inlier_ratio_for_accurate_pose_from_sampson = max_count_sampson / size(all_points3D_v1, 1);
                        end
                    end

                end
            end
        end
    end 

    Times.success = 1 - flag;
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

function inlier_count = count_inliers_from_Sampson_error(PARAMS, point3D_v1, point2D_v2, R, T, K)
    
    transformed_xyz1 = K * (R * point3D_v1 + T);
    projected_pt = transformed_xyz1 ./ transformed_xyz1(3,:);
    reproj_errors = point2D_v2 - projected_pt(1:2, :);
    reproj_errors_norm = vecnorm(reproj_errors, 2);

    %> Find indices where the reprojection error is less than some threshold
    indx_Pass_Reproj_Err_Thresh = find(reproj_errors_norm <= PARAMS.REPROJECTION_ERR_PIXEL_THRESH);
    inlier_count = size(indx_Pass_Reproj_Err_Thresh, 2);
end
