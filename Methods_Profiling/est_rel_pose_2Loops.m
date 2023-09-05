function Times = est_rel_pose_2Loops( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
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
    Times.profile_pass_geom_constraint_num = 0;
    Times.profile_pass_photo_constraint_num = 0;
    Times.profile_pass_combined_conostraint_num = 0;
    Times.profile_total_cost_per_picking_one_hypothesis = 0;
    
    Times.max_inlier_count_when_success = 0;
    Times.max_inlier_count = 0;
    
    flag = 1;
    
    if PARAMS.DO_TIME_PROFILING
        profiling_iterations = 1000; 
        num_of_1st_loop = PARAMS.NUM_OF_FIRST_ITERATIONS;
        num_of_2nd_loop = PARAMS.NUM_OF_SECOND_ITERATIONS*PARAMS.NUM_OF_THIRD_ITERATIONS;    
    else
        profiling_iterations = 1;
        num_of_1st_loop = 10;
        num_of_2nd_loop = 25;
    end
    
    Times.Min_Iterations1 = num_of_1st_loop;
    Times.Min_Iterations2 = num_of_2nd_loop;
    
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
            
            %> Profiling 2: Randomly pick 2 points other than the first
            %                point correspondence
            tic;
            for ti = 1:profiling_iterations
                id = randperm(num_of_matches,2);
                id2 = id(1);
                id3 = id(2);

                while id1==id2
                    id2 = randi(num_of_matches);
                end

                while id1==id3
                    id3 = randi(num_of_matches);
                end
            end
            time_ = toc;
            Times.profile_time2 = Times.profile_time2 + (time_/profiling_iterations);

            %> Proifiling 3: Access 3D data and RGB data for the 2 picked points
            tic;
            for ti = 1:profiling_iterations
                xyz2  = all_points3D_v1(id2, :);
                xyz2_ = all_points3D_v2(id2, :);
                xyz3  = all_points3D_v1(id3, :);
                xyz3_ = all_points3D_v2(id3, :);

                pt2_2d  = all_points2D_v1(id2, :);
                pt2_2d_ = all_points2D_v2(id2, :);
                pt3_2d  = all_points2D_v1(id3, :);
                pt3_2d_ = all_points2D_v2(id3, :);

                rgb2  = all_pointsRGB_v1(id2, :);
                rgb2_ = all_pointsRGB_v2(id2, :);
                rgb3  = all_pointsRGB_v1(id3, :);
                rgb3_ = all_pointsRGB_v2(id3, :);
            end
            time_ = toc;
            Times.profile_time3 = Times.profile_time3 + (time_/profiling_iterations);

            %> Profiling 4: Geometric constraint residuals
            tic;
            for ti = 1:profiling_iterations
                distance1  = norm(xyz1  - xyz2);
                distance1_ = norm(xyz1_ - xyz2_);
                distance2  = norm(xyz1  - xyz3);
                distance2_ = norm(xyz1_ - xyz3_);
                distance3  = norm(xyz2  - xyz3);
                distance3_ = norm(xyz2_ - xyz3_);
                err1_geom = abs(distance1-distance1_);
                err2_geom = abs(distance2-distance2_);
                err3_geom = abs(distance3-distance3_);
            end
            time_ = toc;
            Times.profile_time4 = Times.profile_time4 + (time_/profiling_iterations);
            
%             %> Profiling 5: Photometric constraint
%             tic;
%             for ti = 1:profiling_iterations
%                 err2_photo = norm(rgb2(:) - rgb2_(:));
%                 err3_photo = norm(rgb3(:) - rgb3_(:));
%             end
%             time_ = toc;
%             Times.profile_time5 = Times.profile_time5 + (time_/profiling_iterations);
            
            %> Count number of matches pass geometric constraint
            if err1_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
               err2_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
               err3_geom < PARAMS.GEOMETRY_ERR_THRESH
                Times.profile_pass_geom_constraint_num = Times.profile_pass_geom_constraint_num + 1;
            end
            
%             %> Count number of matches pass photometric constraint
%             if err2_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
%                err3_photo < PARAMS.PHOTOMETRIC_ERR_THRESH
%                 Times.profile_pass_photo_constraint_num = Times.profile_pass_photo_constraint_num + 1;
%             end
            
            pass_constraint_flag = 0;
            if PARAMS.APPLY_CONSTRAINTS == 1
                if err1_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
                   err2_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
                   err3_geom < PARAMS.GEOMETRY_ERR_THRESH %&& ...
    %                err2_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
    %                err3_photo < PARAMS.PHOTOMETRIC_ERR_THRESH
                    pass_constraint_flag = 1;
                end
            else
                %> If not applyong any constraint, the constraint flag is always 1
                pass_constraint_flag = 1;
            end

            if pass_constraint_flag == 1

                %> TODO: maybe use a regression model to estimate the
                %        relative camera pose?????????
                Times.profile_pass_combined_conostraint_num = Times.profile_pass_combined_conostraint_num + 1;

                points3D_set1 = [xyz1; xyz2; xyz3];
                points3D_set2 = [xyz1_; xyz2_; xyz3_];
                [R_, t_] = align_point_clouds(points3D_set2, points3D_set1);
                %time_ = toc;
                %Times.profile_time6 = Times.profile_time6 + time_;

                %> Profiling 7: find final solution corresponds to maximal inliers
                %> 
                %tic;
                [inlier_count, ~] = compute_alignment_rmse(all_points3D_v1, all_points3D_v2, R_, t_);
                
                %> Measure pose error w.r.t. ground truth
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
                    Times.orientation_err_deg = orientation_err_deg;
                    Times.transl_err = transl_err;
                    Times.max_inlier_count_when_success = max_count / size(all_points3D_v1, 1);
                    Times.success_flag = 1;
                    flag = 0;
                else
                    Times.orientation_err_deg = orientation_err_deg;
                    Times.transl_err = transl_err;
                    Times.success_flag = 0;
                end
                
                
                
                
                if max_count <= inlier_count
                    R = R_;
                    T = t_;
                    max_count = inlier_count;

                    %> Angular error for rotation matrix
                    orientation_err_rad = abs(acos((trace(R'*R12)-1)/2));
                    orientation_err_deg = rad2deg(orientation_err_rad);

                    %> Translation error
                    transl_err = norm(T - T12);
%                     if flag == 1
%                         if transl_err < PARAMS.TRANSLATION_DIST_TOL && ...
%                            orientation_err_deg < PARAMS.ORIENTATION_DEG_TOL
%                             Times.Min_Iterations1 = iter_Loop1;
%                             Times.Min_Iterations2 = iter_Loop2;
%                             Times.orientation_err_deg = orientation_err_deg;
%                             Times.transl_err = transl_err;
%                             Times.max_inlier_count_when_success = max_count / size(all_points3D_v1, 1);
%                             Times.success_flag = 1;
%                             flag = 0;
%                         else
%                             Times.orientation_err_deg = orientation_err_deg;
%                             Times.transl_err = transl_err;
%                             Times.max_inlier_count = max_count / size(all_points3D_v1, 1);
%                             Times.success_flag = 0;
%                         end
%                     end
                    Times.max_inlier_count = max_count / size(all_points3D_v1, 1);
                end
            end
             
%             %> Compute the reprojection errors to see how many loops we need to get very good 3 pairs
%             transformed_xyz1 = K * (R12 * xyz1' + T12);
%             projected_pt = transformed_xyz1 ./ transformed_xyz1(3,1);
%             reprojection_error_pt1 = norm(pt1_2d_' - projected_pt(1:2, 1));
% 
%             transformed_xyz2 = K * (R12 * xyz2' + T12);
%             projected_pt = transformed_xyz2 ./ transformed_xyz2(3,1);
%             reprojection_error_pt2 = norm(pt2_2d_' - projected_pt(1:2, 1));
% 
%             transformed_xyz3 = K * (R12 * xyz3' + T12);
%             projected_pt = transformed_xyz3 ./ transformed_xyz3(3,1);
%             reprojection_error_pt3 = norm(pt3_2d_' - projected_pt(1:2, 1));
            
%             if flag == 1 && reprojection_error_pt1 < 1.5 && reprojection_error_pt2 < 1.5 && reprojection_error_pt3 < 1.5
%                 Times.Min_Iterations1 = ind;
%                 Times.Min_Iterations2 = iter;
%                 flag = 0;
%                 %break;
%             end
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
    
    %> Maybe we should use

    % Compute the distances between each corresponding point in the two point clouds
    distances = sqrt(sum((point_cloud1 - transformed_point_cloud2).^2, 2));

    inlier_count=sum(distances<0.01);
    % Compute the root-mean-square error between the two point clouds
    rmse_error = sqrt(mean(distances.^2));
end
