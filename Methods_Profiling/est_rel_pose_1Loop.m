function Times = est_rel_pose_1Loop( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
                                       all_points2D_v1, all_points2D_v2, ...
                                       all_points3D_v1, all_points3D_v2)
    
    Times.profile_time1 = 0;
    Times.profile_time2 = 0;
    Times.profile_time3 = 0;
    Times.profile_time4 = 0;
    Times.profile_time5 = 0;
    Times.profile_time6 = 0;
    
    Times.profile_pass_geom_constraint_num = 0;
    Times.profile_pass_photo_constraint_num = 0;
    Times.profile_pass_combined_conostraint_num = 0;
    
    if PARAMS.TOP_RANK_ORDERED_LIST_SIZE > size(all_points3D_v1, 1)
        num_of_matches = size(all_points3D_v1, 1);
    else
        num_of_matches = PARAMS.TOP_RANK_ORDERED_LIST_SIZE;
    end

    max_count=0;
    for iter = 1:PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS

        %> Profiling 1: Pick 3 random points from the rank-ordered list and
        %  also access their data
        tic;
        for ti = 1:1000
            ind  = randperm(num_of_matches, 3);
            id1 = ind(1);
            id2 = ind(2);
            id3 = ind(3);
            
            xyz1  = all_points3D_v1(id1, :);
            xyz1_ = all_points3D_v2(id1, :);
            xyz2  = all_points3D_v1(id2, :);
            xyz2_ = all_points3D_v2(id2, :);
            xyz3  = all_points3D_v1(id3, :);
            xyz3_ = all_points3D_v2(id3, :);
            
            pt1_2d  = all_points2D_v1(id1, :);
            pt1_2d_ = all_points2D_v2(id1, :);
            pt2_2d  = all_points2D_v1(id2, :);
            pt2_2d_ = all_points2D_v2(id2, :);
            pt3_2d  = all_points2D_v1(id3, :);
            pt3_2d_ = all_points2D_v2(id3, :);
        end
        time_ = toc;
        Times.profile_time1 = Times.profile_time1 + (time_/1000);
        
        %> Profiling 2: Get data of the selected 3 pairs
        tic;
        for ti = 1:1000
            rgb1  = all_pointsRGB_v1(id1, :);
            rgb1_ = all_pointsRGB_v2(id1, :);            
            rgb2  = all_pointsRGB_v1(id2, :);
            rgb2_ = all_pointsRGB_v2(id2, :);
            rgb3  = all_pointsRGB_v1(id3, :);
            rgb3_ = all_pointsRGB_v2(id3, :);
        end
        time_ = toc;
        Times.profile_time2 = Times.profile_time2 + (time_/1000);
        %>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        %> Profiling 3: Compute geometric constraints
        tic;
        for ti = 1:1000
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
        Times.profile_time3 = Times.profile_time3 + (time_/1000);

        %> Profiling 4: Compute photometric constraints
        tic;
        for ti = 1:1000
            err1_photo = norm(rgb1(:) - rgb1_(:));
            err2_photo = norm(rgb2(:) - rgb2_(:));
            err3_photo = norm(rgb3(:) - rgb3_(:));
        end
        time_ = toc;
        Times.profile_time4 = Times.profile_time4 + (time_/1000);
        
        %>>> Measure the percentage of passing geometric constraint
        if err1_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
           err2_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
           err3_geom < PARAMS.GEOMETRY_ERR_THRESH
            Times.profile_pass_geom_constraint_num = Times.profile_pass_geom_constraint_num + 1;
        end
        
        %>>> Measure the percentage of passing photometric constraint
        if err1_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
           err2_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
           err3_photo < PARAMS.PHOTOMETRIC_ERR_THRESH
            Times.profile_pass_photo_constraint_num = Times.profile_pass_photo_constraint_num + 1;
        end
        
%         %> Compute the reprojection errors to see how many loops we need to get very good 3 pairs
%         transformed_xyz1 = K * (R12 * xyz1' + T12);
%         projected_pt = transformed_xyz1 ./ transformed_xyz1(3,1);
%         reprojection_error_pt1 = norm(pt1_2d_' - projected_pt(1:2, 1));
%         
%         transformed_xyz2 = K * (R12 * xyz2' + T12);
%         projected_pt = transformed_xyz2 ./ transformed_xyz2(3,1);
%         reprojection_error_pt2 = norm(pt2_2d_' - projected_pt(1:2, 1));
%         
%         transformed_xyz3 = K * (R12 * xyz3' + T12);
%         projected_pt = transformed_xyz3 ./ transformed_xyz3(3,1);
%         reprojection_error_pt3 = norm(pt3_2d_' - projected_pt(1:2, 1));
        
        if err1_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
           err2_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
           err3_geom < PARAMS.GEOMETRY_ERR_THRESH && ...
           err1_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
           err2_photo < PARAMS.PHOTOMETRIC_ERR_THRESH && ...
           err3_photo < PARAMS.PHOTOMETRIC_ERR_THRESH 

            Times.profile_pass_combined_conostraint_num = Times.profile_pass_combined_conostraint_num + 1;
        end
    end
end
