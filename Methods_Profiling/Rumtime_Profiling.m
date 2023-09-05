rng(89)

start_frame = 6;
num_frames = 1864;
step_frame_size = 3;
%compute avg pose error and runtime for entire sequence
l = length(start_frame:step_frame_size:num_frames);
all_runtimes_rns=zeros(l,1);

all_runtimes_1loop = zeros(l,1);
all_runtimes_ours_2loops=zeros(l,1);
all_runtimes_ours_3loops=zeros(l,1);

all_runtime_profiling_1Loop = zeros(l,9);
all_runtime_profiling_2loops = zeros(l,11);
all_runtime_profiling_3loops = zeros(l,15);
all_runtime_profiling_classic = zeros(l,5);
all_min_iterations = zeros(l,1);
all_constraint_num = zeros(l,4);

all_min_iterations_2Loops = zeros(l, 2);
all_min_iterations_3Loops = zeros(l, 3);

pose_estimation_err_classic = zeros(l, 6);
pose_estimation_err_2Loops = zeros(l, 5);
pose_estimation_err_3Loops = zeros(l, 3);

% fx = 535.4;
% fy = 539.2;
% cx = 320.1;
% cy = 247.6;
% K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

%> Hyper-parameters
PARAMS.DO_TIME_PROFILING          = 0;
PARAMS.APPLY_CONSTRAINTS          = 0;

PARAMS.TOP_RANK_ORDERED_LIST_SIZE = 100;
PARAMS.TOP_N_FOR_FIRST_PAIR       = 100;     %> 10
PARAMS.TOP_N_FOR_SECOND_PAIR      = 100;     %> 25
PARAMS.TOP_N_FOR_THIRD_PAIR       = 100;    %> 100

PARAMS.PHOTOMETRIC_ERR_THRESH     = 0.2;
PARAMS.GEOMETRY_ERR_THRESH        = 0.2;

PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS = 2500;
PARAMS.NUM_OF_FIRST_ITERATIONS          = 5;
PARAMS.NUM_OF_SECOND_ITERATIONS         = 5;
PARAMS.NUM_OF_THIRD_ITERATIONS          = 5;

%> Criteria for a successful pose estimation
PARAMS.ORIENTATION_DEG_TOL              = 1.5;
PARAMS.TRANSLATION_DIST_TOL             = 0.05;

%> A pool of TUM-RGBD preprocessed datasets
sequence_Name = ["rgbd_dataset_freiburg1_desk/"; ...
                 "rgbd_dataset_freiburg2_desk/"; ...
                 ""; ...
                 "rgbd_dataset_freiburg3_long_office_household/"];


c = 1;
for di = 1:size(sequence_Name, 1)
    seq_Name = sequence_Name(di, 1);
    %> Choose which calibration matrix is based on the sequence category
    if contains(seq_Name, "freiburg1")
        fx = 517.3; fy = 516.5;
        cx = 318.6; cy = 239.5;
        K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    elseif contains(seq_Name, "freiburg2")
        fx = 520.9; fy = 521.0;
        cx = 325.1; cy = 249.7;
        K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    elseif contains(seq_Name, "freiburg3")
        fx = 535.4; fy = 539.2;
        cx = 320.1; cy = 247.6;
        K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    else
        fprintf("Invalid sequence name!\n");
        exit(1);
    end
    for i = start_frame:step_frame_size:num_frames
        fprintf(". ");
        if mod(i, 50) == 0, fprintf("\n"); end

        %> Load images, depths, and ground truth poses
        [rgbImage1, rgbImage2, depthMap1, depthMap2, R12, T12] = ...
            load_TUMRGBD_dataset_sequence(i-step_frame_size, i, K, seq_Name);
        depthMap1 = double(depthMap1)/5000;
        depthMap2 = double(depthMap2)/5000;   
        image1    = double(rgbImage1)./ 255;
        image2    = double(rgbImage2)./ 255;

        %> Detect keypoints and extract SURF features in both images
        points1 = detectSURFFeatures(rgb2gray(image1), 'MetricThreshold', 90);
        points2 = detectSURFFeatures(rgb2gray(image2), 'MetricThreshold', 90);
        [features1, validPoints1] = extractFeatures(rgb2gray(image1), points1);
        [features2, validPoints2] = extractFeatures(rgb2gray(image2), points2);

        %> Match features between the images
        [indexPairs,matchScores] = matchFeatures(features1, features2,'MaxRatio',0.7,'MatchThreshold',80);
        matchedPoints1 = validPoints1(indexPairs(:, 1), :);
        matchedPoints2 = validPoints2(indexPairs(:, 2), :);

        %> Make a top rank-ordered list of potential matches
        [sortedScores, sortedIndices] = sort(matchScores, 'ascend');
        matchedPoints1 = matchedPoints1(sortedIndices, :);
        matchedPoints2 = matchedPoints2(sortedIndices, :);

        %> Get 3D points 
        invalid_depth_indices = [];
        u1 = matchedPoints1.Location(:, 1);
        v1 = matchedPoints1.Location(:, 2);
        z1 = zeros(size(u1));
        for ii = 1 : size(u1, 1)
            z1(ii) = double(depthMap1(round(v1(ii)), round(u1(ii))));
            %> Record feature index with invalid depth
            if z1(ii) == 0, invalid_depth_indices = [invalid_depth_indices; ii]; end
        end
        x1 = (u1 - cx) .* z1 / fx;
        y1 = (v1 - cy) .* z1 / fy;
        all_points3D1 = [x1, y1, z1];

        u2 = matchedPoints2.Location(:, 1);
        v2 = matchedPoints2.Location(:, 2);
        z2 = zeros(size(u2));
        for ii = 1 : size(u2, 1)
            z2(ii) = double(depthMap2(round(v2(ii)), round(u2(ii))));
            if z2(ii) == 0, invalid_depth_indices = [invalid_depth_indices; ii]; end
        end
        x2 = (u2 - cx) .* z2 / fx;
        y2 = (v2 - cy) .* z2 / fy;
        all_points3D2 = [x2, y2, z2];

        %> Now we have all potential matches with valid depths for both 2D/3D
        all_points2D_v1 = matchedPoints1.Location;
        all_points2D_v2 = matchedPoints2.Location;
        all_points3D_v1 = all_points3D1;
        all_points3D_v2 = all_points3D2;
        all_points2D_v1(invalid_depth_indices, :) = [];
        all_points2D_v2(invalid_depth_indices, :) = [];
        all_points3D_v1(invalid_depth_indices, :) = [];
        all_points3D_v2(invalid_depth_indices, :) = [];

        %> Fetch RGB values for all potential matches from 2 views apriori
        all_pointsRGB_v1 = zeros(size(all_points2D_v1, 1), 3);
        all_pointsRGB_v2 = zeros(size(all_points2D_v1, 1), 3);
        for pi = 1:size(all_points2D_v1, 1)
            px = round(all_points2D_v1(pi, 1));
            py = round(all_points2D_v1(pi, 2));
            all_pointsRGB_v1(pi, :) = image1(py, px, :);
            px_ = round(all_points2D_v2(pi, 1));
            py_ = round(all_points2D_v2(pi, 2));
            all_pointsRGB_v2(pi, :) = image2(py_, px_, :);
        end

        %%%%%%%%%%% CLASSIC METHOD %%%%%%%%%%%
        Times_classic = est_rel_pose_classic( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
                                              all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
        all_runtime_profiling_classic(c, 1) = Times_classic.profile_time1 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
        all_runtime_profiling_classic(c, 2) = Times_classic.profile_time2 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
        all_runtime_profiling_classic(c, 3) = Times_classic.profile_time3 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
        all_runtime_profiling_classic(c, 4) = Times_classic.profile_pick_rand_permutation / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
        all_runtime_profiling_classic(c, 5) = Times_classic.profile_access_RGB_data / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
        
        all_min_iterations(c, 1) = Times_classic.Min_Iterations;
        all_constraint_num(c, 1) = Times_classic.profile_pass_geom_constraint_num;
        all_constraint_num(c, 2) = Times_classic.profile_pass_photo_constraint_num;
        all_constraint_num(c, 3) = Times_classic.profile_pass_combined_conostraint_num;
        all_constraint_num(c, 4) = Times_classic.profile_true_matches_num;
        
        %> When accurate pose estimation is obtained
        pose_estimation_err_classic(c, 1) = Times_classic.Min_Iterations;
        pose_estimation_err_classic(c, 2) = Times_classic.orientation_err_deg_when_success;
        pose_estimation_err_classic(c, 3) = Times_classic.transl_err_when_success;
        pose_estimation_err_classic(c, 4) = Times_classic.max_inlier_ratio_for_accurate_pose_when_success;
        pose_estimation_err_classic(c, 5) = Times_classic.success;
        
        %> For max inlier ratio...
        pose_estimation_err_classic(c, 6) = Times_classic.orientation_err_deg;
        pose_estimation_err_classic(c, 7) = Times_classic.transl_err;
        pose_estimation_err_classic(c, 8) = Times_classic.max_inlier_ratio_for_accurate_pose;
        
        
           
        time = Times_classic.profile_time1 + Times_classic.profile_time2 + ...
               Times_classic.profile_time3;
        
        all_runtimes_rns(c)=time;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %     %%%%%%%%%%% 1-LOOP METHOD %%%%%%%%%%%
    %     Times_1Loop = est_rel_pose_1Loop( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
    %                                       all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
    %     all_runtime_profiling_1Loop(c, 1) = Times_1Loop.profile_time1 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
    %     all_runtime_profiling_1Loop(c, 2) = Times_1Loop.profile_time2 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
    %     all_runtime_profiling_1Loop(c, 3) = Times_1Loop.profile_time3 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
    %     all_runtime_profiling_1Loop(c, 4) = Times_1Loop.profile_time4 / PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS;
    %     %all_runtime_profiling_1Loop(c, 5) = Times_1Loop.profile_time5 / Times_1Loop.profile_pass_combined_conostraint_num;
    %     %all_runtime_profiling_1Loop(c, 6) = Times_1Loop.profile_time6 / Times_1Loop.profile_pass_combined_conostraint_num;
    %     all_runtime_profiling_1Loop(c, 7) = Times_1Loop.profile_pass_geom_constraint_num;
    %     all_runtime_profiling_1Loop(c, 8) = Times_1Loop.profile_pass_photo_constraint_num;
    %     all_runtime_profiling_1Loop(c, 9) = Times_1Loop.profile_pass_combined_conostraint_num;
    %     
    %     time = Times_1Loop.profile_time1 + Times_1Loop.profile_time2 + ...
    %            Times_1Loop.profile_time3 + Times_1Loop.profile_time4 + ...
    %            Times_1Loop.profile_time5 + Times_1Loop.profile_time6;
    %     
    %     all_runtimes_1loop(c)=time;
    %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%         %%%%%%%%%%% TWO-LOOPS METHOD %%%%%%%%%%%
%         %tic;
%         Times_2loops = est_rel_pose_2Loops...
%                        ( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
%                          all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
%         %time=toc;
%         time = Times_2loops.profile_time1 + Times_2loops.profile_time2 + Times_2loops.profile_time3 + Times_2loops.profile_time4 + ...
%                Times_2loops.profile_time5 + Times_2loops.profile_time6 + Times_2loops.profile_time7;
%            
%         all_runtime_profiling_2loops(c, 1) = Times_2loops.profile_time1 / PARAMS.NUM_OF_FIRST_ITERATIONS;
%         all_runtime_profiling_2loops(c, 2) = Times_2loops.profile_time2 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS*PARAMS.NUM_OF_THIRD_ITERATIONS);
%         all_runtime_profiling_2loops(c, 3) = Times_2loops.profile_time3 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS*PARAMS.NUM_OF_THIRD_ITERATIONS);
%         all_runtime_profiling_2loops(c, 4) = Times_2loops.profile_time4 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS*PARAMS.NUM_OF_THIRD_ITERATIONS);
%         all_runtime_profiling_2loops(c, 5) = Times_2loops.profile_time5 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS*PARAMS.NUM_OF_THIRD_ITERATIONS);
%         all_runtime_profiling_2loops(c, 6) = Times_2loops.profile_time6 / Times_2loops.profile_pass_combined_conostraint_num;
%         all_runtime_profiling_2loops(c, 7) = Times_2loops.profile_time7 / Times_2loops.profile_pass_combined_conostraint_num;
%         all_runtime_profiling_2loops(c, 8) = Times_2loops.profile_pass_geom_constraint_num;
%         all_runtime_profiling_2loops(c, 9) = Times_2loops.profile_pass_photo_constraint_num;
%         all_runtime_profiling_2loops(c, 10) = Times_2loops.profile_pass_combined_conostraint_num;
%         
%         all_runtimes_ours_2loops(c) = time;
%         
%         all_min_iterations_2Loops(c, 1) = Times_2loops.Min_Iterations1;
%         all_min_iterations_2Loops(c, 2) = Times_2loops.Min_Iterations2;
%         
%         pose_estimation_err_2Loops(c, 1) = Times_2loops.orientation_err_deg;
%         pose_estimation_err_2Loops(c, 2) = Times_2loops.transl_err;
%         pose_estimation_err_2Loops(c, 3) = Times_2loops.success_flag;
%         pose_estimation_err_2Loops(c, 4) = Times_2loops.max_inlier_count_when_success;
%         pose_estimation_err_2Loops(c, 5) = Times_2loops.max_inlier_count;
%     
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%         %%%%%%%%%%% THREE-LOOPS METHOD %%%%%%%%%%%
%         Times_3loops = est_rel_pose_3Loops...
%                        ( PARAMS, all_pointsRGB_v1, all_pointsRGB_v2, K, R12, T12, ...
%                          all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
% 
%         time = Times_3loops.profile_time1 + Times_3loops.profile_time2 + Times_3loops.profile_time3 + Times_3loops.profile_time4 + ...
%                Times_3loops.profile_time5 + Times_3loops.profile_time6 + Times_3loops.profile_time7 + Times_3loops.profile_time8 + ...
%                Times_3loops.profile_time9;
% 
%         all_runtime_profiling_3loops(c, 1) = Times_3loops.profile_time1 / PARAMS.NUM_OF_FIRST_ITERATIONS;
%         all_runtime_profiling_3loops(c, 2) = Times_3loops.profile_time2 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS);
%         all_runtime_profiling_3loops(c, 3) = Times_3loops.profile_time3 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS);
%         all_runtime_profiling_3loops(c, 4) = Times_3loops.profile_time4 / (PARAMS.NUM_OF_FIRST_ITERATIONS*PARAMS.NUM_OF_SECOND_ITERATIONS);
%         all_runtime_profiling_3loops(c, 5) = Times_3loops.profile_time5 / (Times_3loops.profile_pass_combined_nums_2nd_loop*PARAMS.NUM_OF_FIRST_ITERATIONS);
%         all_runtime_profiling_3loops(c, 6) = Times_3loops.profile_time6 / (Times_3loops.profile_pass_combined_nums_2nd_loop*PARAMS.NUM_OF_FIRST_ITERATIONS);
%         all_runtime_profiling_3loops(c, 7) = Times_3loops.profile_time7 / (Times_3loops.profile_pass_combined_nums_2nd_loop*PARAMS.NUM_OF_FIRST_ITERATIONS);
%         all_runtime_profiling_3loops(c, 8) = Times_3loops.profile_time8 / Times_3loops.profile_pass_combined_nums_3rd_loop;
%         all_runtime_profiling_3loops(c, 9) = Times_3loops.profile_time9 / Times_3loops.profile_pass_combined_nums_3rd_loop;
%         all_runtime_profiling_3loops(c, 10) = Times_3loops.profile_pass_geometric_nums_12;
%         all_runtime_profiling_3loops(c, 11) = Times_3loops.profile_pass_photometric_nums_2;
%         all_runtime_profiling_3loops(c, 12) = Times_3loops.profile_pass_geometric_nums_123 / Times_3loops.profile_pass_combined_nums_2nd_loop;
%         all_runtime_profiling_3loops(c, 13) = Times_3loops.profile_pass_photometric_nums_3 / Times_3loops.profile_pass_combined_nums_2nd_loop;
%         all_runtime_profiling_3loops(c, 14) = Times_3loops.profile_pass_combined_nums_2nd_loop;
%         all_runtime_profiling_3loops(c, 15) = Times_3loops.profile_pass_combined_nums_3rd_loop;
% 
%         all_min_iterations_3Loops(c, 1) = Times_3loops.Min_Iterations1;
%         all_min_iterations_3Loops(c, 2) = Times_3loops.Min_Iterations2;
%         all_min_iterations_3Loops(c, 3) = Times_3loops.Min_Iterations3;
% 
%         pose_estimation_err_3Loops(c, 1) = Times_3loops.orientation_err_deg;
%         pose_estimation_err_3Loops(c, 2) = Times_3loops.transl_err;
%         pose_estimation_err_3Loops(c, 3) = Times_3loops.success_flag;
% 
%         all_runtimes_ours_3loops(c) = time;
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        c=c+1;
    end
end


function [fx, fy, cx, cy] = readCameraIntrinsics(filePath)
    % Read the camera intrinsics file
    intrinsicsMatrix = dlmread(filePath);
    
    % Extract the individual components
    fx = intrinsicsMatrix(1, 1);
    fy = intrinsicsMatrix(2, 2);
    cx = intrinsicsMatrix(1, 3);
    cy = intrinsicsMatrix(2, 3);
end
