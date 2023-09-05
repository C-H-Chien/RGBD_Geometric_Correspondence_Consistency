rng(89)
clear all;

%> Hyper-parameters
PARAMS.DO_TIME_PROFILING             = 0;
PARAMS.APPLY_CONSTRAINTS             = 0;

PARAMS.TOP_RANK_ORDERED_LIST_SIZE    = 100;
PARAMS.REPROJECTION_ERR_PIXEL_THRESH = 2;

%> Criteria for a successful pose estimation
PARAMS.ORIENTATION_DEG_TOL           = 1.5;
PARAMS.TRANSLATION_DIST_TOL          = 0.05;

c = 1;

%> A pool of frame index difference
step_Frame_Sizes = [1; 3; 5; 7; 9; 11; 13; 15; 17; 19; 21; 23; 25; 27; 29; 31];
PARAMS.START_FRAME_INDEX = step_Frame_Sizes(end,1)+1;

%> A pool of TUM-RGBD preprocessed datasets
sequence_Names = ["rgbd_dataset_freiburg1_desk/"; ...
                  "rgbd_dataset_freiburg1_room/"; ...
                  "rgbd_dataset_freiburg1_xyz/"; ...
                  "rgbd_dataset_freiburg2_desk/"; ...
                  "rgbd_dataset_freiburg3_long_office_household/"; ...
                  "rgbd_dataset_freiburg3_structure_texture_near_validation/" ];

% for di = 1:size(sequence_Names, 1)
for di = 2:size(sequence_Names, 1)
    all_Outlier_Ratios = [];
    seq_Name = sequence_Names(di, 1);
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
    
    %> Fetch the length of the rgb/depth list
    [rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename] = get_TUMRGBD_rgb_depth_lists(seq_Name);
    seq_Img_Length = size(rgb_time_stamp, 1);
    
    fprintf(strcat("Sequence: ", seq_Name, "\n"));
    fprintf(strcat("Length: ", string(seq_Img_Length), "\n"));
    
    %> Loop over the entire sequence
    for i = PARAMS.START_FRAME_INDEX:seq_Img_Length
        
        fprintf(". ");
        if mod(i, 50) == 0, fprintf("\n"); end
        
        %> Loop over all step frame sizes
        for si = 1:size(step_Frame_Sizes, 1)
            step_frame_size = step_Frame_Sizes(si, 1);
            
            indx1 = i-step_frame_size;
            indx2 = i;
            
            [rgbImage1, rgbImage2, depthMap1, depthMap2, R12, T12] = load_TUMRGBD_Sequence_Data_Pair ...
            (indx1, indx2, rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename, K, seq_Name);
    
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
            [indexPairs,matchScores] = matchFeatures(features1, features2, 'MaxRatio', 0.7, 'MatchThreshold', 80);
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
            
            %> Continue if the number of matches below the size of top rank-ordered list
            if size(all_points2D_v1, 1) < PARAMS.TOP_RANK_ORDERED_LIST_SIZE
                continue;
            end
            
            %> Get matches only from the top-rank-ordered list
            all_points2D_v1 = all_points2D_v1(1:PARAMS.TOP_RANK_ORDERED_LIST_SIZE, :);
            all_points2D_v2 = all_points2D_v2(1:PARAMS.TOP_RANK_ORDERED_LIST_SIZE, :);
            all_points3D_v1 = all_points3D_v1(1:PARAMS.TOP_RANK_ORDERED_LIST_SIZE, :);
            all_points3D_v2 = all_points3D_v2(1:PARAMS.TOP_RANK_ORDERED_LIST_SIZE, :);
            
            %> Compute the reprojection errors to see how many loops we need to get very good 3 pairs
            xyz1 = all_points3D_v1';
            transformed_xyz1 = K * (R12 * xyz1 + T12);
            projected_pt = transformed_xyz1 ./ transformed_xyz1(3,:);
            reproj_errors = all_points2D_v2' - projected_pt(1:2, :);
            reproj_errors_norm = vecnorm(reproj_errors, 2);

            %> Find indices where the reprojection error is less than some threshold
            indx_Pass_Reproj_Err_Thresh = find(reproj_errors_norm <= PARAMS.REPROJECTION_ERR_PIXEL_THRESH);
            num_Of_Matches_Passing_Reproj_Err_Thresh = size(indx_Pass_Reproj_Err_Thresh, 2);
            Outlier_Ratio = (PARAMS.TOP_RANK_ORDERED_LIST_SIZE - num_Of_Matches_Passing_Reproj_Err_Thresh)/100;
            
            %> Record the outlier ratio
            write_Str_array = [seq_Name, string(rgb_filename(indx1,:)), string(rgb_filename(indx2,:)), string(Outlier_Ratio)];
            all_Outlier_Ratios = [all_Outlier_Ratios; write_Str_array];
            
            if si >= 3 && Outlier_Ratio >= 0.95
                break;
            end
        end
    end
    
    seq_Name_wo_slash = extractBefore(seq_Name, "/");
    write_Mat_File_Name = strcat("Outlier_Ratio_", seq_Name_wo_slash, ".mat");
    save(write_Mat_File_Name, 'all_Outlier_Ratios');
end
