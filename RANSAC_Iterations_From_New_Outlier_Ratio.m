% rng(89)
clear all;

%> Hyper-parameters
PARAMS.DO_TIME_PROFILING          = 0;
PARAMS.APPLY_CONSTRAINTS          = 0;
PARAMS.TARGET_OUTLIER_RATIO       = 10; %> 7: 60%~69%; 8: 70%~79%
PARAMS.REPROJECTION_ERR_PIXEL_THRESH = 2;

PARAMS.TOP_RANK_ORDERED_LIST_SIZE = 100;
PARAMS.TOP_N_FOR_FIRST_PAIR       = 25;     %> 10
PARAMS.TOP_N_FOR_SECOND_PAIR      = 50;     %> 25
PARAMS.TOP_N_FOR_THIRD_PAIR       = 100;    %> 100

PARAMS.GEOMETRY_ERR_THRESH        = 0.2;

PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS = 5000;
PARAMS.NUM_OF_FIRST_ITERATIONS          = 20;
PARAMS.NUM_OF_SECOND_ITERATIONS         = 5;
PARAMS.NUM_OF_THIRD_ITERATIONS          = 4;

%> Criteria for a successful pose estimation
PARAMS.ORIENTATION_DEG_TOL              = 1.0;
PARAMS.TRANSLATION_DIST_TOL             = 0.05;

%> Load all outlier ratio data
all_Outlier_Ratios_Dir = "/gpfs/data/bkimia/cchien3/RGBD_Geometry_Depth_Consistency_RANSAC/GT_Constructions/";
Outlier_Ratio_Data_by_Ranges = load(strcat(all_Outlier_Ratios_Dir, "Outlier_Ratio_Data_by_Ranges.mat"));
Outlier_Ratio_Data_by_Ranges = Outlier_Ratio_Data_by_Ranges.Outlier_Ratio_Ranges;
Outlier_Ratio_Data = Outlier_Ratio_Data_by_Ranges{PARAMS.TARGET_OUTLIER_RATIO, 1};

%pose_estimation_err_classic = zeros(size(Outlier_Ratio_Data, 1), 11);
pose_estimation_err_1Loop = zeros(size(Outlier_Ratio_Data, 1), 12);
%pose_estimation_err_2Loops  = zeros(size(Outlier_Ratio_Data, 1), 12);
%pose_estimation_err_3Loops  = zeros(size(Outlier_Ratio_Data, 1), 13);

inlier_indices_1Loop = cell(size(Outlier_Ratio_Data, 1), 4);

NumOfImgPairs = 0;
for di = 1:size(Outlier_Ratio_Data, 1)
    
    %> Exclude the image pairs with outlier ratio > 95%, if the target
    %  outlier ratio is 90%-99%
    if PARAMS.TARGET_OUTLIER_RATIO == 10
        if double(Outlier_Ratio_Data(di, 4)) < 0.95
            continue;
        end
    end
    NumOfImgPairs = NumOfImgPairs + 1;
    seq_Name = Outlier_Ratio_Data(di, 1);
    
    %> Monitor the process
    fprintf(". ");
    if mod(di, 50) == 0, fprintf("\n"); end
    
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

    %> Get time stamps and file names of the sequence 
    %  (this may be redundant if the same sequence is stacked in the Outlier_Ratio_Data array)
    [rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename] = get_TUMRGBD_rgb_depth_lists(seq_Name);

    %> Fetch RGB and Depth image pairs according to the RGBimage pair names
    %  from Outlier_Ratio_Data array
    RGB_Img_Pair_Names = Outlier_Ratio_Data(di, 2:3);
    [rgbImage1, rgbImage2, depthMap1, depthMap2, R12, T12] = load_TUMRGBD_Sequence_Data_Pair_From_Img_Names ...
         (rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename, K, RGB_Img_Pair_Names, seq_Name);
    
    depthMap1 = double(depthMap1)/5000;
    depthMap2 = double(depthMap2)/5000;   
    image1    = double(rgbImage1)./ 255;
    image2    = double(rgbImage2)./ 255;
    
    image1Gray = single(rgb2gray(image1));
    image2Gray = single(rgb2gray(image2));

    %> Compute SIFT features in both images
    [f1, d1] = vl_sift(image1Gray,'Octaves', 8);
    [f2, d2] = vl_sift(image2Gray,'Octaves', 8);
    [matches, scores] = vl_ubcmatch(d1, d2, 0.8);
    
    matchedPoints1 = struct();
    matchedPoints1.Location = f1(1:2, matches(1, :))';
    matchedPoints2 = struct();
    matchedPoints2.Location = f2(1:2, matches(2, :))';
    
    [~, sortedIndices] = sort(scores, 'ascend');
    matchedPoints1.Location = matchedPoints1.Location(sortedIndices, :);
    matchedPoints2.Location = matchedPoints2.Location(sortedIndices, :);
    
    %points1 = detectSURFFeatures(rgb2gray(image1), 'MetricThreshold', 90);
    %points2 = detectSURFFeatures(rgb2gray(image2), 'MetricThreshold', 90);
    %[features1, validPoints1] = extractFeatures(rgb2gray(image1), points1);
    %[features2, validPoints2] = extractFeatures(rgb2gray(image2), points2);

    %> Match features between the images
    %[indexPairs,matchScores] = matchFeatures(features1, features2,'MaxRatio',0.7,'MatchThreshold',80);
    %matchedPoints1 = validPoints1(indexPairs(:, 1), :);
    %matchedPoints2 = validPoints2(indexPairs(:, 2), :);

    %> Make a top rank-ordered list of potential matches
    %[sortedScores, sortedIndices] = sort(matchScores, 'ascend');
    %matchedPoints1 = matchedPoints1(sortedIndices, :);
    %matchedPoints2 = matchedPoints2(sortedIndices, :);

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

%     %%%%%%%%%%% CLASSIC METHOD %%%%%%%%%%%
%     Times_classic = method_classic( PARAMS, K, R12, T12, ...
%                                     all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
% 
%     %> When accurate pose estimation is obtained
%     store_Data_Classic = [Times_classic.Min_Iterations, ...
%         Times_classic.orientation_err_deg_when_success, ...
%         Times_classic.transl_err_when_success, ...
%         Times_classic.max_inlier_ratio_for_accurate_pose_when_success, ...
%         Times_classic.success, ...
%         Times_classic.orientation_err_deg, ...
%         Times_classic.transl_err, ...
%         Times_classic.max_inlier_ratio_for_accurate_pose, ...
%         Times_classic.orientation_err_deg_from_sampson, ...
%         Times_classic.transl_err_from_sampson, ...
%         Times_classic.max_inlier_ratio_for_accurate_pose_from_sampson
%     ];
% 
%     pose_estimation_err_classic(di, :) = store_Data_Classic;
%     
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%% 1-LOOP METHOD %%%%%%%%%%%
    Times_1Loop = method_1Loop( PARAMS, K, R12, T12, ...
                                all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);

    %> When accurate pose estimation is obtained
    store_Data_1Loop = [Times_1Loop.Min_Iterations, ...
        Times_1Loop.pass_GDC_constraint_count/PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS, ...
        Times_1Loop.orientation_err_deg_when_success, ...
        Times_1Loop.transl_err_when_success, ...
        Times_1Loop.max_inlier_ratio_for_accurate_pose_when_success, ...
        Times_1Loop.success, ...
        Times_1Loop.orientation_err_deg, ...
        Times_1Loop.transl_err, ...
        Times_1Loop.max_inlier_ratio_for_accurate_pose, ...
        Times_1Loop.orientation_err_deg_from_sampson, ...
        Times_1Loop.transl_err_from_sampson, ...
        Times_1Loop.max_inlier_ratio_for_accurate_pose_from_sampson
    ];

    inlier_indices_1Loop{di, 1} = Times_1Loop.inlier_indices_from_alignment_when_success;
    inlier_indices_1Loop{di, 2} = Times_1Loop.inlier_indices_from_sampson_when_success;
    inlier_indices_1Loop{di, 3} = Times_1Loop.inlier_indices_from_alignment;
    inlier_indices_1Loop{di, 4} = Times_1Loop.inlier_indices_from_sampson;

    pose_estimation_err_1Loop(di, :) = store_Data_1Loop;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     %%%%%%%%%%% TWO-LOOPS METHOD %%%%%%%%%%%
%     %tic;
%     Times_2loops = method_2Loops(PARAMS, K, R12, T12, ...
%                                  all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
%     
%     store_Data_2Loops = [Times_2loops.Min_Iterations1, ...
%         Times_2loops.Min_Iterations2, ...
%         Times_2loops.orientation_err_deg_when_success, ...
%         Times_2loops.transl_err_when_success, ...
%         Times_2loops.max_inlier_ratio_for_accurate_pose_when_success, ...
%         Times_2loops.success, ...
%         Times_2loops.orientation_err_deg, ...
%         Times_2loops.transl_err, ...
%         Times_2loops.max_inlier_ratio_for_accurate_pose, ...
%         Times_2loops.orientation_err_deg_from_sampson, ...
%         Times_2loops.transl_err_from_sampson, ...
%         Times_2loops.max_inlier_ratio_for_accurate_pose_from_sampson
%     ];
% 
%     pose_estimation_err_2Loops(di, :) = store_Data_2Loops;
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     %%%%%%%%%%% THREE-LOOPS METHOD %%%%%%%%%%%
%     Times_3loops = method_3Loops( PARAMS, K, R12, T12, ...
%                                   all_points2D_v1, all_points2D_v2, all_points3D_v1, all_points3D_v2);
% 
%     store_Data_3Loops = [Times_3loops.Min_Iterations1, ...
%         Times_3loops.Min_Iterations2, ...
%         Times_3loops.Min_Iterations3, ...
%         Times_3loops.orientation_err_deg_when_success, ...
%         Times_3loops.transl_err_when_success, ...
%         Times_3loops.max_inlier_ratio_for_accurate_pose_when_success, ...
%         Times_3loops.success, ...
%         Times_3loops.orientation_err_deg, ...
%         Times_3loops.transl_err, ...
%         Times_3loops.max_inlier_ratio_for_accurate_pose, ...
%         Times_3loops.orientation_err_deg_from_sampson, ...
%         Times_3loops.transl_err_from_sampson, ...
%         Times_3loops.max_inlier_ratio_for_accurate_pose_from_sampson
%     ];
% 
%     pose_estimation_err_3Loops(di, :) = store_Data_3Loops;
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% indx = find(pose_estimation_err_2Loops(:,6) == 0);
% pose_estimation_err_2Loops(indx, :) = [];
% 
% indx = find(pose_estimation_err_3Loops(:,7) == 0);
% pose_estimation_err_3Loops(indx, :) = [];

