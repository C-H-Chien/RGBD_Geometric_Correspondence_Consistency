clear all;

PARAMS.REPROJECTION_ERR_PIXEL_THRESH = 2;

PARAMS.TOP_RANK_ORDERED_LIST_SIZE = 250;
PARAMS.TOP_N_FOR_FIRST_PAIR       = 100;
PARAMS.TOP_N_FOR_SECOND_PAIR      = 150;
PARAMS.TOP_N_FOR_THIRD_PAIR       = 200;

PARAMS.NUM_OF_CLASSIC_RANSAC_ITERATIONS = 1500;
PARAMS.NUM_OF_FIRST_ITERATIONS          = 15;
PARAMS.NUM_OF_SECOND_ITERATIONS         = 15;
PARAMS.NUM_OF_THIRD_ITERATIONS          = 15;

PARAMS.GAUSSIAN_SIGMA                   = 3;
PARAMS.WINDOW_HALF_SIZE                 = 3;

PARAMS.DEPTH_SCALE_FACTOR               = 5000;
PARAMS.TARGET_SEQUENCE_NAME             = 1;

NumOfImgPairs = 0;
meshgrid_flag = 1;
step_frame = 1;
counter=1;
rng(0);

dataset_root_path = "/home/chchien/datasets/TUM-RGBD/";

%> All sequences of the dataset
sequence_Names = ["rgbd_dataset_freiburg1_desk/"; ...
                  "rgbd_dataset_freiburg1_room/"; ...
                  "rgbd_dataset_freiburg1_xyz/"; ...
                  "rgbd_dataset_freiburg2_desk/"; ...
                  "rgbd_dataset_freiburg3_structure_texture_near_validation/"; ...
                  "rgbd_dataset_freiburg3_long_office_household/"];

%> Fetch the length of the rgb/depth list
seq_Name = sequence_Names(PARAMS.TARGET_SEQUENCE_NAME);
% imageList = strcat(dataset_root_path, seq_Name, "rgb.txt");
% rgb_filename = importdata(imageList);
% depthList = strcat(dataset_root_path, seq_Name, "depth.txt");
% depth_filename = importdata(depthList);
% seq_Img_Length = min(size(rgb_filename, 1) - 3, size(depth_filename, 1)-3);
% 
% fprintf(strcat("Sequence: ", seq_Name, "\n"));
% fprintf(strcat("Length: ", string(seq_Img_Length), "\n"));
% 
% pose_estimation_err_Doubly_Nested_GDC = zeros(seq_Img_Length, 3);

[rgb_filenames, depth_filenames, GT_rotations, GT_translations, seq_Img_Length] = ...
    load_TUM_RGBD_data(dataset_root_path, seq_Name);

fx = 517.3;
fy = 516.5;
cx = 318.6;
cy = 255.3;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
invK = inv(K);

for ix = step_frame+1 : step_frame: seq_Img_Length

    id1 = ix-step_frame;
    id2 = ix;
    
    % [rgb1, rgb2, depth1, depth2, R12, T12]= load_tum_gen(dataset_root_path, id1, id2, seq_Name);
    rgb1 = imread(rgb_filenames(id1));
    rgb2 = imread(rgb_filenames(id2));
    image1Gray = single(rgb2gray(rgb1));
    image2Gray = single(rgb2gray(rgb2));
    depth1 = imread(depth_filenames(id1));
    depth2 = imread(depth_filenames(id2));
    depthMap1 = double(depth1)/PARAMS.DEPTH_SCALE_FACTOR;
    depthMap2 = double(depth2)/PARAMS.DEPTH_SCALE_FACTOR;

    C1 = GT_translations(:,id1);
    C2 = GT_translations(:,id2);
    R1 = GT_rotations(:,:,id1);
    R2 = GT_rotations(:,:,id2);
    R12 = R2 * inv(R1);
    T12 = R2 * (C1 - C2);
    
    if meshgrid_flag == 1
        [X,Y] = meshgrid(1:size(depthMap1,2), 1:size(depthMap1,1));
        meshgrid_flag = 0;
    end

    %> Compute SIFT features in both images
    [f1, d1] = vl_sift(image1Gray);
    [f2, d2] = vl_sift(image2Gray);
    [matches, scores] = vl_ubcmatch(d1, d2);
    
    matchedPoints1.Location = f1(1:2, matches(1, :))';
    matchedPoints2.Location = f2(1:2, matches(2, :))';
    
    [sorted_scores, sortedIndices] = sort(scores, 'ascend');
    matchedPoints1.Location = matchedPoints1.Location(sortedIndices, :);
    matchedPoints2.Location = matchedPoints2.Location(sortedIndices, :);
    
    %> Get 3D points 
    invalid_depth_indices = [];
    u1 = matchedPoints1.Location(:, 1);
    v1 = matchedPoints1.Location(:, 2);
    z1 = interp2(X, Y, depthMap1, u1, v1);            
    x1 = (u1 - cx) .* z1 / fx;
    y1 = (v1 - cy) .* z1 / fy;
    match_points3D_v1 = [x1, y1, z1];
    invalid_z1_index = find(z1 == 0);
    
    u2 = matchedPoints2.Location(:, 1);
    v2 = matchedPoints2.Location(:, 2);
    z2 = interp2(X, Y, depthMap2, u2, v2);  
    x2 = (u2 - cx) .* z2 / fx;
    y2 = (v2 - cy) .* z2 / fy;
    match_points3D_v2 = [x2, y2, z2];
    invalid_z2_index = find(z2 == 0);
    
    %> Get the 3D points for the entire images 1 and 2 from noisy depths
    X_view1 = ((X - cx) .* depthMap1) / fx;
    Y_view1 = ((Y - cy) .* depthMap1) / fy;
    all_Points3D_View1 = cat(3, X_view1, Y_view1, depthMap1);
    
    X_view2 = ((X - cx) .* depthMap2) / fx;
    Y_view2 = ((Y - cy) .* depthMap2) / fy;
    all_Points3D_View2 = cat(3, X_view2, Y_view2, depthMap2);

    invalid_index_union = union(invalid_z1_index, invalid_z2_index);
    matchedPoints1.Location(invalid_index_union,:) = [];
    matchedPoints2.Location(invalid_index_union,:) = [];
    match_points3D_v1(invalid_index_union,:) = [];
    match_points3D_v2(invalid_index_union,:) = [];

    %%%%%%%%%%% Doubly Nested GDE Filter %%%%%%%%%%%
    [Doubly_Nested_GDC_Data, Final_R, Final_T] = doubly_Nested_GDC_Grad_Rho ...
                 (PARAMS, X, Y, depthMap1, depthMap2, ...
                  R12, T12, K, invK, fx, fy, ...
                  match_points3D_v1, matchedPoints1.Location, match_points3D_v2, matchedPoints2.Location, ...
                  all_Points3D_View1, all_Points3D_View2);

    %> When accurate pose estimation is obtained
    store_Data_Doubly_Nested_GDC = [ ...
        Doubly_Nested_GDC_Data.orientation_err_deg_when_success, ...
        Doubly_Nested_GDC_Data.transl_err_when_success, ...
        Doubly_Nested_GDC_Data.success_flag
    ];

    pose_estimation_err_Doubly_Nested_GDC(counter, :) = store_Data_Doubly_Nested_GDC;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    counter = counter + 1;    
    
    if mod(counter, 50) == 0
        fprintf("\n");
    else
        fprintf(". ");
    end
end
fprintf("\n")

%> Doubly Nested GDC
%> Fetch only valid data (data is valid only when pose estimation is successful)
valid_index = find(pose_estimation_err_Doubly_Nested_GDC(:,end) == 1);
valid_pose_estimation_err_classic = pose_estimation_err_Doubly_Nested_GDC(valid_index,:);

disp("rot error");    sqrt(mean(pose_estimation_err_Doubly_Nested_GDC(:,1).^2))
disp("rmse t error"); sqrt(mean(pose_estimation_err_Doubly_Nested_GDC(:,2).^2))
