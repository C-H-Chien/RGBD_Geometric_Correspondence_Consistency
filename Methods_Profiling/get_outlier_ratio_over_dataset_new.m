rng(89)


%> Hyper-parameters
PARAMS.DO_TIME_PROFILING             = 0;
PARAMS.APPLY_CONSTRAINTS             = 0;

PARAMS.TOP_RANK_ORDERED_LIST_SIZE    = 100;
PARAMS.REPROJECTION_ERR_PIXEL_THRESH = 2;

%> Criteria for a successful pose estimation
PARAMS.ORIENTATION_DEG_TOL           = 1.0;
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
            
            [rgbImage1, rgbImage2, depthMap1, depthMap2, R12, t12] = load_TUMRGBD_Sequence_Data_Pair ...
            (indx1, indx2, rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename, K, seq_Name);
    
        depthMap1=double(depthMap1)/5000;
depthMap2=double(depthMap2)/5000;
%depthMap1=smoothDepthMap(depthMap1,11, 3);
%depthMap2=smoothDepthMap(depthMap2, 11,3);

image1 = double(rgbImage1)./ 255;
image2 = double(rgbImage2)./ 255;
R=inf;
T=inf;
close all;

image1Gray = single(rgb2gray(image1));
image2Gray = single(rgb2gray(image2));

% Compute SIFT keypoints and descriptors for image 1
[f1, d1] = vl_sift(image1Gray,'Octaves', 8);

% Compute SIFT keypoints and descriptors for image 2
[f2, d2] = vl_sift(image2Gray,'Octaves', 8);
%size(f1)
% Perform nearest-neighbor matching
[matches, scores] = vl_ubcmatch(d1, d2,0.8);
size(matches);
% Get the matched keypoint locations
%matchedPoints1 = f1(1:2, matches(1, :))';
%matchedPoints2 = f2(1:2, matches(2, :))';
matchedPoints1=struct();
matchedPoints1.Location=f1(1:2, matches(1, :))';
matchedPoints2=struct();
matchedPoints2.Location=f2(1:2, matches(2, :))';
peculiar_triplets=[];
 [~, sortedIndices] = sort(scores, 'ascend');
    matchedPoints1.Location = matchedPoints1.Location(sortedIndices, :);
    matchedPoints2.Location = matchedPoints2.Location(sortedIndices, :);
% Detect keypoints and extract SURF features in both images
%{
points1 = detectSURFFeatures(rgb2gray(image1),'MetricThreshold',90);
points2 = detectSURFFeatures(rgb2gray(image2),'MetricThreshold',90);
[features1, validPoints1] = extractFeatures(rgb2gray(image1), points1);
[features2, validPoints2] = extractFeatures(rgb2gray(image2), points2);

 [indexPairs,matchScores] = matchFeatures(features1, features2,'MaxRatio',0.9,'MatchThreshold',50);

    matchedPoints1 = validPoints1(indexPairs(:, 1), :);
    matchedPoints2 = validPoints2(indexPairs(:, 2), :);

       [sortedScores, sortedIndices] = sort(matchScores, 'ascend');
    %indexPairs = indexPairs(sortedIndices, :);
    matchedPoints1 = matchedPoints1(sortedIndices, :);
    matchedPoints2 = matchedPoints2(sortedIndices, :);

%}
   u1 = matchedPoints1.Location(:, 1);
    v1 = matchedPoints1.Location(:, 2);
    z1 = zeros(size(u1));
    for ii = 1 : size(u1, 1)
         [X,Y]=meshgrid(1:size(depthMap1,2),1:size(depthMap1,1));
        z1(ii)=interp2(X,Y,depthMap1,u1(ii),v1(ii));
    end
    x1 = (u1 - cx) .* z1 / fx;
    y1 = (v1 - cy) .* z1 / fy;
    all_points3D1 = [x1, y1, z1];

    u2 = matchedPoints2.Location(:, 1);
    v2 = matchedPoints2.Location(:, 2);
    z2 = zeros(size(u2));
    for ii = 1 : size(u2, 1)
         [X,Y]=meshgrid(1:size(depthMap2,2),1:size(depthMap2,1));
        z2(ii)=interp2(X,Y,depthMap2,u2(ii),v2(ii));%z2(ii) = double(depthMap2(round(v2(ii)), round(u2(ii))));
    end
    x2 = (u2 - cx) .* z2 / fx;
    y2 = (v2 - cy) .* z2 / fy;
    all_points3D2 = [x2, y2, z2];
    wg_depths=[];
    size(all_points3D1);
    for kk=1:size(all_points3D1,1)
        r1=all_points3D1(kk);
        r2=all_points3D2(kk);
        if r1==0 || r2==0
            wg_depths=[kk;wg_depths];
        end
            
    end

    matchedPoints1.Location(wg_depths,:)=[];
    matchedPoints2.Location(wg_depths,:)=[];
    %matchedPoints1(wg_depths)=[]; for surf
    %matchedPoints2(wg_depths)=[];
    all_points3D1(wg_depths,:)=[];
    all_points3D2(wg_depths,:)=[];

            %> Now we have all potential matches with valid depths for both 2D/3D
            all_points2D_v1 = matchedPoints1.Location;
            all_points2D_v2 = matchedPoints2.Location;
            all_points3D_v1 = all_points3D1;
            all_points3D_v2 = all_points3D2;
           
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
            %xyz1 = all_points3D_v1';
            %transformed_xyz1 = K * (R12 * xyz1 + T12);
            %projected_pt = transformed_xyz1 ./ transformed_xyz1(3,:);
            %reproj_errors = all_points2D_v2' - projected_pt(1:2, :);
            %reproj_errors_norm = vecnorm(reproj_errors, 2);

            %> Find indices where the reprojection error is less than some threshold
           % indx_Pass_Reproj_Err_Thresh = find(reproj_errors_norm <= PARAMS.REPROJECTION_ERR_PIXEL_THRESH);
           % num_Of_Matches_Passing_Reproj_Err_Thresh = size(indx_Pass_Reproj_Err_Thresh, 2);
           % Outlier_Ratio = (PARAMS.TOP_RANK_ORDERED_LIST_SIZE - num_Of_Matches_Passing_Reproj_Err_Thresh)/100;
            
            %> Record the outlier ratio
            inlier_list=detemine_gt(image1,image2,depthMap1,depthMap2,matchedPoints1,matchedPoints2,all_points3D1,all_points3D2,R12,t12,seq_Name );
        Outlier_Ratio=1-sum(inlier_list)/length(inlier_list);
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
