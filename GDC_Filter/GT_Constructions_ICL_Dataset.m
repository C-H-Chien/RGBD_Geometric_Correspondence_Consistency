rng(89)
clear;
%> Construct outlier ratio ranges .mat files from the ICL-NUIM dataset

%> Hyper-parameters
PARAMS.TOP_RANK_ORDERED_LIST_SIZE    = 100;
PARAMS.REPROJECTION_ERR_PIXEL_THRESH = 2;
PARAMS.WRITE_FILES_PATH              = "/gpfs/data/bkimia/RGBD_Dataset/ICL_NUIM/";
PARAMS.VISUALIZE                     = 0;

%> A pool of frame index difference
step_Frame_Sizes = [1; 3; 5; 7; 9; 11; 13; 15; 17; 19; 21; 23; 25; 27; 29; 31; 33];
PARAMS.START_FRAME_INDEX = step_Frame_Sizes(end,1)+1;

%> All sequences of the ICL_TUIM dataset
sequence_Names = ["living_room_traj0_frei_png/"; ...
                  "living_room_traj1_frei_png/"; ...
                  "living_room_traj2_frei_png/"; ...
                  "living_room_traj3_frei_png/" ...
                  "office_room_traj0_frei_png/"; ...
                  "office_room_traj1_frei_png/"; ...
                  "office_room_traj2_frei_png/"; ...
                  "office_room_traj3_frei_png/"];

meshgrid_flag = 1;
for di = 1:size(sequence_Names, 1)
    all_Outlier_Ratios = [];
    seq_Name = sequence_Names(di, 1);
    
    fx = 481.20; fy = -480.0;
    cx = 319.50; cy = 239.50;
    K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

    %> Fetch the length of the rgb/depth list
    [rgb_filename, depth_filename] = get_ICL_NUIM_rgb_depth_lists(seq_Name);
    seq_Img_Length = size(rgb_filename, 1);
    
    fprintf(strcat("Sequence: ", seq_Name, "\n"));
    fprintf(strcat("Length: ", string(seq_Img_Length), "\n"));
    
    all_SIFT_Matches = cell(length(PARAMS.START_FRAME_INDEX:seq_Img_Length), 1);
    stored_SIFT_matches_counter = 1;
    counter = 1;
    
    %> Loop over the entire sequence
    for i = PARAMS.START_FRAME_INDEX:2:seq_Img_Length
        
        fprintf(". ");
        if mod(counter, 50) == 0, fprintf("\n"); end
        counter = counter + 1;
        
        %> Loop over all step frame sizes
        for si = 1:length(step_Frame_Sizes)
            step_frame_size = step_Frame_Sizes(si, 1);
            
            indx1 = i-step_frame_size;
            indx2 = i;
            
            [rgbImage1, rgbImage2, depthMap1, depthMap2, R12, T12] = load_ICL_NUIM_Sequence_Data_Pair ...
            (indx1, indx2, rgb_filename, depth_filename, K, seq_Name);
        
            if meshgrid_flag == 1
                [X,Y] = meshgrid(1:size(depthMap1,2), 1:size(depthMap1,1));
                meshgrid_flag = 0;
            end
    
            %> Noise-free depth maps, so no interpolation is needed, and
            %  there is no invalid depths too
            depthMap1 = double(depthMap1)/5000;
            depthMap2 = double(depthMap2)/5000;

            image1Gray = single(rgb2gray(rgbImage1));
            image2Gray = single(rgb2gray(rgbImage2));

            %> Compute SIFT keypoints and descriptors for image 1
            [f1, d1] = vl_sift(image1Gray, 'Octaves', 8);

            %> Compute SIFT keypoints and descriptors for image 2
            [f2, d2] = vl_sift(image2Gray, 'Octaves', 8);
            
            %> Perform nearest-neighbor matching
            [matches, scores] = vl_ubcmatch(d1, d2);
            
            %> Get the matched keypoint locations
            matchedPoints1.Location = f1(1:2, matches(1, :))';
            matchedPoints2.Location = f2(1:2, matches(2, :))';

            u1 = matchedPoints1.Location(:, 1);
            v1 = matchedPoints1.Location(:, 2);
            z1 = interp2(X, Y, depthMap1, u1, v1); 
            x1 = (u1 - cx) .* z1 / fx;
            y1 = (v1 - cy) .* z1 / fy;
            all_points3D1 = [x1, y1, z1];

            u2 = matchedPoints2.Location(:, 1);
            v2 = matchedPoints2.Location(:, 2);
            z2 = interp2(X, Y, depthMap2, u2, v2); 
            x2 = (u2 - cx) .* z2 / fx;
            y2 = (v2 - cy) .* z2 / fy;
            all_points3D2 = [x2, y2, z2];
            
            %> Now we have all potential matches with valid depths for both 2D/3D
            all_points2D_v1 = matchedPoints1.Location;
            all_points2D_v2 = matchedPoints2.Location;
            all_points3D_v1 = all_points3D1;
            all_points3D_v2 = all_points3D2;
            
            %> Continue if the number of matches below the size of top rank-ordered list
            if size(all_points2D_v1, 1) < PARAMS.TOP_RANK_ORDERED_LIST_SIZE
                continue;
            end
            
            %> Compute the reprojection errors to see how many loops we need to get very good 3 pairs
            xyz1 = all_points3D_v1';
            transformed_xyz1 = K * (R12 * xyz1 + T12);
            projected_pt = transformed_xyz1 ./ transformed_xyz1(3,:);
            reproj_errors = all_points2D_v2' - projected_pt(1:2, :);
            reproj_errors_norm = vecnorm(reproj_errors, 2);
            
            %> Find indices where the reprojection error is less than some threshold
            indx_Pass_Reproj_Err_Thresh = find(reproj_errors_norm <= PARAMS.REPROJECTION_ERR_PIXEL_THRESH);
            num_Of_Matches_Passing_Reproj_Err_Thresh = size(indx_Pass_Reproj_Err_Thresh, 2);
            Outlier_Ratio = (size(all_points2D_v1, 1) - num_Of_Matches_Passing_Reproj_Err_Thresh)/size(all_points2D_v1, 1);
            
            inlier_matches1 = matchedPoints1.Location(indx_Pass_Reproj_Err_Thresh, :);
            inlier_matches2 = matchedPoints2.Location(indx_Pass_Reproj_Err_Thresh, :);

            %> Record the outlier ratio
            write_Str_array = [string(rgb_filename(indx1,:)), string(rgb_filename(indx2,:)), ...
                               string(depth_filename(indx1,:)), string(depth_filename(indx2,:)), string(Outlier_Ratio)];
            all_Outlier_Ratios = [all_Outlier_Ratios; write_Str_array];
            
            %> Record all SIFT matches tro save time when doing RANSAC
            match_feature_locations = [u1, v1, u2, v2];
            all_SIFT_Matches{stored_SIFT_matches_counter, 1} = match_feature_locations;
            stored_SIFT_matches_counter = stored_SIFT_matches_counter + 1;
            
            if PARAMS.VISUALIZE == 1
                figure(1);
                visualize_matches(rgb2gray(rgbImage1), rgb2gray(rgbImage2), [u1, v1], [u2, v2], size(match_feature_locations, 1));
                figure(2);
                visualize_matches(rgb2gray(rgbImage1), rgb2gray(rgbImage2), ...
                                  [inlier_matches1(:,1), inlier_matches1(:,2)], ...
                                  [inlier_matches2(:,1), inlier_matches2(:,2)], size(inlier_matches1, 1));
            end
        end
    end
    
    %> Save Outlier Ratio Ranges for the current sequence
    seq_Name_wo_slash = extractBefore(seq_Name, "/");
    write_Mat_File_Name = strcat(PARAMS.WRITE_FILES_PATH, "Outlier_Ratio_", seq_Name_wo_slash, ".mat");
    save(write_Mat_File_Name, 'all_Outlier_Ratios');
    
    %> Save the corresponding SIFT matches for the current sequence
    %> Delete empty cell arrays in rows
    SIFT_Matches = all_SIFT_Matches(~cellfun('isempty', all_SIFT_Matches));
    write_Mat_SIFT_Matches_File_Name = strcat(PARAMS.WRITE_FILES_PATH, "SIFT_Matches_", seq_Name_wo_slash, ".mat");
    save(write_Mat_SIFT_Matches_File_Name, 'SIFT_Matches');
end
