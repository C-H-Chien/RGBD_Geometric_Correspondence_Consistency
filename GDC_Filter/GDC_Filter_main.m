close all;
clear all;


PARAMS.POINT_TO_CURVE_DIST_THRESH = 5;
PARAMS.VISUALIZE_CURVES           = 1;

PARAMS.DATASET_PATH              = "/home/chchien/datasets/ICL-NUIM/Noise_Free/"; %> Noise_Free, or Noisy
PARAMS.SEQUENCE_NAME             = "living_room_traj0_frei_png/";
PARAMS.VIEW1_INDX                = 5;
PARAMS.VIEW2_INDX                = 10;

if contains(PARAMS.DATASET_PATH, "Noisy")
    seq_Partition1 = extractBefore(PARAMS.SEQUENCE_NAME, "_frei");
    seq_Partition2 = extractAfter(PARAMS.SEQUENCE_NAME, seq_Partition1);
    PARAMS.SEQUENCE_NAME = strcat(seq_Partition1, "n", seq_Partition2);
end

fx = 481.20; fy = -480.0;
cx = 319.50; cy = 239.50;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

%> Fetch the length of the rgb/depth list
[rgb_filename, depth_filename] = get_ICL_NUIM_rgb_depth_lists(PARAMS);

indx1 = PARAMS.VIEW1_INDX;
indx2 = PARAMS.VIEW2_INDX;

[rgbImage1, rgbImage2, depthMap1, depthMap2, R12, T12] = ...
    load_ICL_NUIM_Sequence_Data_Pair(PARAMS, indx1, indx2, rgb_filename, depth_filename, K);

[X,Y] = meshgrid(1:size(depthMap1,2), 1:size(depthMap1,1));
    
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

[~, sortedIndices] = sort(scores, 'ascend');
matchedPoints1.Location = matchedPoints1.Location(sortedIndices, :);
matchedPoints2.Location = matchedPoints2.Location(sortedIndices, :);

%> Set the anchor point in image 1 and 2
anchor_Indx = 1;
anchor_Point2D_View1 = matchedPoints1.Location(anchor_Indx,:);
x1 = anchor_Point2D_View1(1);
y1 = anchor_Point2D_View1(2);
Z1 = interp2(X, Y, depthMap1, x1, y1);
X1 = (x1 - cx) * Z1 / fx;
Y1 = (y1 - cy) * Z1 / fy;
anchor_Point3D_View1 = [X1, Y1, Z1];

anchor_Point2D_View2 = matchedPoints2.Location(anchor_Indx,:);
x2 = anchor_Point2D_View2(1);
y2 = anchor_Point2D_View2(2);
Z2 = interp2(X, Y, depthMap2, x2, y2);
X2 = (x2 - cx) * Z2 / fx;
Y2 = (y2 - cy) * Z2 / fy;
anchor_Point3D_View2 = [X2, Y2, Z2];

%> Get the 3D points for the entire images 1 and 2
Z_view1 = depthMap1;
X_view1 = ((X - cx) .* Z_view1) / fx;
Y_view1 = ((Y - cy) .* Z_view1) / fy;
all_Points3D_View1 = cat(3, X_view1, Y_view1, Z_view1);

Z_view2 = depthMap2;
X_view2 = ((X - cx) .* Z_view2) / fx;
Y_view2 = ((Y - cy) .* Z_view2) / fy;
all_Points3D_View2 = cat(3, X_view2, Y_view2, Z_view2);

%> Get the 3D points of the matches in image 1 and 2
x1_m = matchedPoints1.Location(:,1);
y1_m = matchedPoints1.Location(:,2);
Z1_m = interp2(X, Y, depthMap1, x1_m, y1_m);
X1_m = (x1_m - cx) .* Z1_m / fx;
Y1_m = (y1_m - cy) .* Z1_m / fy;
matchedPoints3D_View1 = [X1_m, Y1_m, Z1_m];

x2_m = matchedPoints2.Location(:,1);
y2_m = matchedPoints2.Location(:,2);
Z2_m = interp2(X, Y, depthMap1, x2_m, y2_m);
X2_m = (x2_m - cx) .* Z2_m / fx;
Y2_m = (y2_m - cy) .* Z2_m / fy;
matchedPoints3D_View2 = [X2_m, Y2_m, Z2_m];

%> Set the 2D and 3D points of the two picked 2 matches, indexed by (2) and (3)
picked_MatchedPoints2D_View1 = [matchedPoints1.Location(2,:); matchedPoints1.Location(3,:)];
picked_MatchedPoints3D_View1 = [matchedPoints3D_View1(2,:); matchedPoints3D_View1(3,:)];
picked_MatchedPoints2D_View2 = [matchedPoints2.Location(2,:); matchedPoints2.Location(3,:)];

[is_Passed, B_final] = pass_GDC_Filter(PARAMS, anchor_Point3D_View1, picked_MatchedPoints3D_View1, ...
                                       all_Points3D_View2, anchor_Point3D_View2, picked_MatchedPoints2D_View2);

%> Plot the 
if PARAMS.VISUALIZE_CURVES
    figure;
    subplot(1,2,1);
    imshow(rgbImage1); hold on;
    plot(anchor_Point2D_View1(1,1), anchor_Point2D_View1(1,2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); hold on;
    plot(picked_MatchedPoints2D_View1(1,1), picked_MatchedPoints2D_View1(1,2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    hold on;
    plot(picked_MatchedPoints2D_View1(2,1), picked_MatchedPoints2D_View1(2,2), 'ys', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    
    pause(0.3);
    subplot(1,2,2);
    imshow(rgbImage2); hold on;
    plot(anchor_Point2D_View2(1,1), anchor_Point2D_View2(1,2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(B_final{1}(:,2), B_final{1}(:,1), 'c.', 'MarkerSize', 3); hold on;
    plot(picked_MatchedPoints2D_View2(1,1), picked_MatchedPoints2D_View2(1,2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    hold on;
    plot(B_final{2}(:,2), B_final{2}(:,1), 'g.', 'MarkerSize', 3); hold on;
    plot(picked_MatchedPoints2D_View2(2,1), picked_MatchedPoints2D_View2(2,2), 'ys', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
end
