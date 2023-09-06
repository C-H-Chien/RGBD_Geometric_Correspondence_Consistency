% Load RGB images and depth maps
% close all;
clear all;
%num_matches_to_show=24; % if you want to show 50 then put 100 if 30 then put 60 because matches are sampled at 2 apart.. 

%> [CH] (Confirmation required)
num_matches_to_show = 12;

%> [CH] (Confirmation required)
%[rgbImage1,rgbImage2,depthMap1,depthMap2,F,camera_params]=load_tum_office(1,num_matches_to_show*2);

% [rgbImage1,rgbImage2,depthMap1,depthMap2,F,camera_params]=load_tum_office(1,100);


[rgbImage1,rgbImage2,depthMap1,depthMap2,R12,T12]=load_tum_office(1,100);

%[rgbImage1,rgbImage2,depthMap1,depthMap2,F,camera_params]=load_tum_office(1,num_matches_to_show);
%rgbImage1 = imread('image1.jpg');  % Replace 'image1.jpg' with the path to your first RGB image
%rgbImage2 = imread('image2.jpg');  % Replace 'image2.jpg' with the path to your second RGB image
%depthMap1 = imread('depth1.png');  % Replace 'depth1.png' with the path to your first depth map
%depthMap2 = imread('depth2.png');  % Replace 'depth2.png' with the path to your second depth map

% Convert depth maps to double precision
depthMap1 = double(depthMap1)/5000 ; % Assuming depth maps are in the range [0, 255]
depthMap2 = double(depthMap2)/5000 ;
% Camera intrinsics (TUM RGB-D dataset)
fx = 525.0;
fy = 525.0;
cx = 319.5;
cy = 239.5;
%fx=535.4;fy=539.2;cx=320.1;cy=247.6;

%> Perform SURF feature detection and matching
points1 = detectSURFFeatures(rgb2gray(rgbImage1));
points2 = detectSURFFeatures(rgb2gray(rgbImage2));
features1 = extractFeatures(rgb2gray(rgbImage1), points1);
features2 = extractFeatures(rgb2gray(rgbImage2), points2);
[indexPairs,matchScores] = matchFeatures(features1, features2);
 [indexPairs,matchScores] = matchFeatures(features1, features2,'MaxRatio',0.8,'MatchThreshold',80);
colors  = ['r.';'g.';'b.';'y.';'c.';'m.'];
colors_ = ['r';'g';'b';'y';'c';'m'];
colors_pt=['r*';'g*';'b*';'y*';'c*';'m*'];
% Retrieve matched points
[sortedScores, sortedIndices] = sort(matchScores, 'ascend');
    indexPairs = indexPairs(sortedIndices, :);
    
matchedPoints1 = points1(indexPairs(:, 1));
matchedPoints2 = points2(indexPairs(:, 2));


ind=2;
%ind=1;
correspondencePoint1= matchedPoints1(ind).Location;
x1 = round(correspondencePoint1(1));
y1 = round(correspondencePoint1(2));
Z1 = depthMap1(y1, x1);
X1 = (x1 - cx) * Z1 / fx;
Y1 = (y1 - cy) * Z1 / fy;
refPoint3D1= [X1, Y1, Z1];
radiusValues=[];
x1_=x1;
y1_=y1;

refPoint3D2 = matchedPoints2(ind).Location;
x2 = round(refPoint3D2(1));
y2 = round(refPoint3D2(2));
x2_=x2;
y2_=y2;

% Convert pixel coordinates to 3D point in camera frame (Image 2)
Z2 = depthMap2(y2, x2);
X2 = (x2 - cx) * Z2 / fx;
Y2 = (y2 - cy) * Z2 / fy;
refPoint3D2 = [X2, Y2, Z2];

figure;
showMatchedFeatures(rgbImage1,rgbImage2,matchedPoints1(ind),matchedPoints2(ind),'montage','PlotOptions',{'d','d','r'}); hold on % the first anchor 
matchedPoints1(ind)=[];
matchedPoints2(ind)=[];
%sample some matches to show

%> [CH] (Confirmation required)
matchedPoints1 = matchedPoints1(sortedIndices(1:num_matches_to_show), :);
matchedPoints2 = matchedPoints2(sortedIndices(1:num_matches_to_show), :);

% matchedPoints1 = matchedPoints1(sortedIndices(1:num_matches_to_show), :);
% matchedPoints2 = matchedPoints2(sortedIndices(1:num_matches_to_show), :);

matchedPoints1(2)=[];
matchedPoints2(2)=[];

showMatchedFeatures(rgbImage1, rgbImage2, matchedPoints1, matchedPoints2, 'montage');
hold on;
for i = 1:size(matchedPoints1,1)
    pt1=matchedPoints1(i).Location;
    pt2=matchedPoints2(i).Location;
    line([pt1(1, 1) pt2(1, 1) + size(rgbImage1, 2)], ...
         [pt1(1, 2) pt2(1, 2)], 'Color', colors_(mod(i,6)+1, :), 'LineWidth', 1);
end

%> [CH]
set(findall(gcf,'-property','FontSize'),'FontSize',15);
set(gcf,'color','w');

for i=1:size(matchedPoints1,1)
    
    correspondencePoint1=matchedPoints1(i).Location;
    x1 = round(correspondencePoint1(1));
    y1 = round(correspondencePoint1(2));
    Z1 = depthMap1(y1, x1);
    X1 = (x1 - cx) * Z1 / fx;
    Y1 = (y1 - cy) * Z1 / fy;
    Point3D= [X1, Y1, Z1];
    radiusValues = [radiusValues;sqrt(sum((refPoint3D1-Point3D).^2))];
end

figure;

%> [CH]
t = tiledlayout(1,2);

% Iterate over each radius value
%> [CH]
nexttile;
%subplot(1,2,1);
for r = 1:numel(radiusValues)
    radius = radiusValues(r);
    correspondencePoint1=matchedPoints1(r).Location;
    x1 = round(correspondencePoint1(1));
    y1 = round(correspondencePoint1(2));
    
%     if x1 == 394 && y1 == 167
%         continue;
%     end
    
%     if x1 == 467 && y1 == 144
%         fprintf("Here!\n");
%     end
    
    % Get boundary points within the specified radius from the correspondence point in Image 1
    boundaryPoints = getBoundaryPoints(rgbImage1, refPoint3D1, radius,depthMap1, cx, cy, fx, fy, x1, y1, 1);

    % Set the boundary points in the radial map as white
    if r==1
        imshow(rgbImage1);hold on;
    end
    plot(boundaryPoints(:,1),boundaryPoints(:,2),colors(mod(r,6)+1,:), 'MarkerSize', 2); hold on;
    
    plot(x1, y1, 'Marker', 's', 'MarkerSize', 10, 'MarkerFaceColor', colors_(mod(r,6)+1,:), 'MarkerEdgeColor', 'w');hold on;
    
    %plot(x1,y1,colors_pt(mod(r,6)+1,:), 'Marker', 's', 'MarkerSize',8, 'LineWidth', 2); hold on; 
    plot(x1_, y1_, "s", 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k'); hold on;
end

%> [CH]
nexttile;
%subplot(1,2,2);

% Iterate over each radius value in image 2
for r = 1:numel(radiusValues)
    
    radius = radiusValues(r);
    correspondencePoint1=matchedPoints2(r).Location;
    x2 = round(correspondencePoint1(1));
    y2 = round(correspondencePoint1(2));
    if y2 == 147
        fprintf("Here!\n");
    end
%     if x2 == 428 && y2 == 170
%         fprintf("Here!\n");
%     end
    
    % Get boundary points within the specified radius from the correspondence point in Image 2
    boundaryPoints = getBoundaryPoints(rgbImage2, refPoint3D2, radius, depthMap2,cx, cy, fx, fy, x2, y2, 0);

    if r==1
        imshow(rgbImage2);hold on;
    end
    
    %> Show the projected contour
    plot(boundaryPoints(:,1),boundaryPoints(:,2),colors(mod(r,6)+1,:),'MarkerSize', 2); hold on;
    
    %> For the second image we need to decide whether the point is
    %  sufficiently close to the projected contour.
    dist_x = x2 - boundaryPoints(:,1);
    dist_y = y2 - boundaryPoints(:,2);
    dist_t = sqrt(dist_x.^2 + dist_y.^2);
    if min(dist_t) < 8
        plot(x2,y2, 'Marker', 's', 'MarkerSize', 10, 'MarkerFaceColor', colors_(mod(r,6)+1,:), 'MarkerEdgeColor', 'w');hold on; 
    else
        plot(x2,y2, 'Marker', 's', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w');hold on; 
    end

    plot(x2_, y2_, "s", 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');hold on;
    
end
%imshowpair(rgbImage2,gcf,'blend')

%> [CH]
t.TileSpacing = 'none';
% t.Padding = 'tight';
% set(findall(gcf,'-property','FontSize'),'FontSize',15);
set(gcf,'color','w');

% Function to get boundary points within a specified radius from a correspondence point
function boundaryPoints = getBoundaryPoints(rgbImage, correspondencePoint, radius, depthMap, cx, cy, fx, fy, x1, y1, check_on_boundary)
    % Get image size
    rgbImage = double(rgbImage) / 255;
    [height, width, ~] = size(rgbImage);

    % Create meshgrid of pixel coordinates
    [X, Y] = meshgrid(1:width, 1:height);

    % Convert pixel coordinates to 3D coordinates for the entire image
    Z = depthMap;
    X = ((X - cx) .* Z) / fx;
    Y = ((Y - cy) .* Z) / fy;
    points3D = cat(3, X, Y, Z);
%     dummy=repmat(correspondencePoint, [height, width, 1]);
%     
%     dummy=reshape(dummy,size(rgbImage));
%     distances = sqrt(sum((points3D - dummy).^2, 3));
    
    dist_X = correspondencePoint(1) - points3D(:,:,1);
    dist_Y = correspondencePoint(2) - points3D(:,:,2);
    dist_Z = correspondencePoint(3) - points3D(:,:,3);
    distances = sqrt(dist_X.^2 + dist_Y.^2 + dist_Z.^2);
    
    % Convert radius from meters to 3D distance units
    radius3D = radius;

    % Calculate Euclidean distances between all points and the correspondence point
    %distances = sqrt(sum((points3D - correspondencePoint).^2, 2));
    
    % Create a binary mask based on the distance criteria
    mask = distances < radius3D;
    
    %> Search the boundaries of the binary mask
    B = bwboundaries(mask);
    
    max_b_points = 0;
    max_b_idx = 0;
    for i = 1:size(B,1)
        if size(B{i},1) > max_b_points
            max_b_points = size(B{i},1);
            max_b_idx = i;
        end
    end
    
    B_final = B{max_b_idx};
    boundaryPoints = [B_final(:,2), B_final(:,1)];
    
    if check_on_boundary
        bool_check = (abs(B_final(:,2) - x1) < 2) & (abs(B_final(:,1) - y1) < 2);
        check_ = find(bool_check == 1);
        if isempty(check_)
            radius_ = sqrt((x1 - B_final(:,2)).^2 + (y1 - B_final(:,1)).^2);
            [min_radius_, idx] = mink(radius_, 5);

            for ii = 1:size(idx, 1)
                xb = B_final(idx(ii),2);
                yb = B_final(idx(ii),1);
                xyd = sqrt((x1-xb).^2+(y1-yb).^2);
                D = linspace(0, round(xyd), 100);
                x = xb + D .* (x1-xb)./xyd;
                y = yb + D .* (y1-yb)./xyd;
                mask(round(y), round(x)) = 1;
            end

            B = bwboundaries(mask);

            max_b_points = 0;
            max_b_idx = 0;
            for i = 1:size(B,1)
                if size(size(B{i}), 1) > max_b_points
                    max_b_points = size(size(B{i}), 1);
                    max_b_idx = i;
                end
            end

            B_final = B{max_b_idx};
            boundaryPoints = [B_final(:,2), B_final(:,1)];
        end
    end
    
%     mask=mask*.1;
%    
%     se = strel('disk', 45);  % Adjust the size of the structuring element as per your requirement
%     mask = imclose(mask, se);
%     % Reshape the binary mask to match the image size
%     %mask = reshape(mask, height, width);
% 
%     % Compute the boundary of the binary mask
%     boundaryMask = bwperim(mask);
%     se = strel('disk', 5);
%     boundaryMask = imclose(boundaryMask, se);
%     % Extract the boundary points
%     [boundaryPointsY, boundaryPointsX] = find(boundaryMask);
%     boundaryPoints = [boundaryPointsX, boundaryPointsY];
end
