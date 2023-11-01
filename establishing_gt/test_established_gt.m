
ix=546
[rgbImage1, rgbImage2, depthMap1, depthMap2, R12, t12,F] = load_tum_office(ix-15, ix);
depthMap1=double(depthMap1)/5000;
    depthMap2=double(depthMap2)/5000;
    %depthMap1=smoothDepthMap(depthMap1,11, 3);
    %depthMap2=smoothDepthMap(depthMap2, 11,3);
    
    image1 = double(rgbImage1)./ 255;
    image2 = double(rgbImage2)./ 255;
    R=inf;
    T=inf;
    close all;
    % Detect keypoints and extract SURF features in both images
    
    points1 = detectSURFFeatures(rgb2gray(image1),'MetricThreshold',90);
    points2 = detectSURFFeatures(rgb2gray(image2),'MetricThreshold',90);
    [features1, validPoints1] = extractFeatures(rgb2gray(image1), points1);
    [features2, validPoints2] = extractFeatures(rgb2gray(image2), points2);
    
      miss_depth1=[];
    miss_depth2=[];
    for ik=1:length(validPoints1)
        xk=validPoints1.Location(ik,1);
        yk=validPoints1.Location(ik,2);
        if depthMap1(round(yk),round(xk))==0
            miss_depth1=[miss_depth1;ik];
        end


    end

    for ik=1:length(validPoints2)
        xk=validPoints2.Location(ik,1);
        yk=validPoints2.Location(ik,2);
        if depthMap2(round(yk),round(xk))==0
            miss_depth2=[miss_depth2;ik];
        end


    end
    validPoints1(miss_depth1,:)=[];
    validPoints2(miss_depth2,:)=[];
    features1(miss_depth1,:)=[];
    features2(miss_depth2,:)=[];



figure;
matchedPoints1=validPoints1;
matchedPoints2=validPoints2;

    %calucate Features with missing depth in each image
           u1 = matchedPoints1.Location(:, 1);
    v1 = matchedPoints1.Location(:, 2);
    z1 = zeros(size(u1));
    for ii = 1 : size(u1, 1)
        %z1(ii) = double(depthMap1(round(v1(ii)), round(u1(ii))));
         x0=floor(u1(ii));
         y0=floor(v1(ii));
         dx=u1(ii)-x0;
         dy=v1(ii)-y0;
         D00 = depthMap1(y0, x0);
            D01 = depthMap1(y0, x0 + 1);
            D10 = depthMap1(y0 + 1, x0);
            D11 = depthMap1(y0 + 1, x0 + 1);
        
        % perfrom interpolation 
        interpolatedDepth = (1 - dx) * (1 - dy) * D00 + dx * (1 - dy) * D01 + (1 - dx) * dy * D10 + dx * dy * D11;
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
        
%       z2(ii) = double(depthMap2(round(v2(ii)), round(u2(ii))));

        %
        x0=floor(u2(ii));
         y0=floor(v2(ii));
         dx=u2(ii)-x0;
         dy=v2(ii)-y0;
         D00 = depthMap2(y0, x0);
            D01 = depthMap2(y0, x0 + 1);
            D10 = depthMap2(y0 + 1, x0);
            D11 = depthMap2(y0 + 1, x0 + 1);

        % perfrom interpolation 
        interpolatedDepth = (1 - dx) * (1 - dy) * D00 + dx * (1 - dy) * D01 + (1 - dx) * dy * D10 + dx * dy * D11;
        [X,Y]=meshgrid(1:size(depthMap2,2),1:size(depthMap2,1));
        z2(ii)=interp2(X,Y,depthMap2,u2(ii),v2(ii));
    end
    x2 = (u2 - cx) .* z2 / fx;
    y2 = (v2 - cy) .* z2 / fy;
    all_points3D2 = [x2, y2, z2];
    wg_depths=[];
    size(all_points3D1);



tr=6;tp=0.01
[indexPairs1]= establish_gt_algo_new(validPoints1,validPoints2,all_points3D1,all_points3D2,features1,features2,image1,image2,depthMap1,depthMap2,R12,t12,"freiburg3",tr,tp);
[indexPairs2]= establish_gt_algo_new(validPoints2,validPoints1,all_points3D2,all_points3D1,features2,features1,image2,image1,depthMap2,depthMap1,R12',-R12'*t12,"freiburg3",tr,tp);

indexPairs_gt=remove_dup(indexPairs1,fliplr(indexPairs2));


Figure;showMatchedFeatures(image1,image2,validPoints1(indexPairs_gt(:,1)),validPoints2(indexPairs_gt(:,2)),'montage');

function [result]= remove_dup(M,N)

concatenated = [M; N];
unique_first_column = unique(concatenated(:, 1));

% Initialize a logical mask
mask = true(size(concatenated, 1), 1);

% Iterate through unique values in the first column
for i = 1:length(unique_first_column)
    value = unique_first_column(i);
    
    % Find rows with the current value in the first column
    rows_with_value = concatenated(concatenated(:, 1) == value, :);
    
    % If there are multiple rows with the same value in the first column,
    % check if their second column values are not all the same
    if size(rows_with_value, 1) > 1 && ~all(rows_with_value(:, 2) == rows_with_value(1, 2))
        % Mark rows for removal
        mask(concatenated(:, 1) == value) = false;
    end
end


% Apply the mask to keep only the rows to be preserved
result = concatenated(mask, :);
result= unique(result, 'rows');


end