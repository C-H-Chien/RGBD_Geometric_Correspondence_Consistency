function[ matchPairs12]= establish_gt_algo_new(points1,points2,all_features_3d1,all_features_3d2,features1,features2,image1,image2,depthMap1,depthMap2,R12,t12,seq_Name,tr,tp)
%  input is valid features, depth maps and the realtive pose
% output is match indices from img1 to img2, img2 to img1, and the union 
% as well as feature indices with no matches in both the images
%tr=10;
already_matched=zeros(length(points2),1);
depth1=depthMap1;
depthThreshold=tp;
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
matchPairs12=[];
matchPairs21=[];
nomatch1=[];
nomatch2=[];
% creat a depth_list which has the depth of features

depths1=double(all_features_3d1(:,3));
depths2=double(all_features_3d2(:,3));

matched=zeros(length(depths2),1);
% loop over all features in image on // reproject
sim_thresh=0.8;
for i=1:length(points1.Location)
    x1=points1.Location(i,1);
    y1=points1.Location(i,2);
    z1=depths1(i);
  
    
    point3D1=all_features_3d1(i,:);
   
    transformed_pt1=R12*point3D1'+t12';
    x1_transformed=fx*transformed_pt1(1)/transformed_pt1(3)+cx;
    y1_transformed=fy*transformed_pt1(2)/transformed_pt1(3)+cy;
    transformed_pt_2d=[x1_transformed y1_transformed];

    distances=sqrt(sum((points2.Location-transformed_pt_2d).^2,2));
    
    within_radius_idx=find(distances<=tr/2);
    
    if isempty(within_radius_idx)
    x=x1;
    y=y1;
    halfN = floor(tr/ 2);
    halfN=3;
    l=x1-halfN;
    t=y1-halfN;
    r=x1+halfN;
    b=y1+halfN;
    [ox,oy]=meshgrid(l:r,t:b);
    ix=floor(l):ceil(r);
    iy=floor(t):ceil(b);

    neighborhoodDepth=interp2(ix,iy,depthMap1(iy,ix),ox,oy);
   neighborhoodDepth = depth1(max(1, floor(y-halfN)):min(size(depth1, 1), ceil(y+halfN)), ...
                                  max(1, floor(x-halfN)):min(size(depth1, 2), ceil(x+halfN)));
    % Calculate the minimum and maximum depth values in the neighborhood

    % instead of looping over , just see the range of depth in the
    % projected patch and see interection with the target depth
    %lets see 
flatteneddepth= reshape(neighborhoodDepth, [], 1);
 
    %patch_3dz=flatteneddepth;
    minDepth = min(neighborhoodDepth(:));
    maxDepth= max(neighborhoodDepth(:));

    for d =minDepth:0.001:maxDepth
            %d=flatteneddepth(dii);
            pt1=points1.Location(i,:);
            z1=d;
            u1=pt1(1);
            v1=pt1(2);
            x1 = (u1 - cx) .* z1 / fx;
            y1 = (v1 - cy) .* z1 / fy;
            point3D1 = [x1, y1, z1]; 
            transformed_pt1 = (R12 * point3D1' + t12)';
            x1_transformed=fx*transformed_pt1(1)/transformed_pt1(3)+cx;
            y1_transformed=fy*transformed_pt1(2)/transformed_pt1(3)+cy;
            transformed_pt_2d=[x1_transformed y1_transformed];
            distances=sqrt(sum((points2.Location-transformed_pt_2d).^2,2));
          
            % find feature2 indices with satisfy the radius
            within_radius_idx=find(distances<=tr/2);
            
            distances(within_radius_idx);
            length(within_radius_idx);
         
            if ~isempty(within_radius_idx)
                  depthDiff=abs(d-double(depths2(within_radius_idx)))./(d+double(depths2(within_radius_idx)));
                if length(within_radius_idx)==1
                    val= abs(depths2(within_radius_idx)-d)./(d+double(depths2(within_radius_idx)));
                    if double(val)<=depthThreshold
                    [~, minDistIdx] = min(distances(within_radius_idx));

                [~,minDepthIdx]= min(depthDiff);
                [~, minDistIdx] = min(sum((features1(i,:)-features2(within_radius_idx,:)).^2,2));

                %check if match from 2 to 1 is also 
                potential_3d2=all_features_3d2(within_radius_idx(minDistIdx),:);

                %back_porjections-R12'*t12
                transformed_pt2 = (R12' * potential_3d2' + -R12'*t12)';
                x2_transformed=fx*transformed_pt2(1)/transformed_pt2(3)+cx;
                y2_transformed=fy*transformed_pt2(2)/transformed_pt2(3)+cy;
                %[ ~ ,min_idx1]=min(sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2));
                %if within_radius_idx(minDistIdx)==within_radius_idx(minDepthIdx)
                c=1;
                
                while  c<=length(within_radius_idx)&& already_matched(within_radius_idx(c))~=0 
                    
                    c=c+1;
                end
                if c>length(within_radius_idx) || already_matched(within_radius_idx(c))~=0
                    continue;
                
                else
                    minDistIdx=c;
                 end
                  if sqrt(sum((features1(i,:)-features2(within_radius_idx(minDistIdx),:)).^2,2))<=sim_thresh %&&sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2)<=tr/2
                    matchPairs12 = [matchPairs12; i, within_radius_idx(minDistIdx)];
                    already_matched(within_radius_idx(minDistIdx))=1;
                    break;
                   end
                 %   break;
                %end
                    end
                    %continue;

                end
                
                depthDiff=abs(d-double(depths2(within_radius_idx)))./(d+double(depths2(within_radius_idx)));
                validMatchesIdx = within_radius_idx(depthDiff <= depthThreshold);

                 if ~isempty(validMatchesIdx)
                % Find the index of the feature with the closest depth difference.
                [~, minDistIdx] = min(distances(within_radius_idx));
                [~,minDepthIdx]= min(depthDiff);
                [~, minDistIdx] = min(sum((features1(i,:)-features2(within_radius_idx,:)).^2,2));
                %if potential_3d2=all_features_3d2(minDistIdx,:);
                 potential_3d2=all_features_3d2(within_radius_idx(minDistIdx),:);
                %back_porjections-R12'*t12
                transformed_pt2 = (R12' * potential_3d2' + -R12'*t12)';
                x2_transformed=fx*transformed_pt2(1)/transformed_pt2(3)+cx;
                y2_transformed=fy*transformed_pt2(2)/transformed_pt2(3)+cy;
                %[ ~ ,min_idx1]=min(sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2));
                %if within_radius_idx(minDistIdx)==within_radius_idx(minDepthIdx)
                %([x2_transformed y2_transformed]-points1.Location(i,:)).^2
                c=1;
                
                while c<=length(within_radius_idx) && already_matched(within_radius_idx(c))~=0 
                    
                    c=c+1;
                end
                if c>length(within_radius_idx) || already_matched(within_radius_idx(c))~=0
                    continue;
                
                else
                    minDistIdx=c;
                 end
                  if sqrt(sum((features1(i,:)-features2(within_radius_idx(minDistIdx),:)).^2,2))<=sim_thresh %&&sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2)<=tr/2
                    matchPairs12 = [matchPairs12; i, within_radius_idx(minDistIdx)];
                    already_matched(within_radius_idx(minDistIdx))=1;
                     break;
                  end
               
                %end
               
                end


            end

           
         % o meaning this feature has no match
  
       
    end
    else
        depthDiff=abs(z1-depths2(within_radius_idx))./(z1+double(depths2(within_radius_idx)));
                validMatchesIdx = within_radius_idx(depthDiff <= depthThreshold);

                 if length(validMatchesIdx)>0
                % Find the index of the feature with the closest depth difference.
                [~, minDepthIdx] = min(depthDiff);
                [~, minDistIdx] = min(distances(within_radius_idx));
                
                [~, minDistIdx] = min(sum((features1(i,:)-features2(within_radius_idx,:)).^2,2));
                potential_3d2=all_features_3d2(within_radius_idx(minDistIdx),:);

                %back_porjections-R12'*t12
                transformed_pt2 = (R12' * potential_3d2' + -R12'*t12)';
                x2_transformed=fx*transformed_pt2(1)/transformed_pt2(3)+cx;
                y2_transformed=fy*transformed_pt2(2)/transformed_pt2(3)+cy;
                %[ ~ ,min_idx1]=min(sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2));
                %if within_radius_idx(minDistIdx)==within_radius_idx(minDepthIdx)
                c=1;
                while c<=length(within_radius_idx) && already_matched(within_radius_idx(c))~=0 %&& c<=length(within_radius_idx)
                    
                    c=c+1;
                end
                if c>length(within_radius_idx) || already_matched(within_radius_idx(c))~=0
                    continue;
                
                else
                    minDistIdx=c;
                 end
                  if sqrt(sum((features1(i,:)-features2(within_radius_idx(minDistIdx),:)).^2,2))<=sim_thresh %&&sum(([x2_transformed y2_transformed]-points1.Location(i,:)).^2,2)<=tr/2
                    matchPairs12 = [matchPairs12; i, within_radius_idx(minDistIdx)];
                    already_matched(within_radius_idx(minDistIdx))=1;
                    %end
                 end
                 end

    end
    % Project this point to the second image and find if there exists a
    % feature correspondence for it



end
end
