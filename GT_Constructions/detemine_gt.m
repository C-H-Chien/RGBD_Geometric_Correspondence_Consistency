function [inlier_list]=detemine_gt(image1,image2,depth1,depth2,matchedPoints1,matchedPoints2,points3D1,points3D2,R12,t12,seq_Name)
tr=8;
tp=0.02;
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
numMatches=size(matchedPoints1.Location,1);
inlier_list=zeros(numMatches,1);
for i=1:numMatches
    pt1 = matchedPoints1.Location(i,:); %2d pixel coorinate
    
    point3D1=points3D1(i,:); % 3d coorindate of that pixel
    point3D2=points3D2(i,:);
    pt2=matchedPoints2.Location(i,:);
    transformed_pt1 = (R12 * point3D1' + t12)';
    x1_transformed=fx*transformed_pt1(1)/transformed_pt1(3)+cx;
    y1_transformed=fy*transformed_pt1(2)/transformed_pt1(3)+cy;
    transformed_pt_2d=[x1_transformed y1_transformed];


    %calculate reprojection 
    reprojection_error=norm(pt2-transformed_pt_2d);
    rho1=transformed_pt1(3);
    rho2=point3D2(3);
   
    depth_proximity=2*abs(rho1-rho2)/(rho1+rho2);
    
    if reprojection_error<tr && depth_proximity<tp
        inlier_list(i)=1;
    else
        % iterate through depth value in the range -tp,tp and see any point
        % there has a reprojection in bounds
        x=round(pt1(:,1));
        y=round(pt1(:,2));
        
       halfN = floor(tr/ 2);
    neighborhoodDepth = depth1(max(1, y-halfN):min(size(depth1, 1), y+halfN), ...
                                  max(1, x-halfN):min(size(depth1, 2), x+halfN));
    % Calculate the minimum and maximum depth values in the neighborhood
    minDepth = min(neighborhoodDepth(:));
    maxDepth= max(neighborhoodDepth(:));

          
            

        for d =minDepth:0.001:maxDepth
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
            reprojection_error=norm(pt2-transformed_pt_2d);
            rho1=transformed_pt1(3);
            rho2=point3D2(3);
            depth_proximity=2*abs(rho1-rho2)/(rho1+rho2);
            if reprojection_error<=tr && depth_proximity<=tp
                inlier_list(i)=1;
            end


        end
    
    end

end
end
