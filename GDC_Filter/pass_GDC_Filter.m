function [is_Passed, B_final] = pass_GDC_Filter(PARAMS, anchor_Point3D_View1, picked_MatchedPoints3D_View1, ...
                                     all_Points3D_View2, anchor_Point3D_View2, picked_MatchedPoints2D_Views)
    
    %> Construct a distance transform map centered on the 3D anchor point
    %  in image 2
    dist_X = anchor_Point3D_View2(1) - all_Points3D_View2(:,:,1);
    dist_Y = anchor_Point3D_View2(2) - all_Points3D_View2(:,:,2);
    dist_Z = anchor_Point3D_View2(3) - all_Points3D_View2(:,:,3);
    distTransform = sqrt(dist_X.^2 + dist_Y.^2 + dist_Z.^2);
    
    %> Compute the radius of the anchor point in 3D in image 1 to another 
    %  two 3D points, all under the first camera coordinate, Equation (10).
    radius3D = vecnorm(anchor_Point3D_View1' - picked_MatchedPoints3D_View1');
    
    %> Loop over the two radius computed from the anchor point in View1 to 
    %  another two picked points, check GDC of point1 to 2 and point
    %  1 to 3 
    B_final = cell(2,1);
    is_Passed = zeros(2,1);
    for gi = 1:2
        r = radius3D(gi);
        
        %> Create a binary mask based on the radius criteria
        mask = distTransform < r;
        
        %> Search the boundaries of the binary mask
        B = bwboundaries(mask, "noholes");
       
        boundaryPoints = [];
        for bi = 1:size(B, 1)
            boundaryPoints = [boundaryPoints; [B{bi,1}(:,1), B{bi,1}(:,2)]];
        end
        B_final{gi,1} = boundaryPoints;
        
        x2 = picked_MatchedPoints2D_Views(gi,1);
        y2 = picked_MatchedPoints2D_Views(gi,2);

    %     if check_on_boundary
        bool_check = vecnorm(boundaryPoints' - [y2, x2]') < PARAMS.POINT_TO_CURVE_DIST_THRESH;
    %         bool_check = (abs(B_final(:,2) - x1) < 2) & (abs(B_final(:,1) - y1) < 2);
            check_ = find(bool_check == 1);
            if isempty(check_)
                is_Passed(gi) = 0;
            else
                is_Passed(gi) = 1;
            end
    end
            
 
%             if isempty(check_)
%                 radius_ = sqrt((x1 - B_final(:,2)).^2 + (y1 - B_final(:,1)).^2);
%                 [min_radius_, idx] = mink(radius_, 5);
% 
%                 for ii = 1:size(idx, 1)
%                     xb = B_final(idx(ii),2);
%                     yb = B_final(idx(ii),1);
%                     xyd = sqrt((x1-xb).^2+(y1-yb).^2);
%                     D = linspace(0, round(xyd), 100);
%                     x = xb + D .* (x1-xb)./xyd;
%                     y = yb + D .* (y1-yb)./xyd;
%                     mask(round(y), round(x)) = 1;
%                 end
% 
%                 B = bwboundaries(mask);
% 
%                 max_b_points = 0;
%                 max_b_idx = 0;
%                 for i = 1:size(B,1)
%                     if size(size(B{i}), 1) > max_b_points
%                         max_b_points = size(size(B{i}), 1);
%                         max_b_idx = i;
%                     end
%                 end
% 
%                 B_final = B{max_b_idx};
%                 boundaryPoints = [B_final(:,2), B_final(:,1)];
%             end
%     %     end
%     end

    
end