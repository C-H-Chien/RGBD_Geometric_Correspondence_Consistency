close all;
clear;

Num_Of_Curves = 15;
% Define the input grid (in 3D)
Camera1.R = eul2rotm([0, 7.1*pi/8, -pi/4]);
Camera1.T = [2.2; -1; 8.5];
Camera1.K = [400, 0, 320; 0, 400, 240; 0, 0, 1];

Camera2.R = eul2rotm([0, -2*pi/8, -pi/5]);
Camera2.T = [2.86; 3; 6];
Camera2.K = [400, 0, 320; 0, 400, 240; 0, 0, 1];

centres = [0.1 0 0.0;0 -3.5 0];
sphere_radius_values = linspace(0.1, 4, Num_Of_Curves);

colours = 0.5 + 0.5 .* (2*rand([Num_Of_Curves, 3], 'like', 0.5)-1);
% colours = ["g.";"b.";'r.';'c.';'m.';'y.'];
% color_pt = ["g*";"b*"];
resolution=100;
figure;
for i = 1:numel(sphere_radius_values)
    hold on;
    center = centres(1,:)'; % change the centre index to centre=centres(2,:) for second row figures.
    radius = sphere_radius_values(i);
    x_min = center(1) - radius;
    x_max = center(1) + radius;
    y_min = center(2) - radius;
    y_max = center(2) + radius;
    z_min = center(3) - radius;
    z_max = center(3) + radius;
    
    [x3, y3, z3] = meshgrid(linspace(x_min, x_max,resolution), ...
                         linspace(y_min, y_max, resolution), ...
                         linspace(z_min, z_max, resolution));
    
    % Compute the implicitly defined function x^2 + y^2 + z^3 - 0.5^2 = 0
    f1 = (x3-center(1)).^2 + (y3-center(2)).^2 + (z3-center(3)).^2 - radius^2;
    % This surface is z = 2*y - 6*x^3, which can also be expressed as
    
    f2 = 2*y3 - 6*x3.^3 - z3;
    
    % Also compute z = 2*y - 6*x^3 in the 'traditional' way.
    
    [x2, y2] = meshgrid(linspace(x_min, x_max,1000), ...
                         linspace(y_min, y_max, 1000));
    
    z2 = 2*y2 - 6*x2.^3;%1,3,0
    
    % Visualize the two surfaces.
    f3 = f1 - f2;
    
    % Interpolate the difference field on the explicitly defined surface.
    f3s = interp3(x3, y3, z3, f3, x2, y2, z2);
    
    % Find the contour where the difference (on the surface) is zero.
    C = contours(x2, y2, f3s, [0 0]);
    
    % Extract the x- and y-locations from the contour matrix C.
    xL = C(1, 2:end);
    yL = C(2, 2:end);
    
    % Interpolate on the first surface to find z-locations for the intersection
    % line.
    zL = interp2(x2, y2, z2, xL, yL);
    
    % Visualize the line.
    subplot(1,2,1);
    Gamma=[center(1);center(2);center(3);1];
    Intersection_Points3D=[[xL', yL', zL']'; ones(1, size(xL', 1))];
    Transformation_Matrix = [Camera1.R, Camera1.T; 0, 0, 0, 1];
    Transformed_Intersection_Points3D = Transformation_Matrix * Intersection_Points3D;
    Transformed_Intersection_Points3D = Transformed_Intersection_Points3D(1:3, :);
    Projected_Intersection_Points2D = Camera1.K * Transformed_Intersection_Points3D;
    Projected_Intersection_Points2D = Projected_Intersection_Points2D ./ Projected_Intersection_Points2D(3,:);
    plot(Projected_Intersection_Points2D(1,:), Projected_Intersection_Points2D(2,:), 'Color', colours(i,:), 'Marker', '.', 'MarkerSize', 3); hold on;
    
    gamma=Transformation_Matrix*Gamma;
    gamma=gamma(1:3,:);
    gamma=Camera2.K*gamma;
    gamma=gamma./gamma(3);
    plot(gamma(1), gamma(2), 'Color', colours(1,:), 'Marker', '*', 'MarkerSize', 6);
   
    subplot(1,2,2);
    Transformation_Matrix = [Camera2.R, Camera2.T; 0, 0, 0, 1];
    Transformed_Intersection_Points3D = Transformation_Matrix * Intersection_Points3D;
    Transformed_Intersection_Points3D = Transformed_Intersection_Points3D(1:3, :);
    Projected_Intersection_Points2D = Camera2.K * Transformed_Intersection_Points3D;
    Projected_Intersection_Points2D = Projected_Intersection_Points2D ./ Projected_Intersection_Points2D(3,:);
    plot(Projected_Intersection_Points2D(1,:), Projected_Intersection_Points2D(2,:), 'Color', colours(i,:), 'Marker', '.', 'MarkerSize', 3); hold on;
    gamma=Transformation_Matrix*Gamma;
    gamma=gamma(1:3,:);
    gamma=Camera2.K*gamma;
    gamma=gamma./gamma(3);
    plot(gamma(1),gamma(2), 'Color', colours(1,:), 'Marker', '*', 'MarkerSize', 6);
end
set(gcf,'color','w');

pause(1);
subplot(1,2,1);
set(gca,'XTick',[], 'YTick', []);
axis equal;
xlim([1, 640]);
ylim([1, 480]);
% 

subplot(1,2,2);
set(gca,'XTick',[], 'YTick', []);
axis equal;
xlim([1, 640]);
ylim([1, 480]); 

