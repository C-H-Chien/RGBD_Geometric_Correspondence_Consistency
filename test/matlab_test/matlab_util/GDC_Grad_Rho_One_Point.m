
function dist = GDC_Grad_Rho_One_Point(fx, fy, xm, ym, rows, cols, ...
                              rho, gammas, match_points2D_v2, match_points3D_v2, match_points3D_v1, ...
                              all_Points3D_View2, window_length, anchor_Indx, picked_Indx, ...
                              grad_Rho_xi, grad_Rho_eta)


    %> Set the anchor match
    rho0 = rho(anchor_Indx);
    gamma0 = gammas(:,anchor_Indx);

    p = picked_Indx;
    x0 = match_points2D_v2(p,1);
    y0 = match_points2D_v2(p,2);

    %> Generate Phi Map on a small patch of the match point
    Phi_Map = zeros(window_length,window_length);
    xm_0 = round(x0) + xm;
    ym_0 = round(y0) + ym;
    for ri = 1:window_length
        for ci = 1:window_length

           r = ym_0(ri,ci);
           c = xm_0(ri,ci);
           if r >= 1 && r <= rows && c >= 1 && c <= cols
            Phi_Map(ri,ci) = norm(match_points3D_v2(anchor_Indx,:) - [all_Points3D_View2(r,c,1), all_Points3D_View2(r,c,2), all_Points3D_View2(r,c,3)]);
           end
        end
    end

    %> Get Phi(x0,y0)
    x0_ = x0 - round(x0);
    y0_ = y0 - round(y0);
    Phi_x0y0 = interp2(xm, ym, Phi_Map, x0_, y0_);
    %Phi_x0y0 = norm(match_points3D_v2(anchor_Indx,:) - match_points3D_v2(p,:));


    %> Get gradient Phi
    grad_Phi_xi  = 2*(rho(p) * norm(gammas(:,p))^2 + rho0 * gammas(:,p)' * gamma0)*grad_Rho_xi(p)  + 2*rho(p)*((1/fx).*(rho(p)*gammas(1,p) - rho0*gamma0(1)));
    grad_Phi_eta = 2*(rho(p) * norm(gammas(:,p))^2 + rho0 * gammas(:,p)' * gamma0)*grad_Rho_eta(p) + 2*rho(p)*((1/fy).*(rho(p)*gammas(2,p) - rho0*gamma0(2)));
    grad_Phi     = sqrt(grad_Phi_xi^2 + grad_Phi_eta^2);

    %> Get Phi1
    Phi1 = norm(match_points3D_v1(anchor_Indx,:)' - match_points3D_v1(p,:)');

    %> Now, compute the distance
    dist = abs(Phi1' - Phi_x0y0) ./ grad_Phi;

end
