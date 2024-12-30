#ifndef MOTION_TRACKER_CPP
#define MOTION_TRACKER_CPP

#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "MotionTracker.h"
#include "definitions.h"

// ============================================================================================================================
// class MotionTracker: track camera motion, i.e., estimate camera poses, similar to "tracking" used in ORB-SLAM or OpenVSLAM,
//                      but the name aims to differentiate "camera motion tracks" from "feature tracks".
//
// ChangeLogs
//    Chien  24-01-17    Initially created.
//
//> (c) LEMS, Brown University
//> Chiang-Heng Chien (chiang-heng_chien@brown.edu)
// ============================================================================================================================

MotionTracker::MotionTracker() {
    Estimated_Rel_Rot = Eigen::Matrix3d::Identity();
    Estimated_Rel_Transl << 0.0, 0.0, 0.0;
    Final_Rel_Rot = Eigen::Matrix3d::Identity();
    Final_Rel_Transl << 0.0, 0.0, 0.0;
    Final_Num_Of_Inlier_Support = 0;
}

void MotionTracker::get_Relative_Pose_from_RANSAC( 
    Frame::Ptr Curr_Frame, Frame::Ptr Prev_Frame, int Num_Of_Good_Feature_Matches, bool use_GCC_filter ) 
{
    //> Reset
    int Max_Num_of_Inlier_Support = 0;
    int Num_Of_Inlier_Support = 0;

    //> RANSAC loop
    for (unsigned iter = 0; iter < RANSAC_NUM_OF_ITERATIONS; iter++) {
        
        //> 1) Hypothesis formulation
        int Sample_Indices[3] = {0, 0, 0};
        do {
            //> Randomly pick 3 points from Num_Of_Good_Feature_Matches passed by Pipeline.cpp
            for (int i = 0; i < 3; i++) {
                Sample_Indices[i] = Uniform_Random_Number_Generator< int >(0, Num_Of_Good_Feature_Matches-1);
            }
        }
        while ( (Sample_Indices[0] == Sample_Indices[1]) && (Sample_Indices[0] == Sample_Indices[2]) && (Sample_Indices[1] == Sample_Indices[2]) );

        if (use_GCC_filter) {
            double gcc_dist_forward, gcc_dist_backward;
           
            //> GCC 1st round: make Sample_Indices[0] as the anchor index and the *second* as the picked index
            gcc_dist_forward  = get_GCC_dist( Curr_Frame, Prev_Frame, Sample_Indices[0], Sample_Indices[1] );
            gcc_dist_backward = get_GCC_dist( Prev_Frame, Curr_Frame, Sample_Indices[0], Sample_Indices[1] );

            if (gcc_dist_forward < GCC_2D_THRESH && gcc_dist_backward < GCC_2D_THRESH) {
                //> GCC 2nd round: make Sample_Indices[0] as the anchor index and the *third* as the picked index
                gcc_dist_forward  = get_GCC_dist( Curr_Frame, Prev_Frame, Sample_Indices[0], Sample_Indices[2] );
                gcc_dist_backward = get_GCC_dist( Prev_Frame, Curr_Frame, Sample_Indices[0], Sample_Indices[2] );

                //> When anchor point to the second and third picked points both satisfy GCC constraint, go to hypothesis support measurement
                if (gcc_dist_forward < GCC_2D_THRESH && gcc_dist_backward < GCC_2D_THRESH) {}
                else {
                    continue;
                }
            }
            else {
                continue;
            }
        }

        //> Estimate relative rotation (Rel_Rot) and translation (Rel_Transl) by aligning two point clouds
        get_Relative_Pose_by_Three_Points_Alignment( Curr_Frame, Prev_Frame, Sample_Indices );
        
        //> 2) Hypothesis support measurement
        Num_Of_Inlier_Support = get_Hypothesis_Support_Reproject_from_3D_Points( Curr_Frame, Prev_Frame );
        if (Num_Of_Inlier_Support > Max_Num_of_Inlier_Support) {
            Final_Rel_Rot               = Estimated_Rel_Rot;
            Final_Rel_Transl            = Estimated_Rel_Transl;
            Final_Num_Of_Inlier_Support = Num_Of_Inlier_Support;
            Max_Num_of_Inlier_Support   = Num_Of_Inlier_Support;
        }
    }
}

double MotionTracker::get_GCC_dist( Frame::Ptr Curr_Frame, Frame::Ptr Prev_Frame, int anchor_index, int picked_index ) {

    //> View 1 is previous frame, view 2 is current frame
    double phi_view1 = (Prev_Frame->Gamma[ anchor_index ] - Prev_Frame->Gamma[ picked_index ]).norm();
    double phi_view2 = (Curr_Frame->Gamma[ anchor_index ] - Curr_Frame->Gamma[ picked_index ]).norm();
    Eigen::Vector3d gamma_view2   = Prev_Frame->inv_K * Curr_Frame->SIFT_Match_Locations_Pixels[ picked_index ];
    Eigen::Vector3d gamma_0_view2 = Prev_Frame->inv_K * Curr_Frame->SIFT_Match_Locations_Pixels[ anchor_index ];

    double rho_0 = (Curr_Frame->Gamma[ anchor_index ])(2);
    double rho_p = (Curr_Frame->Gamma[ picked_index ])(2);

    double gradient_phi_xi  = 2*(rho_p*(gamma_view2.norm()*gamma_view2.norm()) + rho_0*(gamma_view2.dot(gamma_0_view2))) * Curr_Frame->gradient_Depth_at_Features[ picked_index ].first \
                            + 2*rho_p*( (1.0/Curr_Frame->K(0,0))*(rho_p*gamma_view2(0) - rho_0*gamma_0_view2(0)) );
    double gradient_phi_eta = 2*(rho_p*(gamma_view2.norm()*gamma_view2.norm()) + rho_0*(gamma_view2.dot(gamma_0_view2))) * Curr_Frame->gradient_Depth_at_Features[ picked_index ].second \
                            + 2*rho_p*( (1.0/Curr_Frame->K(1,1))*(rho_p*gamma_view2(1) - rho_0*gamma_0_view2(1)) );

    double gradient_phi = sqrt(gradient_phi_xi*gradient_phi_xi + gradient_phi_eta*gradient_phi_eta);
    return fabs(phi_view1 - phi_view2) / gradient_phi;
}

void MotionTracker::get_Relative_Pose_by_Three_Points_Alignment( Frame::Ptr Curr_Frame, Frame::Ptr Prev_Frame, int Sample_Indices[3] ) {
    
    std::vector<Eigen::Vector3d> Prev_Frame_Gammas;
    std::vector<Eigen::Vector3d> Curr_Frame_Gammas;
    Eigen::Matrix3d Prev_Frame_Shifted_Gammas;
    Eigen::Matrix3d Curr_Frame_Shifted_Gammas;
    Prev_Frame_Gammas.push_back( Prev_Frame->Gamma[ Sample_Indices[0] ] );
    Prev_Frame_Gammas.push_back( Prev_Frame->Gamma[ Sample_Indices[1] ] );
    Prev_Frame_Gammas.push_back( Prev_Frame->Gamma[ Sample_Indices[2] ] );
    Curr_Frame_Gammas.push_back( Curr_Frame->Gamma[ Sample_Indices[0] ] );
    Curr_Frame_Gammas.push_back( Curr_Frame->Gamma[ Sample_Indices[1] ] );
    Curr_Frame_Gammas.push_back( Curr_Frame->Gamma[ Sample_Indices[2] ] );
    Eigen::Vector3d Centroid_Prev = { (Prev_Frame_Gammas[0](0) + Prev_Frame_Gammas[1](0) + Prev_Frame_Gammas[2](0))/(double)(3), \
                                      (Prev_Frame_Gammas[0](1) + Prev_Frame_Gammas[1](1) + Prev_Frame_Gammas[2](1))/(double)(3), \
                                      (Prev_Frame_Gammas[0](2) + Prev_Frame_Gammas[1](2) + Prev_Frame_Gammas[2](2))/(double)(3)
                                    };
    Eigen::Vector3d Centroid_Curr = { (Curr_Frame_Gammas[0](0) + Curr_Frame_Gammas[1](0) + Curr_Frame_Gammas[2](0))/(double)(3), \
                                      (Curr_Frame_Gammas[0](1) + Curr_Frame_Gammas[1](1) + Curr_Frame_Gammas[2](1))/(double)(3), \
                                      (Curr_Frame_Gammas[0](2) + Curr_Frame_Gammas[1](2) + Curr_Frame_Gammas[2](2))/(double)(3)
                                    };
    //> Shift the 3D point Gammas by the centroid point
    for (int i = 0; i < 3; i++) {
        Prev_Frame_Shifted_Gammas.row(i) = Prev_Frame_Gammas[i] - Centroid_Prev;
        Curr_Frame_Shifted_Gammas.row(i) = Curr_Frame_Gammas[i] - Centroid_Curr;
    }

    //> Compute covariance matrix
    Eigen::Matrix3d Cov_Matrix = Prev_Frame_Shifted_Gammas.transpose() * Curr_Frame_Shifted_Gammas;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd( Cov_Matrix, Eigen::ComputeFullU | Eigen::ComputeFullV );

    Eigen::Matrix3d test = svd.matrixV() * svd.matrixU().transpose();
    Estimated_Rel_Rot = svd.matrixV() * svd.matrixU().transpose();

    if (Estimated_Rel_Rot.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V(0,2) *= -1.0;
        V(1,2) *= -1.0;
        V(2,2) *= -1.0;
        Estimated_Rel_Rot = V * svd.matrixU().transpose();
    }
    Estimated_Rel_Transl = Centroid_Curr - Estimated_Rel_Rot * Centroid_Prev;

    // std::cout << "Estimated relative rotation:" << std::endl;
    // std::cout << Estimated_Rel_Rot << std::endl;
    // std::cout << "Estimated relative translation:" << std::endl;
    // std::cout << Estimated_Rel_Transl << std::endl;
}

int MotionTracker::get_Hypothesis_Support_Reproject_from_3D_Points( Frame::Ptr Curr_Frame, Frame::Ptr Prev_Frame ) {
    
    //> transform from the previous frame to the current frame and count the number of inliers
    int Num_Of_Inlier_Support = 0;
    for (size_t i = 0; i < Prev_Frame->Gamma.size(); i++) {
        Eigen::Vector3d Transf_Gamma = Estimated_Rel_Rot * Prev_Frame->Gamma[i] + Estimated_Rel_Transl;
        Transf_Gamma(0) /= Transf_Gamma(2);
        Transf_Gamma(1) /= Transf_Gamma(2);
        Eigen::Vector3d Projected_Point = {Transf_Gamma(0)*Curr_Frame->K(0,0) + Curr_Frame->K(0,2), Transf_Gamma(1)*Curr_Frame->K(1,1) + Curr_Frame->K(1,2), 1.0};
        
        //> Reprojection error
        double Reproj_Err = (Projected_Point - Curr_Frame->SIFT_Match_Locations_Pixels[i]).norm();

        //> Measure inlier support
        if ( Reproj_Err < REPROJ_ERROR_THRESH )
            Num_Of_Inlier_Support++;
    }
    return Num_Of_Inlier_Support;
}

void get_Hypothesis_Support_Epipolar_Constraint( Frame::Ptr Curr_Frame, Frame::Ptr Prev_Frame ) {
    
}

#endif