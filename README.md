## Introduction
This is an internal collaborative project, Geometry Depth Consistency in RGBD Relative Pose Estimation, from LEMS lab at Brown University. <br />
(To be updated...)

## Contributors
Chiang-Heng Chien (chiang-heng_chien@brown.edu) <br />
Sourav Kumar (sourav_kumar@brown.edu)

## Descriptions
``GT_Constructions``: Ground truth correspondences of TUM-RGBD dataset constructed by the method proposed in our paper. <br />

Various levels of outlier ratios for many image pairs from the TUM-RGBD dataset have been collected in ``Outlier_Ratio_Ranges.mat`` under ``Methods_Profiling/Outlier_Ratios/``. Check ``RANSAC_Iterations_Profiling_Versus_Outlier_Ratio.m`` to see how you can load image pairs of specific range of outlier ratios to do experiment. <br />

To generate the tables of profiling, use ``RANSAC_Iterations_Profiling_Versus_Outlier_Ratio.m``. Change the hyper-parameters organized by a struct ``PARAMS``. 
