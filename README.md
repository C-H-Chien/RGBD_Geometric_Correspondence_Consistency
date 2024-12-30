# RGBD Geometric Correspondence Consistency (3DV 2025)
### Research @Brown University
This repository hosts the code for the paper "Geometric Correspondence Consistency in RGB-D Relative Pose Estimation" published in 3DV 2025. The geometric correspondence consistency (GCC) states that given one pair of RGBD correspondences, the two images are partitioned into a family of nested curves such that corresponding points must lie on corresponding curves. This constraint acts on the image space (observation space) so that it is immune to depth errors and robust to depth noise.

## Dependencies
* Eigen 3.X
* OpenCV 4.X
* [yaml-cpp](https://github.com/jbeder/yaml-cpp) (used to parse data from the dataset config file)
* glog (optional; if not used, comment macro definitions in ``include/definitions.h``)
* gflags (optional; if not used, comment macro definitions in ``include/definitions.h``)

## Build and Compile
Follow the standard build and compile steps, namely, 
```bash
$ mkdir build && cd build
$ cmake ..
$ make -j
```
You shall find executive files under the ``bin`` folder.

## Usage
Under the ``bin`` folder, do
```bash
./main_VO --config_file=../config/tum.yaml
```
where the configuration file can be customized depending on the dataset in use. See ``.yaml`` files under the ``config`` folder for more information. If no ``--config_file`` input argument is given, the default value will be used which is defined in ``cmd/main_VO.cpp``.

## Paper
Please cite the following paper if you use the code:
```BibTeX
@InProceedings{zheng:etal:WACV:2025,
  title={Geometric Correspondence Consistency in RGB-D Relative Pose Estimation},
  author={Kumar, Sourav and Chien, Chiang-Heng and Kimia, Benjamin},
  booktitle={2025 International Conference on 3D Vision (3DV)},
  pages={},
  year={2025}
}
```

## Contributors
Chiang-Heng Chien* (chiang-heng_chien@brown.edu) <br />
Sourav Kumar (sourav_kumar@brown.edu) <br />
*corresponding author