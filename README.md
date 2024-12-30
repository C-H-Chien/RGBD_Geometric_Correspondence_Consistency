# RGBD Geometric Correspondence Consistency (3DV 2025)
### Research @Brown University
This repository hosts the code for the paper "Geometric Correspondence Consistency in RGB-D Relative Pose Estimation" published in 3DV 2025. The geometric correspondence consistency (GCC) states that given one pair of RGBD correspondences, the two images are partitioned into a family of nested curves such that corresponding points must lie on corresponding curves. This constraint acts on the image space (observation space) so that it is immune to depth errors and robust to depth noise.

## Dependencies
* Eigen 3.X
* OpenCV 4.X
* [yaml-cpp](https://github.com/jbeder/yaml-cpp) (used to parse data from the dataset config file)
* glog (optional; if not used, comment the [macro definition](https://github.com/C-H-Chien/RGBD_Geometric_Correspondence_Consistency/blob/c374a0dab33d2630464a5e05031f76e32851afb6/include/definitions.h#L2) in ``include/definitions.h``)
* gflags (optional; if not used, comment the [macro definition](https://github.com/C-H-Chien/RGBD_Geometric_Correspondence_Consistency/blob/c374a0dab33d2630464a5e05031f76e32851afb6/include/definitions.h#L3) in ``include/definitions.h``)

## Build, Compile, and Execute the Code
Follow the standard build and compile steps, namely, 
```bash
$ mkdir build && cd build
$ cmake ..
$ make -j
```
You shall find executive files under the ``bin`` folder. To execute the code, do
```bash
./main_VO --config_file=../config/tum.yaml
```
where the configuration file can be customized depending on the dataset in use. See ``.yaml`` files under the ``config`` folder for more information. If no ``--config_file`` input argument is given, the default value will be used which is defined in ``cmd/main_VO.cpp``. <br />
There is also a ``./test_functions`` executive file which runs the ``test/test_functions.cpp`` and is used to verify the functionality of the C++ implementation. This can be ignored for now.

## Datasets
We use [TUM-RGBD](https://cvg.cit.tum.de/data/datasets/rgbd-dataset), [ICL-NUIM](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html), and [RGBD Scene v2](https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes-v2/) datasets for the experiments. They share consistent data format which is beneficial for data parsing in the code.

## Paper
Please cite the following paper if you use the code:
```BibTeX
@InProceedings{kumar:etal:3DV:2025,
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