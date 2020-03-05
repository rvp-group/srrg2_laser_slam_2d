# SRRG2-LASER-SLAM-2D
This repository contains a single Catkin package:

1. [`srrg2_laser_slam_2d`](srrg2_laser_slam_2d): a 2D LiDAR-based SLAM pipeline; supports multiple rangefinders and wheel odometry (at the moment).

This package lives inside the `srrg2` multiverse. It builds on top of our multi-cue SLAM architecture reported in [`srrg2_slam_interfaces`](https://github.com/srrg-sapienza/srrg2_slam_interfaces).

To know how to build and to use our pipeline, please refer to the [package readme](srrg2_laser_slam_2d/README.md).

## Publications
To have a more detailed overview on this solver, you can read our new [preprint](https://arxiv.org/abs/2003.00754).
If you use our code, please cite us in your work.
```bibtex
@misc{colosi2020plugandplay,
    title={Plug-and-Play SLAM: A Unified SLAM Architecture for Modularity and Ease of Use},
    author={Mirco Colosi and Irvin Aloise and Tiziano Guadagnino and Dominik Schlegel and Bartolomeo Della Corte and Kai O. Arras and Giorgio Grisetti},
    year={2020},
    eprint={2003.00754},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

## Contributors
* Mirco Colosi
* Irvin Aloise
* Dominik Schlegel
* Giorgio Grisetti
* Bartolomeo Della Corte
* Tiziano Guadagnino

## License
BSD 3.0
