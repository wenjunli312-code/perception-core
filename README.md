# perception-core

A C++ perception core library for autonomous driving, providing detection and fusion algorithms.

## Features

- **YoloDetector** — camera-based 2D object detection using YOLO
- **PointPillarsDetector** — LiDAR-based 3D object detection using PointPillars
- **ObjectFusion** — multi-sensor detection fusion using Eigen
- **LaneDetector** — lane detection from camera input

## Dependencies

- [common-utils](https://github.com/wenjunli312-code/common-utils)
- [sensor-abstraction](https://github.com/wenjunli312-code/sensor-abstraction)
- [Eigen3](https://eigen.tuxfamily.org/)

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --install build
```

## OpenBuilder Demo Project
