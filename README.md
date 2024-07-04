# CUADC

## Introduction

- VITAL
  - ONLY fit for authors' own landing pad style
- find_object_2d
  - ONLY fit for specific height and angle
- YOLO
  - TODO
  

## Installation & Launch

```sh
catkin_make install --source CUADC --build CUADC/build
```

```sh
roslaunch cuadc simu_p450.launch
```


## Release Notes

### [v0.2.6] - 2024-07-04

implement: get global position of landing_pad from pixel frame

### [v0.2.5] - 2024-07-03

implement: detect landing_pad via YOLOv2

### [v0.2.4] - 2024-06-27

implement: detect landing_pad via find_object_2d

### [v0.3.0] - 2024-06-27

implement: autonomous landing via VITAL

### [v0.2.3] - 2024-06-26

implement: detect landing_pad via VITAL

### [v0.2.2] - 2024-06-25

implement: read image from monocular camera

### [v0.2.1] - 2024-06-24

implement: move to task area

### [v0.2.0] - 2024-06-24

implement: Takeoff

### [v0.1.0] - 2024-06-23

implement: OFFBOARD & arming

### [v0.0.1] - 2024-06-23

build pass
