
[image1]: ./final-project-ros-graph-v2.png
[image2]: ./RetinaNet.png
[image3]: ./TrafficLightDetection.png
[image4]: ./PathPlanning.png

# Team BTS

## Sample Video
[![alt text](https://img.youtube.com/vi/-qO-iFIxGf0/0.jpg)](https://youtu.be/-qO-iFIxGf0)

## Team Members
| Name                    | Email                     | Slack ID    | Role |
| :--                     | :--                       | :--         |:--     |
| Jaeyoung Lee (Lead)     | jylee0121@gmail.com       | @stormboy   | Path Planning / Control |
| Toshiya Matsuda         | tmresolution59@gmail.com  | @t-matsuda  | Traffic Light Detection Data  |
| Dmitry Kudinov          | d.kudinov@gmail.com       | @dkudinov   | Traffic Light Detection Algorithm |
| Zeyu Zhang              | hellozeyu@gmail.com       | @hellozeyu  | System Integration |

## Install Guide

1. Clone git with option `--recursive`
```
$ git clone --recursive <this repository url>
```
2. Install Keras 2.1.3
```
$ pip install keras==2.1.3
```
3. [Download the trained model `d00_9_csv_34.h5` here](https://www.dropbox.com/s/xpifcgnw0lcd5ce/d00_9_csv_34.h5?dl=0)
  * Move this file into `keras-retinanet/snapshots`

## System Overview

![alt text][image1]

### Directory Summary
```
.
├── data
├── imgs
├── keras-retinanet         // <-- added for Traffic Light Detection
│   ├── examples
│   ├── images
│   ├── keras_resnet
│   ├── keras_retinanet
│   ├── snapshots
│   └── tests
├── linux_sim
│   └── linux_sys_int
└── ros
    ├── build
        ├── devel
            ├── launch
                └── src
```

## Traffic Light Detection
### Main Detection Algorithm: **RetinaNet**
* We applied one of the latest and high performance object detection algorithm, **RetinaNet**
  ![alt text][image2]

* Implementation is [Here: Keras-RetinaNet Git Repository](https://github.com/dmitrykudinov/keras-retinanet/)
* Sample Result
  ![alt text][image3]

* References
  * Original Paper
    * [Focal Loss for Dense Object Detection](https://arxiv.org/abs/1708.02002)
  * Reference Source Code
    * [Keras RetinaNet](https://github.com/fizyr/keras-retinanet)

## Path Planning

* Simplified Linear Accel/Decel
![alt text][image4]

## Control

* Manually Tuned PID Control for both of Steering and Throttling.

| Type           | K_P  | K_I     | K_D |
| :--            | :--  | :--     |:--  |
| Throttling     | 0.3  | 0.005   | 0.1 |
| Steering       | 0.7  | 0.0     | 0.1 |


