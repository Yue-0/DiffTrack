# DiffTrack

## Introduction

__DiffTrack__ is an efficient and robust motion planning framework for differential robots to autonomously track targets.

## Quick Start

In Ubuntu 24 & ROS-jazzy:

```shell
cd DiffTrack && colcon build
```

Launch the simulator:

```shell
source install/setup.zsh
ros2 launch simulator simulation.launch.py
```

Launch the tracker in another terminal:

```shell
cd DiffTrack
source install/setup.zsh
ros2 launch tracker tracking.launch.py
```

Use `2D Goal Pose` to control the target movement, the tracker will track the target autonomously.
