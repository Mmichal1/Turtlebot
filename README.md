# MMRS TurtleBot3 simulation in docker container

```bash
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
```

Even though that the model path is sourced on image build, gazebo doesn't acknowledge that and tries to download models form the internet which hangs and gazebo doesn't load properly. If it doesn't load, then you need to source it again:

```bash
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ros/turtlebot3/turtlebot_mmrs/models
```
