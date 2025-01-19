# otomo_control
Contains config and launch files for ros2_control on otomo.  Also contains the URDF description

# how to launch mapping

1. launch sim or lidar driver
```bash
ros2 launch otomo_sim sim.launch.py world:=/your/world/here.sdf

# OR

ros2 launch otomo_core rplidar.launch.py
```

2. launch slam_toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py # (with use_sim_time:=true for sim)
```

3. drive around to do the mapping

4. launch rviz with the slam toolbox plugin, save the map with that

# how to launch navigation
1. launch sim or lidar driver
```bash
ros2 launch otomo_sim sim.launch.py world:=/your/world/here.sdf

# OR

ros2 launch otomo_core rplidar.launch.py
```

2. launch rviz using the nav2 config
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz                                                                                                     
```

3. launch nav2_bringup
```bash
ros2 launch nav2_bringup bringup_launch.py params_file:=/path/to/ws/install/otomo_control/share/otomo_control/config/nav2_jazzy_params.yaml map:=/path/to/map.yaml #use_sim_time:=True
```

4. set the initial pose based on real life or sim

5. set the goal marker using nav2 goal

# guide to navigate while mapping
[link to nav2 docs here](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
