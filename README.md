## How to run
1. Clone this repo to ROS2 workspace
2. Build using command `colcon build --packages-select particle_filter`
3. Source `source install/local_setup.bash`
4. Run rviz with prepared configuration `rviz2 -d view.rviz` (`view.rviz` in root dir)
5. Run node `ros2 run particle_filter pf_node`
