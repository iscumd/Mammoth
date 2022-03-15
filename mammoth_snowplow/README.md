# Mammoth Robot

This package contains launch and run information for launching Mammoth in Real Life.

## Launching

Use `ros2 launch mammoth_gazebo mammoth.launch.py` launches that do not need an autonomy system active.

Use `ros2 launch mammoth_gazebo mammoth.launch.py follow_waypoints:=true` launches that do need an autonomy system active.