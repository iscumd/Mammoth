# Mammoth Simulation

This package contains launch and run information for simulating Mammoth with Gazebo.

## Launching

Use `ros2 launch mammoth_gazebo mammoth.launch.py` for simulation launches that do not need an autonomy system active.

Use `ros2 launch mammoth_gazebo mammoth.launch.py follow_waypoints:=true` for simulation launches that do need an autonomy system active.

After that, it is by default in teleop mode. You are free to joystick around but it will not navigate. To put it into 
autonomous mode, press either the home button or start button on your xbox controller.