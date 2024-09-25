
echo "Launching manual mode with additional configuration"

# Launch ROS2 in the background and capture the PID
ros2 launch metasepia_core manual_control_launch.py &
LAUNCH_PID=$!

# Wait for the launch process to start
sleep 5 

# Set the parameter using ROS2 command
ros2 param set /camera/camera .camera.color.image_raw.compressed.jpeg_quality 2 && echo "Setting JPEG quality"

# Output the PID of the background launch process
echo "Manual control launch PID: $LAUNCH_PID"