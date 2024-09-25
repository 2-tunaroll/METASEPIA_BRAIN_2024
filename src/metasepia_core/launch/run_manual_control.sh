#!/bin/bash

# Function to clean up ROS nodes and processes
cleanup() {
    echo "Terminating ROS2 nodes and processes..."
    
    # Kill all ROS2 nodes
    ros2 node list | while read node; do
        echo "Killing node: $node"
        ros2 lifecycle set $node shutdown || true
        ros2 service call $node/destroy_node std_srvs/srv/Empty {} || true
    done

    # Kill the main launch process
    if [ ! -z "$LAUNCH_PID" ]; then
        echo "Terminating launch process (PID: $LAUNCH_PID)"
        kill $LAUNCH_PID || true
    fi

    # Kill any remaining ROS2 processes
    pkill -f ros2 || true
    
    echo "Cleanup complete."
    exit 0
}

# Set up trap to catch Ctrl+C (SIGINT) and termination signal (SIGTERM)
trap cleanup SIGINT SIGTERM

echo "Launching manual mode with additional configuration"

# Launch ROS2 in the background and capture the PID
ros2 launch metasepia_core manual_control_launch.py &
LAUNCH_PID=$!

# Wait for the launch process to start
sleep 10

# Set the parameter using ROS2 command
if ros2 param set /camera/camera camera.color.image_raw.compressed.jpeg_quality 2; then
    echo "Setting JPEG quality successful"
else
    echo "Failed to set JPEG quality parameter"
    exit 1
fi

# Output the PID of the background launch process
echo "Manual control launch PID: $LAUNCH_PID"
echo "Press Ctrl+C to terminate the script and all ROS2 nodes/processes"

# Wait indefinitely, allowing for manual Ctrl+C termination
while true; do
    sleep 1
done
