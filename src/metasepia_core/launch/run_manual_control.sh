#!/bin/bash

# Function to clean up ROS nodes and processes
cleanup() {
    echo "Aggressively terminating ROS2 nodes and processes..."
    
    # Force kill all ROS2 processes
    echo "Force killing all ROS2 processes..."
    pkill -9 -f ros2 || true
    
    # Kill specific node processes by name
    for node in arduino_node controller_node joy_node rosapi rosapi_params rosbridge_websocket
    do
        echo "Killing process: $node"
        pkill -9 -f "$node" || true
    done
    
    # Kill the main launch process if it exists
    if [ ! -z "$LAUNCH_PID" ]; then
        echo "Terminating launch process (PID: $LAUNCH_PID)"
        kill -9 $LAUNCH_PID 2>/dev/null || true
    fi
    
    # Additional cleanup: kill any Python processes related to ROS
    echo "Killing any remaining Python processes related to ROS..."
    pkill -9 -f "python.*ros" || true
    
    echo "Cleanup complete. Verifying..."
    sleep 2  # Give some time for processes to fully terminate
    
    # Check if any ROS2 nodes are still running
    remaining_nodes=$(ros2 node list 2>/dev/null)
    if [ ! -z "$remaining_nodes" ]; then
        echo "Warning: Some nodes are still running:"
        echo "$remaining_nodes"
        echo "You may need to manually terminate these processes."
    else
        echo "All ROS2 nodes have been terminated successfully."
    fi
    
    exit 0
}


source install/setup.sh
# Set up trap to catch Ctrl+C (SIGINT) and termination signal (SIGTERM)
trap cleanup SIGINT SIGTERM

echo "Launching manual mode with additional configuration"

# Launch ROS2 in the background and capture the PID
ros2 launch metasepia_core manual_control_launch.py &
LAUNCH_PID=$!

# Wait for the launch process to start
sleep 20

# Set the parameter using ROS2 command
if ros2 param set /camera/camera .camera.color.image_raw.compressed.jpeg_quality 70; then
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
