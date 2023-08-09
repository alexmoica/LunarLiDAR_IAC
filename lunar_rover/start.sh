#!/bin/bash

# Function to clean up and terminate child processes
cleanup() {
    echo "Cleaning up and terminating child processes..."
    pkill -f "ros2 launch lunar_rover lunar.launch.py slam:=True"
    pkill -f "python3 avoidance_action_server.py"
    pkill -f "python3 4d_avoidance_pub.py"
    pkill -f "python3 3d_avoidance_pub.py"
    pkill -f "python3 avoidance_sub.py"
    exit
}

# Set up a trap to catch Ctrl+C and execute the cleanup function
trap cleanup INT

if [ "$1" = "4d" ]; then
    gnome-terminal -- bash -c "cd ../.. && source install/setup.bash && ros2 launch lunar_rover lunar.launch.py slam:=True; exec bash" &
    gnome-terminal -- bash -c "python3 avoidance_action_server.py; exec bash" &
    gnome-terminal -- bash -c "python3 4d_avoidance_pub.py; exec bash" &
    gnome-terminal -- bash -c "python3 avoidance_sub.py; exec bash" &
    
    # Wait for Ctrl+C to be pressed
    while true; do
        sleep 1
    done
elif [ "$1" = "3d" ]; then
    gnome-terminal -- bash -c "cd ../.. && ros2 launch lunar_rover lunar.launch.py slam:=True; exec bash" &
    gnome-terminal -- bash -c "python3 avoidance_action_server.py; exec bash" &
    gnome-terminal -- bash -c "python3 3d_avoidance_pub.py; exec bash" &
    gnome-terminal -- bash -c "python3 avoidance_sub.py; exec bash" &
    
    # Wait for Ctrl+C to be pressed
    while true; do
        sleep 1
    done
else
    echo "Invalid argument. Usage: ./start.sh <4d|3d>"
fi

