#!/bin/bash

if [ "$1" = "4d" ]; then
	gnome-terminal -- bash -c "cd ../.. && source install/setup.bash && ros2 launch lunar_rover lunar.launch.py slam:=True; exec bash"
	gnome-terminal -- bash -c "python3 avoidance_action_server.py; exec bash"
	gnome-terminal -- bash -c "python3 4d_avoidance_pub.py; exec bash"
	#sleep 30
	#ros2 action send_goal avoidance lunar_rover/action/Avoidance "{ \"command\": \"start\" }"
elif [ "$1" = "3d" ]; then
	gnome-terminal -- bash -c "cd ../.. && ros2 launch lunar_rover lunar.launch.py slam:=True; exec bash"
	gnome-terminal -- bash -c "python3 avoidance_action_server.py; exec bash"
	gnome-terminal -- bash -c "python3 3d_avoidance_pub.py; exec bash"
	#sleep 30
	#ros2 action send_goal avoidance lunar_rover/action/Avoidance "{ \"command\": \"start\" }"
else
    echo "Invalid argument. Usage: ./start.sh <4d|3d>"
fi
