export ROS_MASTER_URI=http://jackal5:11311   # Hostname for Jackal 2
export ROS_IP="$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
