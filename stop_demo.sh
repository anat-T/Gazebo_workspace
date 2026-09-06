#!/bin/bash
# Closing just the Gazebo GUI window (the X button) does NOT stop everything
# ros2 launch started - the bridge, belief_region_node, and controller
# spawners keep running in the background as orphans if the launch terminal
# itself wasn't also Ctrl+C'd (or was closed instead). Run this before
# relaunching if you're ever unsure whether a previous run is fully stopped -
# two overlapping instances collide (same ROS_DOMAIN_ID, same node/topic
# names) and cause exactly the kind of "robots don't move" symptom that's
# otherwise very confusing to track down.
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "ros2 launch box_pick_place_demo" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "belief_region_node" 2>/dev/null
pkill -9 -f "search_and_pick_node" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "spawner" 2>/dev/null
sleep 1
remaining=$(pgrep -af "ign gazebo|ros2 launch box_pick_place_demo|parameter_bridge|belief_region_node|search_and_pick_node|robot_state_publisher" 2>/dev/null)
if [ -z "$remaining" ]; then
    echo "Stopped. Clean."
else
    echo "Still running (may need another pass or manual kill -9):"
    echo "$remaining"
fi
