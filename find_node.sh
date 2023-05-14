#!/bin/bash

# Usage:
#    find_node.sh <keyword|topic|service|action>
#    
#     $ find_node.sh /follow_path
#        Searching: 37/37 nodes
#        Action Server
#        =============
#        Node: /controller_server_rclcpp_node
#            /follow_path: nav2_msgs/action/FollowPath
#
#        Action Clients
#        ==============
#        Node: /bt_navigator_rclcpp_node
#            /follow_path: nav2_msgs/action/FollowPath
#

if [ $# -eq 0 ]; then
  echo "Please provide the service name as an argument."
  exit 1
fi

interface_name=$1

# Get a list of active nodes using ros2 node list
active_nodes=$(ros2 node list)
total_found=$(echo "$active_nodes" | wc -l) 

Topics_Publisher=""
Topics_Subscribers=""
Service_Servers=""
Service_Clients=""
Action_Servers=""
Action_Clients=""

current=1
# Iterate through each active node
for node in $active_nodes; do

  # Display Progress
  echo -ne "\rSearching: $current/$total_found nodes"
 
  # Get Node Info  
  interfaces=$(ros2 node info $node)

  # Get list of filtered topics publish by this node
  interface=$(echo -e "$interfaces" | grep "Publishers" -A 1000 | grep "Service Servers:" -B 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Topics_Publisher+="Node: $node\n$interface\n"
  fi
  
  # Get list of filtered topics subscribed by this node
  interface=$(echo -e "$interfaces" | grep "Subscribers" -A 1000 | grep "Publishers" -B 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Topics_Subscribers+="Node: $node\n$interface\n"
  fi

  # Get list of filtered services offered by this node
  interface=$(echo -e "$interfaces"  | grep "Service Servers:" -A 1000 | grep "Service Clients:" -B 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Service_Servers+="Node: $node\n$interface\n"
  fi
  
  # Get list of filtered services this node is client for.
  interface=$(echo -e "$interfaces"  | grep "Service Clients:" -A 1000 | grep "Action Servers" -B 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Service_Clients+="Node: $node\n$interface\n"
  fi
  
  # Get list of filtered action services offered by this node
  interface=$(echo -e "$interfaces"  | grep "Action Servers:" -A 1000 | grep "Action Clients:" -B 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Action_Servers+="Node: $node\n$interface\n"
  fi
  
  # Get list of filtered actions this node is using
  interface=$(echo -e "$interfaces" | grep "Action Clients:" -A 1000 | grep $interface_name)
  if [[ -n "$interface" ]]; then
        Action_Clients+="Node: $node\n$interface\n"
  fi
  
  ((current++))
done

if [[ -n "$Topics_Publisher" ]]; then
        echo -e "\nTopic Publishers\n================"
        echo -e "$Topics_Publisher"
fi

if [[ -n "$Topics_Subscribers" ]]; then
        echo -e "\nTopic Subscribers\n================="
        echo -e "$Topics_Subscribers"
fi

if [[ -n "$Service_Servers" ]]; then
        echo -e "\nService Server\n=============="
        echo -e "$Service_Servers"
fi

if [[ -n "$Service_Clients" ]]; then
        echo -e "\nService Clients\n==============="
        echo -e "$Service_Clients"
fi

if [[ -n "$Action_Servers" ]]; then
        echo -e "\nAction Server\n============="
        echo -e "$Action_Servers"
fi

if [[ -n "$Action_Clients" ]]; then
        echo -e "\nAction Clients\n=============="
        echo -e "$Action_Clients"
fi
