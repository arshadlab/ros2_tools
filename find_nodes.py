#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Usage:
#    find_nodes.py <keyword|topic|service|action>
#    
#     $ find_nodes.py /follow_path
#        Searching: 37/37 nodes
#        Service Servers
#        ===============
#        Node: /controller_server_rclcpp_node
#            /follow_path [Action] : nav2_msgs/action/FollowPath
#
#        Service Clients
#        ===============
#        Node: /bt_navigator_rclcpp_node
#            /follow_path [Action] : nav2_msgs/action/FollowPath
#
#  Author: Arshad Mehmood (arshadm78@yahoo.com)

import sys
import rclpy
import time
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.node import Node

NODE_NAME='info_node'
class InfoNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)


    def print_info(self, pattern):
    
        # Dictionaries to hold searched informations
        publishers = {}
        subscribers = {}
        servers = {}  # Services and Actions are combined
        clients = {}  # Services and Actions are combined
        
        # Work around to give some time to node to populate. 
        #client = self.create_client(SetParameters, f'{NODE_NAME}/set_parameters') 
        #client.wait_for_service(timeout_sec=3.0)
        
        # Create client approach not giving expected results. Switching back to manual wait
        time.sleep(3)
        
        # Get list of nodes in the system
        node_names = self.get_node_names_and_namespaces()        
        
        for name, namespace in node_names:
            
            # Create fully qualified name if namespace is given
            full_name = namespace + "/" + name if namespace != "/" else name
            #print(full_name)
            # Get list of topics created by this node
            publishers.update(
                self.get_list(
                        full_name, pattern, self.get_publisher_names_and_types_by_node(name, namespace)
                       )
                )
            
            # Get list of topics subscribed by this node
            subscribers.update(
                self.get_list(
                        full_name, pattern, self.get_subscriber_names_and_types_by_node(name, namespace)
                       )
                )

            # Get list of services and actions created by this node           
            servers.update(
                self.get_list(
                        full_name, pattern, self.get_service_names_and_types_by_node(name, namespace)
                       )
                )

            # Get list of services actions utilized by this node             
            clients.update(
                self.get_list(
                        full_name, pattern, self.get_client_names_and_types_by_node(name, namespace)
                       )
                )
          
        # Print gathered info
        
        self.print_results("Topics_Publishers", publishers)
        self.print_results("Topic Subscribers", subscribers)
        self.print_results("Service Servers", servers)
        self.print_results("Service Clients", clients)
                
        
    def print_results(self, header, lst):

        if not lst:
           return
           
        print (header)
        for node, value in lst.items():
            print(f"\tNode: {node}")
            for intr, type in value:
                print(f"\t\t {intr} : {type[0]}")
        print("")
        
    def get_list(self, name, pattern, interface_list):
        exclude_string = ['/transition_event', '/set_parameters_atomically', '/set_parameters', 
                          '/list_parameters', '/get_transition_graph', '/get_parameters',
                          '/get_state', '/get_parameter_types', '/change_state',
                          '/describe_parameters','/get_available_states', '/cancel_goal',
                          '/get_result','/feedback', '/status', '/get_available_transitions',
                          '/rosout', 'parameter_event']
        result = {}
      
        for tup in interface_list:
            if pattern in tup[0]:
                 if not any(s in tup[0] for s in exclude_string):
                        if '_action' in tup[0]:
                                tup_new = (tup[0].replace("/_action/send_goal", " [Action]"),[tup[1][0].replace("_SendGoal","")])
                        else:
                                tup_new = tup
                                
                        if name in result:
                            result[name].append(tup_new)
                        else:
                            result[name] = [tup_new]
        return result

def main(args=None):
    rclpy.init(args=args)

    info_node = InfoNode()
    
    info_node.print_info(pattern=sys.argv[1])

    rclpy.shutdown()

if __name__ == '__main__':
    main()
