# my_action_server

This example illustrates how to design an action server and an action client pair.

The action server "my_action_server" is based on wsnewman's "simple" action server.  This example is documented in the document "Introduction to action servers and clients:  designing your own action server."

## Example usage
Run the following commands to use the STDR simulator in wsnewman's learning_ros
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch` to start STDR,
`rosrun my_action_server my_lidar_alarm` to start the lidar alarm,
`rosrun my_action_server my_action_server` to start the server, and:
`rosrun my_action_server my_action_client` to start the client.


