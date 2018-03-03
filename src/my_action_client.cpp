// example_action_client: 
// wsn, October, 2014
// this is modified for ps4
// Justin Ferri

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../my_action_server/action/demo.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (demo) and appended name (Action)
// If you write a new client of the server in this package, you will need to include my_action_server in your package.xml,
// and include the header file below
#include<my_action_server/demoAction.h>
#include<std_msgs/Bool.h>

bool g_is_obstacle; //whether or not an obstance needs to be avoided
bool g_goal_done; //whether or not there is an active goal to be finished


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //int diff = result->output - result->goal_stamp;
    g_goal_done = true;
    //ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d", result->output, result->goal_stamp, diff);
}

void activeCb() {
    ROS_INFO("Goal is currently active");
}

//listen to lidar to see if there's an obsticle in the way
void lidaralarmCb(const std_msgs::Bool obstacle_detected){
    if(obstacle_detected.data == true){
	g_is_obstacle = true;
    } else {
	g_is_obstacle = false;
    }
}
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_client"); // name this node 
    
    // here is a "goal" object compatible with the server, as defined in my_action_server/action
    my_action_server::demoGoal goal;

    // use the name of our server, which is: my_action (named in my_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<my_action_server::demoAction> action_client("my_action", true);
    
    ros::NodeHandle nh;
    //create subscriber that subscribes to the lidar alarm
    ros::Subscriber lidar_alarm = nh.subscribe("/lidar_alarm", 1, lidaralarmCb);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    g_is_obstacle = false;
    g_goal_done = false;

    bool move = true;

    //these should all be the same size can be changed for different goals (multiple poses)
    float x_v[] = {3, 0, 3.5, 0, -3, 0, 3};
    float y_v[] = {0, 3, 0, 2.25, 0, 7, 0};
    float angle_v[] = {3.14/2, -3.14/2, 3.14/2, 3.14/2, 0, -3.14/2, 3.14/2};

    //if an obstacle is struck these values can be changed for different interactions (def is default)
    float x_v_def[] = {0};
    float y_v_def[] = {0};
    float angle_v_def[] = {3.14/4};

    while (true) {
	if(move == true){
	    goal.x_vals.resize(sizeof(x_v));
	    goal.y_vals.resize(sizeof(x_v));
	    goal.angle_vals.resize(sizeof(x_v));
	    //copy the values from the array over to the goal to be sent
	    for (int i = 0; i < sizeof(x_v); i++){
		goal.x_vals[i] = x_v[i];
		goal.y_vals[i] = y_v[i];
		goal.angle_vals[i] = angle_v[i];
 	    }
	    //ROS_INFO("HIT C4");
	    action_client.sendGoal(goal, &doneCb, &activeCb);

	    while(g_goal_done == false && g_is_obstacle == false){
		ros::spinOnce();
	    }
	    
	    if(g_is_obstacle == true){
		move=false;
	    }

	    g_goal_done = false;
	    g_is_obstacle = false;

	} else if (move == false) {
	    action_client.cancelAllGoals();
	    goal.x_vals.resize(sizeof(x_v_def));
	    goal.y_vals.resize(sizeof(x_v_def));
	    goal.angle_vals.resize(sizeof(x_v_def));
	    ROS_INFO("THERE IS AN OBJECT");
	    //copy the values from the array over to the goal to be sent
	    for (int i = 0; i < 1; i++){
		goal.x_vals[i] = x_v_def[i];
		goal.y_vals[i] = y_v_def[i];
		goal.angle_vals[i] = angle_v_def[i];
 	    }
	    action_client.sendGoal(goal, &doneCb);
	    move = false;
	    g_goal_done = false;
	    g_is_obstacle = false;
	}
    }

    return 0;
}

