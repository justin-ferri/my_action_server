// my_action_server: a simple action server
// Wyatt Newman
// this is modified for ps4
// Justin Ferri

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../my_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<my_action_server/demoAction.h>

//new imports Pose for messaging position and orientation and Twist for velocity
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

//import for bool
#include <std_msgs/Bool.h>

//constants (global) for how fast the robot should move 1.0 m/s forward and .5236 rad/s (~30 deg/s)
const double g_move_speed = 1.0;
const double g_spin_speed = .5236;
const double g_sample_dt = .01;

//global for velocity cmd
geometry_msgs::Twist g_twist_cmd;

//publisher object to tell the robot how to move
ros::Publisher g_twist_commander;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in my_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<my_action_server::demoAction> as_;
    
    // here it's time to define a 
    // here are some message types to communicate with our client(s)
    my_action_server::demoGoal goal_; // goal message, received from client
    my_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::demoFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client


public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_action_server::demoAction>::GoalConstPtr& goal);
    void do_spin(double angle);
    void do_move(float x_goal, float y_goal);
    double sgn(double val);
    void do_halt();
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "my_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "my_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

double distanceFromOrigin(float x_goal, float y_goal){
    return fabs(pow(pow(x_goal, 2) + pow(y_goal, 2), 0.5));
}

//use the distance formula to calculate total distance from (0,0) to (x_goal, y_goal)
//distance/rate=time
double calculateTimeToMove(double distance){
    double time = distance/g_move_speed;
    return time;
}

double calculateTimeToSpin(double angle){
    return fabs(angle) / g_spin_speed;
}

double ExampleActionServer::sgn(double val){
    if(val > 0)
	return 1;
    else if (val < 0)
	return -1;
    else
	return 0;
}

void ExampleActionServer::do_halt() {
    ros::Rate loop_timer(1.0/g_sample_dt);   
    g_twist_cmd.angular.z = 0.0;
    g_twist_cmd.linear.x = 0.0;
    for (int i = 0; i < 10; i++) {
        g_twist_commander.publish(g_twist_cmd);
        loop_timer.sleep(); 
    }
}

void ExampleActionServer::do_spin(double angle) {
    ROS_INFO("SPINNING angle: %f", angle);
    ros::Rate loop_timer(1.0/g_sample_dt);
    double timer = 0.0;
    double final_time = calculateTimeToSpin(angle);

    g_twist_cmd.angular.z = sgn(angle) * g_spin_speed;
    while(timer < final_time) {
	//ROS_INFO("SPINNING angle: %f", angle);
	g_twist_commander.publish(g_twist_cmd);
	timer += g_sample_dt;
	loop_timer.sleep();
    }  
    do_halt();
}

void ExampleActionServer::do_move(float x_goal, float y_goal) {
    ROS_INFO("MOVING X: %f, Y: %f", x_goal, y_goal);
    ros::Rate loop_timer(1.0/g_sample_dt);
    double distance = distanceFromOrigin(x_goal, y_goal);
    double timer = 0.0;
    double final_time = calculateTimeToMove(distance);
    g_twist_cmd.angular.z = 0.0;
    g_twist_cmd.linear.x = sgn(distance) * g_move_speed;
    while(timer < final_time) {
    	//ROS_INFO("MOVING X: %f, Y: %f", x_goal, y_goal);
	g_twist_commander.publish(g_twist_cmd);
	timer += g_sample_dt;
	loop_timer.sleep();
    }
    do_halt();
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <my_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "my_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<my_action_server::demoAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes

    //define a loop timer
    ros::Rate loop_timer(1/g_sample_dt);

    //use the functions defined above to get a desired function
    //do this for every step passed from the client
    for(int i = 0; i < sizeof(goal->x_vals); i++){
	if(as_.isPreemptRequested()){
	    ROS_INFO("CANCEL REQUESTED BY CLIENT");
	    result_.done = false;
	    as_.setAborted(result_);
	    return;
	}
	ExampleActionServer::do_move(goal->x_vals[i], goal->y_vals[i]);
	ExampleActionServer::do_spin(goal->angle_vals[i]);
	loop_timer.sleep();
    }

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    result_.done = true;

    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_server"); // name this node 


    ROS_INFO("instantiating my action server: ");


    ros::NodeHandle nh; // node handle 
    
    g_twist_commander = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

