#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mrs_msgs/Vec4.h>
#include <geometry_msgs/Point.h>

// Global variables
geometry_msgs::Point current_position;
ros::ServiceClient client;
std::string service_name;
bool position_received_ = false; 

// Subscriber callback for /drone_0_planning/pos_cmd
void positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    current_position = msg->position;
    position_received_ = true; 
    //ROS_INFO("Position received: x: %f, y: %f, z: %f", current_position.x, current_position.y, current_position.z);
}

void callService() {

    if (!position_received_) {
        ROS_WARN("No position received yet. Service will not be called.");
        return;
    }

    mrs_msgs::Vec4 srv;
    srv.request.goal[0] = current_position.x;
    srv.request.goal[1] = current_position.y;
    srv.request.goal[2] = current_position.z;
    srv.request.goal[3] = 0.0;  // Assuming `w` is not used for position

    if (client.call(srv)) {
        ROS_INFO("Service call succeeded: Pos received: x: %f, y: %f, z: %f", current_position.x, current_position.y, current_position.z);
    } else {
        ROS_ERROR("Failed to call service: Pos received: x: %f, y: %f, z: %f", current_position.x, current_position.y, current_position.z);
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "my_drone_node");
    ros::NodeHandle nh_;

    // Get topic and service names from ROS parameters
    std::string pos_cmd_topic;
    nh_.param<std::string>("pos_cmd_topic", pos_cmd_topic, "/drone_0_planning/pos_cmd");
    nh_.param<std::string>("service_name", service_name, "control_manager/goto");

    // Subscribe to the pos_cmd topic
    ros::Subscriber sub = nh_.subscribe(pos_cmd_topic, 1000, positionCmdCallback);

    // Set up the service client
    client = nh_.serviceClient<mrs_msgs::Vec4>(service_name);

    // Create a rate object for 100 Hz frequency
    ros::Rate loop_rate(110);

    // Create a rate object for service call at 1 Hz
    // ros::Rate service_rate(1);
    
    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // Process incoming messages
        ros::spinOnce();

        if( (ros::Time::now() - last_request > ros::Duration(0.5))){
            callService();
            last_request = ros::Time::now();
        }

        // // Service call at 1 Hz
        // if (service_rate.cycleTime() >= service_rate.expectedCycleTime()) {
        //     callService();
        // }

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
