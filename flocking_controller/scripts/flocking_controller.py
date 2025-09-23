#!/usr/bin/env python3
"""
Flocking Controller Node (ROS)

This script defines a basic ROS node template for controlling a flock of robots
using the `mrs_msgs/Vec4` service. 
"""

import rospy
from mrs_msgs.srv import Vec4, Vec4Request, Vec4Response


class FlockingController:
    def __init__(self):
        """
        Initialize the Flocking Controller node.
        """
        rospy.init_node("flocking_controller", anonymous=True)

        # Name of the Vec4 service (I have to adjust)
        self.service_name = "control_manager/velocity_reference"

        rospy.loginfo(f"Waiting for service: {self.service_name}")
        rospy.wait_for_service(self.service_name)

        try:
            # Create a proxy to call the Vec4 service
            self.service_client = rospy.ServiceProxy(self.service_name, Vec4)
            rospy.loginfo("Service client initialized successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to connect to service {self.service_name}: {e}")
            rospy.signal_shutdown("Service connection failed.")

    def send_velocity_command(self, x: float, y: float, z: float, yaw: float):
        """
        Send a velocity command using the Vec4 service.

        Args:
            x (float): velocity in x direction
            y (float): velocity in y direction
            z (float): velocity in z direction
            yaw (float): yaw angular velocity
        """
        try:
            # Create the request message
            request = Vec4Request()
            request.goal.x = x
            request.goal.y = y
            request.goal.z = z
            request.goal.yaw = yaw

            rospy.loginfo(
                f"Sending velocity command: "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}"
            )

            # Call the service
            response: Vec4Response = self.service_client(request)

            if response.success:
                rospy.loginfo("Velocity command sent successfully.")
            else:
                rospy.logwarn("Velocity command failed.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    try:
        controller = FlockingController()

        # Keep node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Flocking controller node terminated.")
