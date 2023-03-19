import rospy
from geometry_msgs.msg import Twist, Pose2D
import numpy as np


def pose_received_callback(turtle_pose):
    """
    Callback function to receive the current pose of the robot.

    Args:
        turtle_pose (geometry_msgs/Pose2D): The current pose of the robot.

    Returns:
        None
    """
    global x_turtle, y_turtle, theta_turtle

    x_turtle = turtle_pose.x
    y_turtle = turtle_pose.y
    theta_turtle = turtle_pose.theta


def goal_received_callback(turtle_goal):
    """
    Callback function to receive the goal pose for the robot.

    Args:
        turtle_goal (geometry_msgs/Pose2D): The goal pose for the robot.

    Returns:
        None
    """
    x_g = turtle_goal.x
    y_g = turtle_goal.y
    theta_g = turtle_goal.theta
    rho = np.sqrt(np.power(x_g - x_turtle, 2) + np.power(y_g - y_turtle, 2))
    turtle_speed = Twist()

    # Move towards the goal pose using closed-loop controller
    while not rho < 0.1:
        rho = np.sqrt(np.power(x_g - x_turtle, 2) + np.power(y_g - y_turtle, 2))
        alpha = np.arctan2((y_g - y_turtle), (x_g - x_turtle)) - theta_turtle
        alpha = np.mod(alpha + np.pi, 2 * np.pi) - np.pi
        turtle_speed.linear.x = k_rho * rho
        turtle_speed.angular.z = k_alpha * alpha
        velocity_pub.publish(turtle_speed)
        rospy.sleep(0.01)
    print("Reach position")

    # Turn to face the goal orientation using closed-loop controller
    beta = theta_turtle - theta_g
    while not np.abs(beta) < 0.05:
        beta = theta_turtle - theta_g
        turtle_speed.linear.x = 0
        turtle_speed.angular.z = k_beta * beta
        velocity_pub.publish(turtle_speed)
        rospy.sleep(0.01)
    print("Reach orientation")

    # Stop the robot
    turtle_speed.linear.x = 0
    turtle_speed.angular.z = 0
    velocity_pub.publish(turtle_speed)
    rospy.sleep(0.5)


def controller_init():
    """
    Initializes the ROS node and subscribes to the required topics.
    """
    rospy.init_node("turtle_closed_loop", anonymous=True)
    global k_rho, k_alpha, k_beta
    k_rho = 0.8  # rospy.get_param("k_rho")
    k_alpha = 0.8  # rospy.get_param("k_alpha")
    k_beta = 0  # -3.5 #rospy.get_param("k_beta")
    rospy.Subscriber("/pioneer/pose", Pose2D, pose_received_callback)
    rospy.Subscriber("/pioneer/goal", Pose2D, goal_received_callback)
    global velocity_pub

    velocity_pub = rospy.Publisher("/pioneer/cmd_vel", Twist, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    controller_init()
