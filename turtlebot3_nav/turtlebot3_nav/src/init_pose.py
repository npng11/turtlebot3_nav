#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def main():
    """
    main program
    """
    
    # Initialize
    rospy.init_node("init_pose")
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    # Construct message
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = rospy.Time.now()

    # Get current position from topic /odom
    odom_pose = rospy.wait_for_message('/odom', Odometry)
    init_pose.pose.pose.position.x = odom_pose.pose.pose.position.x
    init_pose.pose.pose.position.y = odom_pose.pose.pose.position.y
    init_pose.pose.pose.orientation.x = odom_pose.pose.pose.orientation.x
    init_pose.pose.pose.orientation.y = odom_pose.pose.pose.orientation.y
    init_pose.pose.pose.orientation.z = odom_pose.pose.pose.orientation.z
    init_pose.pose.pose.orientation.w = odom_pose.pose.pose.orientation.w

    rospy.sleep(1)

    # Publish
    rospy.loginfo(f"setting initial pose\n \
                pos x: {init_pose.pose.pose.position.x}\n \
                pos y: {init_pose.pose.pose.position.y}\n \
                orien x: {init_pose.pose.pose.orientation.x}\n \
                orien y: {init_pose.pose.pose.orientation.y}\n \
                orien z: {init_pose.pose.pose.orientation.z}\n \
                orien w: {init_pose.pose.pose.orientation.w}")
    pub.publish(init_pose)
    rospy.loginfo("initial pose is set")


if __name__ == "__main__":
    main()