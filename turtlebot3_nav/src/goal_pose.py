#!/usr/bin/env python3

import rospy
import actionlib
import random
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def main():
    """
    main program
    """

    # Iniitialize
    rospy.init_node("goal_pose")
    nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    nav_client.wait_for_server()
    
    # Get map & costmap data, estimate not occupy cells
    map_data = get_map_data()
    costmap_data = get_costmap_data()
    free_cells, r, ori_x, ori_y = estimate_free_space(map_data, costmap_data)

    # Loop to keep autonomous navigation 
    while True:
        goal = random_goal_pose(free_cells, r, ori_x, ori_y)
        nav_client.send_goal(goal, done_cb=done_cb)
        res = nav_client.wait_for_result()

        if not res:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo(nav_client.get_result())
        
        rospy.sleep(3)

def estimate_free_space(map_data, costmap_data):
    """
    # Estimate available empty cells from map and costmap
    """

    map_info = map_data.info
    w, h = map_info.width, map_info.height
    r = map_info.resolution
    ori_x, ori_y = map_info.origin.position.x, map_info.origin.position.y

    free_cells = []
    for y in range(h):
        for x in range(w):
            if is_valid(map_data, costmap_data, x, y):
                free_cells.append((x, y))

    if not free_cells:
        rospy.logerr("No free space found in the map!")
        return None

    return free_cells, r, ori_x, ori_y

def random_goal_pose(free_cells, r, ori_x, ori_y):
    """
    # Randomly generate goal pose for navigation based from available free cells
    """

    # Get curretn robot position
    distance = 0
    robot_x, robot_y = get_robot_pos()

    # To ensure travel distance > 1
    while distance < 1:
        random_cell = free_cells[np.random.randint(len(free_cells))]
        goal_x = random_cell[0] * r + ori_x
        goal_y = random_cell[1] * r + ori_y

        distance = np.sqrt((robot_x - goal_x)**2 + (robot_y - goal_y)**2)
        rospy.loginfo(f"{distance}")

    # Set goal pose
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation = random_orientation()

    rospy.loginfo(f"setting goal pose\n \
                pos x: {goal.target_pose.pose.position.x}\n \
                pos y: {goal.target_pose.pose.position.y}\n \
                orien x: {goal.target_pose.pose.orientation.x}\n \
                orien y: {goal.target_pose.pose.orientation.y}\n \
                orien z: {goal.target_pose.pose.orientation.z}\n \
                orien w: {goal.target_pose.pose.orientation.w}")
    return goal

def get_map_data():
    """
    # Get map data from topic /map 
    """

    rospy.loginfo("Waiting for map data")
    map_data = None
    
    def map_cb(data):
        nonlocal map_data
        map_data = data
    
    rospy.Subscriber("/map", OccupancyGrid, map_cb)
    rospy.sleep(1)
    
    return map_data

def get_costmap_data():
    """
    # Get costmap data from topiv /move_base/global_costmap/costmap 
    """

    rospy.loginfo("Watinig for local costmap data")
    costmap_data = None

    # Callback
    def costmap_cb(data):
        nonlocal costmap_data
        costmap_data = data

    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, costmap_cb)
    rospy.sleep(1)

    return costmap_data

def get_robot_pos():
    """
    # Get robot current position from topic /amcl_pose
    """
    
    robot_pos = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    return robot_pos.pose.pose.position.x, robot_pos.pose.pose.position.y

def is_valid(map_data, costmap_data, x, y):
    """
    # Check if coordinate in map and costmap is empty and valid
    """

    map_info = map_data.info
    costmap_info = costmap_data.info

    # 2D Map data are stored in 1-Dimesion
    # x + y * width <== to estimate which index of map data belongs to coordinate (x, y)
    map_index = x + y * map_info.width
    costmap_index = x + y * costmap_info.width

    # -1: not available
    # 0: not occupy
    # 100: occupied
    if map_data.data[map_index] == 0 and costmap_data.data[costmap_index] < 30: 
        return True
        
    return False

def random_orientation():
    """
    # Randomly generate orientcation in quaternion
    """
    
    q = quaternion_from_euler(0.0, 0.0, random.uniform(0, 2 * 3.14))
    orien = Quaternion()
    orien.x = q[0]
    orien.y = q[1]
    orien.z = q[2]
    orien.w = q[3]

    return orien

# callback
def active_cb(extra):
    rospy.logdebug(f"Goal pose being processed. {extra}")

def feedback_cb(feedback):
    rospy.logdebug(f"Current location: {feedback}")

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Reached goal")
    elif status == 4:
        rospy.loginfo("Aborted!")

if __name__ == "__main__":
    main()