#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

# Define the goal positions as an array
goal_positions = [
    {
        'x': 1,
        'y': 0,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': -0.7,
        'qw': 0.7
    },
    {
        'x': 1,
        'y': -1.3,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': -1,
        'qw': 0
    },
    {
        'x': 0.3,
        'y': -1.4,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0,
        'qw': 1
    },
    {
        'x': 0.1803995817899704,
        'y': 0.056696951389312744,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0,
        'qw': 1
    }
]

# Callbacks definition
def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    elif status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    elif status == 4:
        rospy.loginfo("Goal aborted")

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    pass  # You can process feedback here if needed

def send_navigation_goal(client, x, y, z, qx, qy, qz, qw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
    client.wait_for_result()

def set_initial_pose():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
    initpose_msg = PoseWithCovarianceStamped()
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = 0.1803995817899704
    initpose_msg.pose.pose.position.y = 0.056696951389312744
    initpose_msg.pose.pose.orientation.x = 0.0
    initpose_msg.pose.pose.orientation.y = 0.0
    initpose_msg.pose.pose.orientation.z = 0
    initpose_msg.pose.pose.orientation.w = 0.9999999999999988
    rospy.sleep(1)
    rospy.loginfo("Setting initial pose")
    pub.publish(initpose_msg)
    rospy.loginfo("Initial pose SET")

if __name__ == '__main__':
    rospy.init_node('set_initial_and_send_goal', anonymous=True)
    set_initial_pose()
    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    navclient.wait_for_server()

    while not rospy.is_shutdown():
        for position in goal_positions:
            send_navigation_goal(
                navclient,
                position['x'],
                position['y'],
                position['z'],
                position['qx'],
                position['qy'],
                position['qz'],
                position['qw']
            )

    rospy.spin()