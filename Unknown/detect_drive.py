#!/usr/bin/env python

# 정해진 경로대로 이동하다, 객체 인식을 한다면 객체까지 이동 (+해치 동작)
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

# 새로운 콜백 함수: 객체 인식 결과를 받는 콜백
def object_recognition_callback(msg):
    if msg.data:  # 객체를 감지했을 때
        rospy.loginfo("Object detected. Cancelling current goal and stopping the robot.")
        navclient.cancel_all_goals()  # 현재 수행 중인 목표 취소

        # 로봇을 객체의 위치까지 이동
        # 아래는 예시로 객체의 위치로 이동하는 코드입니다. 실제 동작에 따라 변경해야 합니다.
        goal_to_object = MoveBaseGoal()
        goal_to_object.target_pose.header.frame_id = "map"
        goal_to_object.target_pose.header.stamp = rospy.Time.now()
        goal_to_object.target_pose.pose.position.x = object_x  # 객체의 x 좌표로 설정
        goal_to_object.target_pose.pose.position.y = object_y  # 객체의 y 좌표로 설정
        goal_to_object.target_pose.pose.orientation.w = 1.0  # 직진 방향 설정
        navclient.send_goal(goal_to_object)
        navclient.wait_for_result()

        # 여기에 "내가 지정한 동작" 코드를 추가하세요.
        # 이 부분에는 로봇이 객체와 상호작용하는 동작 등을 구현합니다.
        # 예를 들어, 객체를 집는다거나, 특정 동작을 수행한다면 해당 코드를 작성합니다.
        # 작성된 코드는 객체의 위치로 이동한 후 수행됩니다.

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

rospy.init_node('set_initial_and_send_goal', anonymous=True)

# Setting initial pose
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
initpose_msg = PoseWithCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = 0.6965455412113103
initpose_msg.pose.pose.position.y = -0.7980375089218309
initpose_msg.pose.pose.orientation.x = 0.0
initpose_msg.pose.pose.orientation.y = 0.0
initpose_msg.pose.pose.orientation.z = 0.7443778905867938
initpose_msg.pose.pose.orientation.w = 0.6677586060887236
rospy.sleep(1)
rospy.loginfo("Setting initial pose")
pub.publish(initpose_msg)
rospy.loginfo("Initial pose SET")

# Initializing navigation client
navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

# Subscribe to object recognition topic
rospy.Subscriber("/object_recognition", Bool, object_recognition_callback)

# First navigation goal
goal1 = MoveBaseGoal()
goal1.target_pose.header.frame_id = "map"
goal1.target_pose.header.stamp = rospy.Time.now()
goal1.target_pose.pose.position.x = 0.24866577976929335
goal1.target_pose.pose.position.y = 0.35748931801156236
goal1.target_pose.pose.position.z = 0.0
goal1.target_pose.pose.orientation.x = 0.0
goal1.target_pose.pose.orientation.y = 0.0
goal1.target_pose.pose.orientation.z = 0.12206856298411714
goal1.target_pose.pose.orientation.w = 0.9925216702576285

# Sending first navigation goal
navclient.send_goal(goal1, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
navclient.wait_for_result()

# Second navigation goal
goal2 = MoveBaseGoal()
goal2.target_pose.header.frame_id = "map"
goal2.target_pose.header.stamp = rospy.Time.now()
goal2.target_pose.pose.position.x = 1.7174397627040616
goal2.target_pose.pose.position.y = 0.7517977623856505
goal2.target_pose.pose.position.z = 0.0
goal2.target_pose.pose.orientation.x = 0.0
goal2.target_pose.pose.orientation.y = 0.0
goal2.target_pose.pose.orientation.z = -0.5948196303059364
goal2.target_pose.pose.orientation.w = 0.8038591962543622

# Sending second navigation goal
navclient.send_goal(goal2, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
navclient.wait_for_result()

# Third navigation goal
goal3 = MoveBaseGoal()
goal3.target_pose.header.frame_id = "map"
goal3.target_pose.header.stamp = rospy.Time.now()
goal3.target_pose.pose.position.x = 1.9390258404829472
goal3.target_pose.pose.position.y = -0.1508526021041243
goal3.target_pose.pose.position.z = 0.0
goal3.target_pose.pose.orientation.x = 0.0
goal3.target_pose.pose.orientation.y = 0.0
goal3.target_pose.pose.orientation.z = -0.6253059055562248
goal3.target_pose.pose.orientation.w = 0.780379730949305

# Sending third navigation goal
navclient.send_goal(goal3, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
navclient.wait_for_result()

rospy.spin()