# 1. better_code : 좌표 1,2,3,4 반복 이동 
#   시용함수 : set_init_pose() : 초기좌표 설정
#             sned_navigation_goal() : 목표위치 설정
# *** set_init_pose는  map5 기준 || better_code는 map3 기준
# 하나의 파일에는 하나의 노드만 존재해야한다. ( = init_node 하나 여야한다.) 


# 2. App_nav.py

# 3. second.py : cmd_vel 이동 , 회전 &  해치,리니어 작동 

# 초기위치 설정 - rviz상 모델 이동
import numpy as np
from tf.transformations import euler_from_quaternion
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32MultiArray
from hatch.msg import Message  


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
    pass  

def hatch(mode):
    hatch_pub = rospy.Publisher("topic", Message, queue_size=10) 
    work = Message()
    work.mode = mode
    hatch_pub.publish(work)
    rospy.loginfo("동작 완료")

# 회전 함수
def rotate_robot(rotation_speed, rotation_direction, duration):
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # Set the angular (z-axis) velocity for rotation
    twist.angular.z = rotation_speed * rotation_direction
    start_time = rospy.Time.now()
    
    # 주어진 지속 시간 동안 Twist 메시지를 주기적으로 게시하여 로봇을 회전시킴
    while (rospy.Time.now() - start_time).to_sec() < duration:
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)

    # 회전이 완료되면 로봇을 정지시키고 ROS 노드를 종료
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    rospy.loginfo("회전 완료")
    
def set_initial_pose():
    global initpose_msg
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initpose_msg = PoseWithCovarianceStamped()
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = 0.07339318096637726
    initpose_msg.pose.pose.position.y = 0.030580442398786545
    initpose_msg.pose.pose.orientation.x = 0.0
    initpose_msg.pose.pose.orientation.y = 0.0
    initpose_msg.pose.pose.orientation.z = 0.9238795336521503
    initpose_msg.pose.pose.orientation.w = 0.38268342961080154
    rospy.sleep(1)
    rospy.loginfo("Setting initial pose")
    pub.publish(initpose_msg)
    rospy.loginfo("Initial pose SET")

def mj_depth_callback(data):
    global initpose_msg 
    x_values = data.data[:len(data.data)//2] # 첫 번째 절반은 x_values
    z_values = data.data[len(data.data)//2:]  # 나머지 절반은 z_values

    current_x = initpose_msg.pose.pose.position.x
    current_y = initpose_msg.pose.pose.position.y
    current_orientation = initpose_msg.pose.pose.orientation
    (current_roll, current_pitch, current_yaw) = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

    # 좌측 또는 우측으로 x cm 이동
    x_m = - 0.012168 -0.0151599566389232

    # 앞쪽으로 z cm 이동
    y_m = z_values[0]*10 -0.1

    # 새로운 위치 계산
    object_x = current_x + y_m * np.cos(current_yaw)
    object_y = current_y + y_m * np.sin(current_yaw)
    new_x = object_x - x_m * np.sin(current_yaw)
    new_y = object_y + x_m * np.cos(current_yaw)
    new_yaw = np.arctan2(current_y - new_y, current_x - new_x)

    new_orientation = [0, 0, np.sin(new_yaw / 2), np.cos(new_yaw / 2)]

    #새로 계산한 쿼터니언 값 출력
    new_orientation_x = new_orientation[0]
    new_orientation_y = new_orientation[1]
    new_orientation_z = new_orientation[2]
    new_orientation_w = new_orientation[3]

    print("Received x_values:", x_values)
    print("Received z_values:", z_values)
    print(new_x)
    print(new_y)


    # rospy.init_node('send_goal')

    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    navclient.wait_for_server()

    # Example of navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 목표위치 설정
    goal.target_pose.pose.position.x = new_x
    goal.target_pose.pose.position.y = new_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = new_orientation_x
    goal.target_pose.pose.orientation.y = new_orientation_y
    goal.target_pose.pose.orientation.z = new_orientation_z
    goal.target_pose.pose.orientation.w = new_orientation_w 
    
    # 목표위치 이동
    navclient.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
    finished = navclient.wait_for_result()

    # 로봇 회전
    rospy.sleep(2.0)
    rotate_robot(0.5, -2, 3.6)  # 180도 회전
    rospy.sleep(2.0)

    # 후면해치 작동
    hatch(4)
    rospy.sleep(1.0)
    hatch(5)
    rospy.sleep(1.0)
    hatch(6)
    rospy.sleep(1.0)
    hatch(7)
    rospy.sleep(1.0)



# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.x: {:.6f}".format(new_orientation_x))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.y: {:.6f}".format(new_orientation_y))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.z: {:.6f}".format(new_orientation_z))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.w: {:.6f}".format(new_orientation_w))
# print("로봇의 중점에서 좌측으로 {} cm, 앞쪽으로 {} cm 떨어진 위치: x={}, y={}".format(x_cm, y_cm, new_x, new_y))


if __name__ == '__main__':
    rospy.init_node('first_node', anonymous=True)

    # Setting initial pose
    set_initial_pose()

    # Initializing navigation client
    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    navclient.wait_for_server()

    # Subscriber for depth data
    sub = rospy.Subscriber('mj_depth', Float32MultiArray, mj_depth_callback)

rospy.spin()  # 노드를 계속 실행하기 위해 추가
