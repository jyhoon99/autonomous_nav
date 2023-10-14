#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
# from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import numpy as np
# ROS 노드 초기화
rospy.init_node('set_initial_pose_node')

def set_initial_pose():
    # '/initialpose' 토픽에 PoseWithCovarianceStamped 메시지를 퍼블리시하기 위한 퍼블리셔 생성
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # PoseWithCovarianceStamped 메시지 생성
    initpose_msg = PoseWithCovarianceStamped()
    
    # 메시지 필드 설정
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = -0.009999923408031464
    initpose_msg.pose.pose.position.y = -0.09999989718198776
    initpose_msg.pose.pose.orientation.x = 0.0
    initpose_msg.pose.pose.orientation.y = 0.0
    initpose_msg.pose.pose.orientation.z = -0.2392205871446352
    initpose_msg.pose.pose.orientation.w = 0.9709652468992781
    current_orientation = initpose_msg.pose.pose.orientation
    (current_roll, current_pitch, current_yaw) = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
    # 좌측 또는 우측으로 x cm 이동




    ######## np.arctan2 계산할때 new_y - initpose로 바꾸기

    
    x_values = 0 # 첫 번째 절반은 x_values
    z_values = 0.08090106332302094 * 10 + 0.2  # 나머지 절반은 z_values

    x_m =   x_values

    # 앞쪽으로 z cm 이동
    y_m = z_values


    object_x = initpose_msg.pose.pose.position.x  + y_m * np.cos(current_yaw)
    object_y = initpose_msg.pose.pose.position.y  + y_m * np.sin(current_yaw)
    new_x = object_x - x_m * np.sin(current_yaw)
    new_y = object_y + x_m * np.cos(current_yaw)


    new_yaw = np.arctan2(new_y - initpose_msg.pose.pose.position.y, new_x - initpose_msg.pose.pose.position.x)

    new_orientation = [0, 0, np.sin(new_yaw / 2), np.cos(new_yaw / 2)]

    #새로 계산한 쿼터니언 값 출력
    initpose_msg.pose.pose.orientation.x = new_orientation[0]
    initpose_msg.pose.pose.orientation.y = new_orientation[1]
    initpose_msg.pose.pose.orientation.z = new_orientation[2]
    initpose_msg.pose.pose.orientation.w = new_orientation[3]
    # 1초 대기
    rospy.sleep(1)
    
    # 로그 출력
    rospy.loginfo("Setting initial pose")
    
    # 메시지 퍼블리시
    pub.publish(initpose_msg)
    
    # 로그 출력
    rospy.loginfo("Initial pose SET")

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python

# import rospy
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from tf.transformations import euler_from_quaternion
# import numpy as np
# # CvBridge 객체 생성


# current_x = 0.07339318096637726
# current_y = 0.030580442398786545
# initpose_msg = PoseWithCovarianceStamped()
# initpose_msg.pose.pose.orientation.x = 0.0
# initpose_msg.pose.pose.orientation.y = 0.0
# initpose_msg.pose.pose.orientation.z = 0.9238795336521503
# initpose_msg.pose.pose.orientation.w = 0.38268342961080154
# current_orientation = initpose_msg.pose.pose.orientation
# (current_roll, current_pitch, current_yaw) = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
# # 좌측 또는 우측으로 x cm 이동



# x_values = - 0.012168 -0.0151599566389232 # 첫 번째 절반은 x_values
# z_values = 0.057690106332302094 * 10 + 0.2  # 나머지 절반은 z_values

# x_m =   x_values

# # 앞쪽으로 z cm 이동
# y_m = z_values


# object_x = current_x + y_m * np.cos(current_yaw)
# object_y = current_y + y_m * np.sin(current_yaw)
# new_x = object_x - x_m * np.sin(current_yaw)
# new_y = object_y + x_m * np.cos(current_yaw)


# new_yaw = np.arctan2(current_y - new_y, current_x - new_x)

# new_orientation = [0, 0, np.sin(new_yaw / 2), np.cos(new_yaw / 2)]

# #새로 계산한 쿼터니언 값 출력
# new_orientation_x = new_orientation[0]
# new_orientation_y = new_orientation[1]
# new_orientation_z = new_orientation[2]
# new_orientation_w = new_orientation[3]


# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.x: {:.6f}".format(new_orientation_x))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.y: {:.6f}".format(new_orientation_y))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.z: {:.6f}".format(new_orientation_z))
# print("로봇이 객체를 일직선으로 바라볼 때의 orientation.w: {:.6f}".format(new_orientation_w))
# print("x={}, y={}".format(new_x, new_y))
