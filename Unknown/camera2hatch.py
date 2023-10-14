#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from hatch.msg import Message

# 객체를 감지했을때 동작하는 함수
def object_detection():
    rospy.init_node("Publisher_node")
    pub = rospy.Publisher("topic", Message, queue_size=10)
    rate = rospy.Rate(10)
    work = Message()

    # 카메라 정보 받을 객체 생성
    # cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # 객체 정보 읽기
        # ret, frame = cap.read()

        if detected_object == 'A':
            # A위치까지 이동
            work.mode = 1
        elif detected_object == 'B':
            # B위치까지 이동
            work.mode = 2
        elif detected_object == 'C':
            # C위치까지 이동            
            work.mode = 3
        else:
            work.mode = 0  # 객체를 감지하지 않은 경우

        pub.publish(work)
        rate.sleep()

    # 웹캠을 해제합니다.
    cap.release()

# 객체를 감지하고 값을 반환하는 함수 
def detect_object(frame):
    # 여기에서 객체 감지를 수행하고 A, B, C 중 하나를 반환합니다.
    # OpenCV 또는 다른 객체 감지 라이브러리를 사용하여 구현합니다.
    
    # 예를 들어, frame에서 객체를 감지하고 해당 객체의 레이블을 반환하는 코드를 작성하십시오.

    # 임시로 무작위로 객체를 감지하는 가짜 코드 예시:
    import random
    objects = ['A', 'B', 'C', None]  # None은 객체를 감지하지 않은 경우
    return random.choice(objects)

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass