#!/usr/bin/env python

import rospy
from hatch.msg import Message 

def main():
    rospy.init_node("Publisher_node")
    pub = rospy.Publisher("topic", Message, queue_size=10)
    rate = rospy.Rate(10)
    work = Message()

    while not rospy.is_shutdown():
        key = input() 

#350
#600
#700
        if key == 'q':
            work.mode = 1
        elif key == 'w':
            work.mode = 2
        elif key == 'e':
            work.mode = 3
        elif key == 'r':
            work.mode = 4
        elif key == 'a':
            work.mode = 5
        elif key == 's':
            work.mode = 6
        elif key == 'd':
            work.mode = 7

        pub.publish(work)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




# def main():
#     rospy.init_node("Publisher_node")
#     pub = rospy.Publisher("topic", Message, queue_size=10)
#     rate = rospy.Rate(10)
#     work = Message()

#     while not rospy.is_shutdown():
#         work.mode = 1
#         pub.publish(work)
#         rate.sleep()  # sleep() 함수를 호출할 때 인수를 제거합니다.
        
#         work.mode = 2
#         pub.publish(work)
#         rate.sleep()  # sleep() 함수를 호출할 때 인수를 제거합니다.
        
#         work.mode = 3
#         pub.publish(work)
#         rate.sleep()  # sleep() 함수를 호출할 때 인수를 제거합니다.

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass