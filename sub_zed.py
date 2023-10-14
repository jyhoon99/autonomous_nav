import rospy
from std_msgs.msg import Float32MultiArray, String

def mj_depth_callback(msg):
    # 이 콜백 함수에서 수신한 메시지 처리 로직을 작성합니다.
    # msg.data를 사용하여 메시지 데이터에 접근할 수 있습니다.
    rospy.loginfo("Received 'mj_depth' message: %s", msg.data)

def mj_type_callback(msg):
    # 이 콜백 함수에서 수신한 메시지 처리 로직을 작성합니다.
    # mj_type을 사용하여 원하는 작업을 수행합니다.
    rospy.loginfo("Received 'mj_type' message: %s", msg.data)

def main():
 
    rospy.init_node('mj_depth_subscriber')  # 노드 초기화
    rospy.init_node('mj_type_subscriber')  # 노드 초기화
    # "mj_depth" 토픽에서 Float32MultiArray 메시지를 구독하는 서브스크라이버 생성
    rospy.Subscriber("mj_depth", Float32MultiArray, mj_depth_callback)
    rospy.Subscriber("mj_type", String, mj_type_callback)

    # 루프를 유지하여 노드가 종료되지 않도록 합니다.
    rospy.spin()

if __name__ == '__main__':
    main()