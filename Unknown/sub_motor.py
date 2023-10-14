#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from dynamixel_sdk import *

# Control table address
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting
DXL1_ID = 1
DXL2_ID = 2
DXL3_ID = 3
BAUDRATE = 115200
DEVICE_NAME = '/dev/ttyUSB0'

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def topic_callback(work):
    mod = work.mode
    dxl_error = 0
    dxl_comm_result = 0

    if mod == 1:
        # 동작1
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6554000)
        rospy.sleep(6)  # wait for 6 seconds
        # 리니어
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6553800)
        rospy.sleep(6)
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6554150)
    elif mod == 2:
        # 동작2
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, 6554100)
        rospy.sleep(3)  # wait for 3 seconds
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, 6553650)
        rospy.sleep(3)
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, 6553800)
    elif mod == 3:
        # 동작3
        dxl_comm_result, _ = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, 6553790)

    rospy.loginfo("Received message:")
    rospy.loginfo("mode = %d", work.mode)

def main():
    global portHandler, packetHandler

    dxl_error = 0
    dxl_comm_result = COMM_TX_FAIL

    if not portHandler.openPort():
        rospy.logerr("Failed to open the port!")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Failed to set the baudrate!")
        return

    dxl_comm_result, _ = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("Failed to enable torque for Dynamixel ID %d", DXL1_ID)
        return

    dxl_comm_result, _ = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("Failed to enable torque for Dynamixel ID %d", DXL2_ID)
        return

    dxl_comm_result, _ = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("Failed to enable torque for Dynamixel ID %d", DXL3_ID)
        return

    rospy.init_node("back_sub_node")
    rospy.Subscriber("/topic", String, topic_callback)
    rospy.spin()

    portHandler.closePort()

if __name__ == "__main__":
    main()