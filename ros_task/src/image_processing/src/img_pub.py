#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image():
    # 初始化ROS节点
    rospy.init_node('image_publisher', anonymous=True)
    
    # 创建Publisher，发布到 'image_topic' 话题
    pub = rospy.Publisher('image_topic', Image, queue_size=10)
    
    # 创建CvBridge对象
    bridge = CvBridge()
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)  # 0 表示默认摄像头
    if not cap.isOpened():
        rospy.logerr("Failed to open camera!")
        return

    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz，每秒发布10次
    
    while not rospy.is_shutdown():
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture frame from camera!")
            break
        
        # 将OpenCV图像转换为ROS图像消息
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        
        # 发布图像消息
        pub.publish(msg)
        
        # 按照频率休眠
        rate.sleep()

    # 释放摄像头
    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
