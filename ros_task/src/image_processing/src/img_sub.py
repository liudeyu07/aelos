#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def detect_red_object(image):
    # 高斯模糊
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # 定义红色的HSV范围 (根据实际情况调整)
    lower_red1 = np.array([0, 100, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 50])
    upper_red2 = np.array([180, 255, 255])
    
    # 创建掩码
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 形态学操作 (可以尝试调整)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 设置最小面积阈值
    min_area = 500
    
    for contour in contours:
        # 计算轮廓面积
        area = cv2.contourArea(contour)
        
        # 过滤掉面积较小的轮廓
        if area > min_area:
            # 获取矩形框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 绘制矩形框
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 红色边框
            
            # 输出矩形中心坐标
            center = (x + w // 2, y + h // 2)
            print(f"Red object detected at: {center}")
    
    return image

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return
    
    # 处理图像
    processed_image = detect_red_object(cv_image)
    
    # 发布处理后的图像
    pub.publish(bridge.cv2_to_imgmsg(processed_image, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    pub = rospy.Publisher('processed_image_topic', Image, queue_size=10)
    rospy.Subscriber('image_topic', Image, image_callback)
    rospy.spin()
