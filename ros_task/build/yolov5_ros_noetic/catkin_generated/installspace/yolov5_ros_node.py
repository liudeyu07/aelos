#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import cv2
import torch
import numpy as np

class YOLOv5ROS:
    def __init__(self):
        rospy.init_node('yolov5_ros_node', anonymous=True)
        self.detection_pub = rospy.Publisher('yolov5_detections', Float32MultiArray, queue_size=10)
        self.object_name_pub = rospy.Publisher('yolov5_object_names', String, queue_size=10)
        self.rate = rospy.Rate(30)  # 发布频率 30Hz

        # YOLOv5 模型加载
        self.model_path = "/home/yyros/yolov5/yolov5s.pt"  # 修改为你的 yolov5s.pt 文件路径
        self.yolov5_path = "/home/yyros/yolov5" # 修改为你的 yolov5 代码仓库路径
        self.conf_thres = 0.5  # 置信度阈值
        self.iou_thres = 0.45  # IOU 阈值

        try:
            self.model = torch.hub.load(self.yolov5_path, 'custom', path=self.model_path, source='local')
            # 或者使用本地加载方式，如果上面 hub.load 不成功
            # self.model = torch.load(self.model_path)
            # self.model.eval() # 设置为评估模式
        except Exception as e:
            rospy.logerr("Failed to load YOLOv5 model: %s", str(e))
            exit(1)


        self.cap = cv2.VideoCapture(0) # 默认摄像头，你可以修改为视频文件路径

        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open video source.")
            exit(1)
        else:
            rospy.loginfo("Video source opened successfully.")


    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Video frame empty, exiting.")
                break

            # YOLOv5 推理
            results = self.model(frame)

            # 解析结果并发布
            self.publish_detections(results)

            self.rate.sleep()

        self.cap.release()


    def publish_detections(self, results):
        detections_array = Float32MultiArray()
        object_names_str = ""

        # 获取检测结果
        detections = results.xyxy[0].cpu().numpy()  # 获取 xyxy 格式的检测框，并转换为 numpy 数组

        for *xyxy, conf, cls in detections:
            if conf >= self.conf_thres: # 过滤掉低置信度的检测结果
                x_min, y_min, x_max, y_max = map(int, xyxy) # 转换为整数坐标
                class_id = int(cls)
                class_name = self.model.names[class_id]

                # 创建检测框数据 [x_min, y_min, x_max, y_max, confidence, class_id]
                detection_data = [float(x_min), float(y_min), float(x_max), float(y_max), float(conf), float(class_id)]
                detections_array.data.extend(detection_data) # 将检测框数据添加到 Float32MultiArray 的 data 列表中
                object_names_str += class_name + "," # 拼接物体名称字符串


        self.detection_pub.publish(detections_array) # 发布所有检测框的坐标和置信度等信息
        self.object_name_pub.publish(String(data=object_names_str)) # 发布所有检测到的物体名称 (逗号分隔)

        if len(detections_array.data) > 0:
            rospy.loginfo("Published detections: %s", str(detections_array.data))
            rospy.loginfo("Published object names: %s", object_names_str)



if __name__ == '__main__':
    try:
        yolov5_ros = YOLOv5ROS()
        yolov5_ros.run()
    except rospy.ROSInterruptException:
        pass
