#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class YOLOv5ROS:
    def __init__(self):
        rospy.init_node('yolov5_ros_node', anonymous=True)
        self.detection_pub = rospy.Publisher('yolov5_detections', Float32MultiArray, queue_size=10)
        self.object_name_pub = rospy.Publisher('yolov5_object_names', String, queue_size=10)
        self.image_pub = rospy.Publisher('yolov5_detection_image', Image, queue_size=10)
        self.rate = rospy.Rate(100)  # 发布频率 100Hz

        # CvBridge 用于 OpenCV 图像和 ROS 图像消息之间的转换
        self.bridge = CvBridge()

        # YOLOv5 模型加载
        self.model_path = "/home/yyros/yolov5/yolov5s.pt"  #  yolov5s.pt 文件路径
        self.yolov5_path = "/home/yyros/yolov5" #  yolov5 代码仓库路径
        self.conf_thres = 0.5  # 置信度阈值
        self.iou_thres = 0.45  # IOU 阈值

        try:
            self.model = torch.hub.load(self.yolov5_path, 'custom', path=self.model_path, source='local')
        except Exception as e:
            rospy.logerr("Failed to load YOLOv5 model: %s", str(e))
            exit(1)

        self.cap = cv2.VideoCapture(0) # 默认摄像头

        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open video source.")
            exit(1)
        else:
            rospy.loginfo("Video source opened successfully.")

        # 定义用于不同类别的颜色列表 (RGB 格式)
        self.colors = [
            (0, 255, 0),       # 绿色
            (255, 0, 0),     # 蓝色
            (0, 0, 255),     # 红色
            (255, 255, 0),   # 黄色
            (255, 0, 255),   # 紫色
            (0, 255, 255),   # 青色
            (192, 192, 192), # 银色
            (255, 105, 180), # 热粉色
            (128, 0, 128),   # 紫红色
            (0, 128, 128),   # 青绿色
        ] 


    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Video frame empty, exiting.")
                break

            # YOLOv5 推理
            results = self.model(frame)

            # 解析结果、发布检测信息和绘制 bounding box
            frame_with_boxes = self.publish_detections(results, frame)

            # 发布带有 bounding box 的图像 ROS 话题
            try:
                image_message = self.bridge.cv2_to_imgmsg(frame_with_boxes, "bgr8")
                self.image_pub.publish(image_message)
            except Exception as e:
                rospy.logerr("Error converting image message: %s", str(e))

            # 显示带有 bounding box 的图像 (OpenCV 窗口显示)
            cv2.imshow('YOLOv5 Detections', frame_with_boxes)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()


    def publish_detections(self, results, frame):
        detections_array = Float32MultiArray()
        object_names_str = ""
        detections = results.xyxy[0].cpu().numpy()

        for *xyxy, conf, cls in detections:
            if conf >= self.conf_thres:
                x_min, y_min, x_max, y_max = map(int, xyxy)
                class_id = int(cls)
                class_name = self.model.names[class_id]

                # 从颜色列表中选择颜色，循环使用颜色列表
                color = self.colors[class_id % len(self.colors)] # 关键修改:  根据类别索引选择颜色

                # 绘制彩色 bounding box
                label = f'{class_name} {conf:.2f}'
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2) # 使用选定的颜色
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2) # 使用选定的颜色

                # 创建检测数据
                detection_data = [float(x_min), float(y_min), float(x_max), float(y_max), float(conf), float(class_id)]
                detections_array.data.extend(detection_data)
                object_names_str += class_name + ","

        # 发布检测数据和物体名称
        self.detection_pub.publish(detections_array)
        self.object_name_pub.publish(String(data=object_names_str))

        if len(detections_array.data) > 0:
            rospy.loginfo("Published detections: %s", str(detections_array.data))
            rospy.loginfo("Published object names: %s", object_names_str)

        return frame


if __name__ == '__main__':
    try:
        yolov5_ros = YOLOv5ROS()
        yolov5_ros.run()
    except rospy.ROSInterruptException:
        pass
