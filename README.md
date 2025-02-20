# Aelos选拔任务

### 1 基本任务

1.1 对下列图像进行处理，通过python的opencv库函数实现将图像中的红色小球框选并输出其坐标



windows中运行ROS_Aelos1中的main.py：

![](image\1.1.png)



1.2  自主编写代码完成以下两个文件 img_pub.py（发布） 与 img_sub.py(订阅)，对[给定图像](https://ultramarine-image.oss-cn-beijing.aliyuncs.com/img/image-20221027164357622.png)进行处理

- img_pub.py文件通过cv_bridge获取摄像头图像并通过ros的publisher发布图像
- img_sub.py文件接收上面的图像话题（topic）， 然后进行图像处理， 结合第一次任务的代码， 还是提取摄像头中的红色物体（自己周围找一个就行，比如苹果之类的，阈值自己摸索调整）， 然后进行框选输出坐标与矩形形状。处理完图像后将新的图像进行发布
- 利用rqt_image工具查看处理完后输出的图像topic

在unbutu中在ros_task文件夹打开命令行，roscore启动后用roslaunch image_processing image_processing.launch运行代码，用rqt_image_view使用rqt_image工具查看处理完后输出的图像topic

![](image\1.2.1.png)

![](image\1.2.2.png)

### 2 进阶任务[](https://irctasks.readthedocs.io/zh-cn/latest/aelostask.html#id3)

***使用yolo***

- 跑通yolo识别框架，完成基础的图像识别功能
- yolo版本不限，能完成目标即可
- 将yolo识别到的物体在视频中的像素坐标通过ros话题发布

采用yolov5完成任务

在ROS\ros_task\src\yolov5_ros_noetic代码中完成相应的路径设置并调整适合的发布频率后运行rosrun yolov5_ros_noetic yolov5_ros_node.py

函数最后 `return frame`，返回经过绘制 bounding box 的图像帧，也可以使用rqt_image工具查看处理完后输出的图像topic

用rostopic echo /yolov5_detections查看发布的消息内容，包含检测到的物体信息和坐标。

![](image\2.1.png)

![](image\2.2.png)
