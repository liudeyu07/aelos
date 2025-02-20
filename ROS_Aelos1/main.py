import cv2

# 读取图像
image = cv2.imread("red.png")

# 颜色空间转换
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 定义红色范围（调整后的红色范围）
lower_red1 = (0, 40, 10)
upper_red1 = (12, 255, 255)
lower_red2 = (165, 40, 10)
upper_red2 = (180, 255, 255)

# 创建掩模
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask = cv2.add(mask1, mask2)

# 形态学处理，使用闭运算
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # 使用闭运算

# 寻找轮廓
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 筛选轮廓并绘制边界框
for contour in contours:
    area = cv2.contourArea(contour)
    if area > 1000:  # 设置面积阈值
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        print("小球坐标：", x, y)

# 显示结果
cv2.imshow("Original Image", image)
cv2.imshow("Mask", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()