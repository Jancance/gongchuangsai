# 2025年工创赛智能物流搬运赛道--西安理工大学塔吊结构--视觉部分开源
# 时间：2025/8/14 版本号：1.0

#这里是一些要下的库，大部分直接pip就能下下来
import cv2
import matplotlib as plt
import numpy as np
import serial as ser
import serial
import threading
import serial.tools.list_ports
from pyzbar.pyzbar import decode

# 从这里开始到93行都是卡尔曼滤波算法的初始化步骤
kalman = cv2.KalmanFilter(2, 2)
kalman_2 = cv2.KalmanFilter(2, 2)
kalman_3 = cv2.KalmanFilter(2, 2)

kalman.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_2.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_3.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)

kalman.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_2.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_3.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)

kalman.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3
kalman_2.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3
kalman_3.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3

kalman.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01
kalman_2.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01
kalman_3.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01

kalman.statePre = np.array([[6], [6]], np.float32)
kalman_2.statePre = np.array([[6], [6]], np.float32)
kalman_3.statePre = np.array([[6], [6]], np.float32)

# 初始化变量
last_measurement = current_measurement = np.array((2, 2), np.float32)
last_prediction = current_prediction = np.array((2, 2), np.float32)

last_measurement_2 = current_measurement_2 = np.array((2, 2), np.float32)
last_prediction_2 = current_prediction_2 = np.array((2, 2), np.float32)

last_measurement_3 = current_measurement_3 = np.array((2, 2), np.float32)
last_prediction_3 = current_prediction_3 = np.array((2, 2), np.float32)


def kalman_filter(measured_value):
    global kalman, last_measurement, current_measurement, last_prediction, current_prediction

    last_measurement = current_measurement
    last_prediction = current_prediction

    # 更新测量值
    kalman.correct(measured_value)

    # 预测下一个状态
    current_prediction = kalman.predict()

    return current_prediction


def kalman_filter_2(measured_value):
    global kalman_2, last_measurement_2, current_measurement_2, last_prediction_2, current_prediction_2

    last_measurement_2 = current_measurement_2
    last_prediction_2 = current_prediction_2

    # 更新测量值
    kalman_2.correct(measured_value)

    # 预测下一个状态
    current_prediction_2 = kalman_2.predict()

    return current_prediction_2


def kalman_filter_3(measured_value):
    global kalman_3, last_measurement_3, current_measurement_3, last_prediction_3, current_prediction_3

    last_measurement_3 = current_measurement_3
    last_prediction_3 = current_prediction_3

    # 更新测量值
    kalman_3.correct(measured_value)

    # 预测下一个状态
    current_prediction_3 = kalman_3.predict()

    return current_prediction_3

# 定义串口接收和串口发送数组以及标志码unit任务码unit_target
receive = [0, 0, 0, 0]
send = [0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
unit = 0
unit_target = 1

fps = 0

# 创建串口进程函数
def uart_process():
    global receive  # receive the MCU's uart
    global number
    global unit  # mean the scenario of the map
    global unit_target  # mean the target of the scenario
    k = 0

    while True:
        input = ser.read(4)
        com_input = list(input)
        if com_input:  # 如果读取结果非空，则输出
            print(com_input)
            try:
                receive[0] = int(com_input[0])
                receive[1] = int(com_input[1])
                receive[2] = int(com_input[2])
                receive[3] = int(com_input[3])
            except:
                receive = [0, 0, 0, 0]
            print(receive)
            if (receive[0] == 102):
                unit = receive[1]
                unit_target = receive[2]
            else:
                None

# 由于在Jetson Nano中使用的串口设备是CH340所以这里直接设置为这个设备而不是COM口这里需要你们自己配置成自己的串口设备号
ser = serial.Serial(port="/dev/ttyCH341USB0", baudrate=115200, timeout=0.05)
#ser = serial.Serial(port="COM7" , baudrate=115200, timeout=0.05)
serial_thread = threading.Thread(target=uart_process)
serial_thread.daemon = True
serial_thread.start()

def color_blocks_position_WL (img,color,size_code):
    """
    根据命令借助HSV颜色阈值筛选识别指定的物料
    :param img: 输入的图像
    :param color: 要追踪的颜色目标对象（RGB）
    :param size_code: 颜色色块面积限幅（用于灵活筛选追踪颜色大小）
    :return: 相应色块的中心坐标
    """
    # 定义颜色识别阈值（HSV）
    color_dist = {'red': {'Lower1': np.array([156, 60, 60]), 'Upper1': np.array([180, 255, 255]),'Lower2': np.array([0, 60, 60]), 'Upper2': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 100, 45]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([38, 80, 45]), 'Upper': np.array([90, 255, 255])},
              }
    ball_color = color
    if img is not None:
        gs_img = cv2.GaussianBlur(img, (5, 5), 0)                     # 高斯模糊
        hsv_img = cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
        erode_hsv = cv2.erode(hsv_img, None, iterations=2)                   # 腐蚀 粗的变细（平整边缘）
        inRange_hsv = None
        # 红色在HSV空间阈值有两部分所以需要特殊处理
        if (ball_color == 'red'):
            inRange_hsv1 = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower1'], color_dist[ball_color]['Upper1'])
            res1 = cv2.bitwise_and(erode_hsv, erode_hsv, mask=inRange_hsv1)
            inRange_hsv2 = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower2'], color_dist[ball_color]['Upper2'])
            res2 = cv2.bitwise_and(erode_hsv, erode_hsv, mask=inRange_hsv2)
            inRange_hsv = inRange_hsv1 + inRange_hsv2
        else:
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
        cv2.imshow("end2",erode_hsv)
        cv2.imshow("end",inRange_hsv)
        cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]# 找角点
        c = max(cnts, key=cv2.contourArea)# 筛选面积最大的色块
        size = int(cv2.contourArea(c))
        print(size)
        if (size > size_code): # 面积限幅
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            cv2.drawContours(img0, [np.int0(box)], -1, (0, 255, 255), 2)
            center_x, center_y = rect[0]
            return (int(center_x), int(center_y)) # 返回相应色块的中心坐标
        else:
            pass
    else:
        print("无画面")


def color_circle_position(img):
    """
    色环识别辅助实现定位
    :param img: 输入的图像
    :return: 三个色环的中心坐标 x1,y1,x2,y2,x3,y3
    """
    erode_hsv = cv2.erode(img, None, iterations=2)  # 腐蚀 粗的变细
    kernel = np.ones((7, 7), np.uint8)  # 5,5
    diRange_hsv = cv2.dilate(erode_hsv, kernel, 1)  # 膨胀 填补空洞
    gray_img = cv2.cvtColor(diRange_hsv, cv2.COLOR_BGR2GRAY)  # 转化为单通道灰度图

    # 限制对比度自适应直方图均衡，非常好的增强对光线鲁棒性的方法，但是阈值过大容易出现噪点
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))  # 5.0
    clahed = clahe.apply(gray_img)  # 对灰度图做 CLAHE 均衡

    # 计算形态学梯度（增强物体边缘）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    gradient = cv2.morphologyEx(gray_img, cv2.MORPH_GRADIENT, kernel)

    result = cv2.GaussianBlur(gradient, (7, 7), 3, 3)  # 高斯模糊 平滑边缘

    eqal_img = cv2.convertScaleAbs(result, alpha=4, beta=0)  # 再整体增强对比度
    cv2.imshow("video2", eqal_img)
    eqal_img = cv2.GaussianBlur(eqal_img, (7, 7), 3, 3)  # 再一次高斯模糊 平滑边缘

    retval, threshold_img = cv2.threshold(eqal_img, 70, 255, cv2.THRESH_BINARY)  # 二值化

    threshold_img = cv2.GaussianBlur(threshold_img, (9, 9), 3, 3)  # 最后再来一次高斯模糊 平滑边缘

    # canny_img = cv2.Canny(gradient,120,200)

    # diRange_img = cv2.dilate(canny_img, kernel, 1)

    # 霍夫圆检测
    circles = cv2.HoughCircles(threshold_img, cv2.HOUGH_GRADIENT_ALT, 1.5, 50, param1=100, param2=0.95, minRadius=15,
                               maxRadius=50)
    cv2.imshow("video", gray_img)

    cv2.imshow("video3", threshold_img)

    try:
        if (len(circles[0]) == 3):
            circles = np.uint16(np.around(circles))
            # 遍历
            for circle in circles[0, :]:
                cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
                cv2.circle(img, (circle[0], circle[1]), 2, (255, 0, 0), 2)
            circle_all = [circles[0][0], circles[0][1], circles[0][2]]
            circle_list = sorted(circle_all, key=lambda x: x[0])
            return (circle_list[0][0], circle_list[0][1], circle_list[1][0], circle_list[1][1], circle_list[2][0],
                    circle_list[2][1])  # 依次返回色环的中心坐标
    except:
        pass


def Judgeposition(qx1, qx2, qx3):
    """
    决赛时在粗加工区识别三种颜色物料并按照X坐标大小排序
    :param qx1 qx2 qx3:输入识别到的三种颜色物料坐标
    :return: 按照X排序的索引
    """
    sorted_indices = sorted([0, 1, 2], key=lambda i: [qx1, qx2, qx3][i])  # 利用sorted根据索引x值进行排序索引值
    return sorted_indices

def Judgeorder(qx1, qx2, qx3):
    """
    决赛时在粗加工区根据排序后的索引确定从左到右的颜色顺序
    :param qx1 qx2 qx3:输入识别到的三种颜色物料坐标
    :return: 颜色顺序码
    """
    sorted_indices = Judgeposition(qx1, qx2, qx3)
    # 根据排序后的索引确定颜色顺序
    if sorted_indices == [2, 1, 0]:  # qx1 < qx2 < qx3
        return 0x01  # RGB
    elif sorted_indices == [2, 0, 1]:  # qx1 < qx3 < qx2
        return 0x02  # RBG
    elif sorted_indices == [1, 2, 0]:  # qx2 < qx1 < qx3
        return 0x03  # GRB
    elif sorted_indices == [1, 0, 2]:  # qx2 < qx3 < qx1
        return 0x04  # GBR
    elif sorted_indices == [0, 2, 1]:  # qx3 < qx1 < qx2
        return 0x05  # BRG
    elif sorted_indices == [0, 1, 2]:  # qx3 < qx2 < qx1
        return 0x06  # BGR


def display(img, bbox):
    """
    在图像上绘制二维码的边界框和中心点
    :param img: 输入图像
    :param bbox: 边界框坐标
    """
    if bbox is None:
        print("边界框为空，无法绘制")
        return

    # 将边界框坐标转换为整数类型
    bbox = bbox.astype(int)
    # 打印边界框坐标
    print("Boundary Box Coordinates:", bbox)

    # 确保 bbox 的形状是 (4, 2)
    if len(bbox) == 1:
        bbox = bbox[0]

    # 绘制边界框
    n = len(bbox)
    for j in range(n):
        pt1 = tuple(bbox[j])
        pt2 = tuple(bbox[(j + 1) % n])
        cv2.line(img, pt1, pt2, (255, 0, 0), 3)

    # 计算边界框的中心点
    center_x = int(np.mean(bbox[:, 0]))
    center_y = int(np.mean(bbox[:, 1]))

    # 绘制中心点
    cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)

if __name__ == "__main__":
    cap = cv2.VideoCapture(0) # 主函数默认使用摄像头1

    while True:
        success, img0 = cap.read()

        if success:
            if (unit == 1):
                # 物料盘识别定位返回三色块中心坐标
                try:
                    poz_x1, poz_y1 = color_blocks_position_WL(img0, 'red', 2000)
                    poz_x2, poz_y2 = color_blocks_position_WL(img0, 'green', 2000)
                    poz_x3, poz_y3 = color_blocks_position_WL(img0, 'blue', 2000)
                    poz_x0 = int((poz_x1 + poz_x2 + poz_x3) / 3)
                    poz_y0 = int((poz_y1 + poz_y2 + poz_y3) / 3)
                    measured_x = poz_x0
                    measured_y = poz_y0
                    z = np.array([[measured_x], [measured_y]], dtype=np.float32)
                    x = kalman_filter(z)
                    qx0 = int(x[0][0])
                    qy0 = int(x[1][0])
                    send[1] = 0x01
                    send[2] = (qx0 & 0xff00) >> 8
                    send[3] = (qx0 & 0xff)
                    send[4] = (qy0 & 0xff00) >> 8
                    send[5] = (qy0 & 0xff)
                    send[6] = 0x77
                    
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(send)
                except:
                    pass

            elif (unit == 2):
                # 按照串口接收的任务码通过颜色筛选相应的物料，返回物料中心坐标
                try:
                    if (unit_target == 1):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'red', 4000)
                        send[2] = 0x01
                    elif (unit_target == 2):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'green', 4000)
                        send[2] = 0x02
                    elif (unit_target == 3):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'blue', 4000)
                        send[2] = 0x03
                    else:
                        pass
                    measured_x = poz_x0
                    measured_y = poz_y0
                    z = np.array([[measured_x], [measured_y]], dtype=np.float32)
                    x = kalman_filter(z)
                    qx0 = int(x[0][0])
                    qy0 = int(x[1][0])

                    send[1] = 0x02

                    send[3] = (qx0 & 0xff00) >> 8
                    send[4] = (qx0 & 0xff)
                    send[5] = (qy0 & 0xff00) >> 8
                    send[6] = (qy0 & 0xff)
                    send[7] = 0x77
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(send)
                except:
                    pass

            elif (unit == 3):
                # 色环定位：识别三个色环返回中心色环中心以及左右色环Y差值做定位
                try:
                    poz_x1, poz_y1, poz_x2, poz_y2, poz_x3, poz_y3 = color_circle_position(img0)
                    measured_x1 = poz_x1
                    measured_y1 = poz_y1

                    measured_x2 = poz_x2
                    measured_y2 = poz_y2

                    measured_x3 = poz_x3
                    measured_y3 = poz_y3

                    z1 = np.array([[measured_x1], [measured_y1]], dtype=np.float32)
                    x1 = kalman_filter(z1)
                    qx1 = int(x1[0][0])
                    qy1 = int(x1[1][0])

                    z2 = np.array([[measured_x2], [measured_y2]], dtype=np.float32)
                    x2 = kalman_filter_2(z2)
                    qx2 = int(x2[0][0])
                    qy2 = int(x2[1][0])

                    z3 = np.array([[measured_x3], [measured_y3]], dtype=np.float32)
                    x3 = kalman_filter_3(z3)
                    qx3 = int(x3[0][0])
                    qy3 = int(x3[1][0])

                    send[1] = 0x03
                    
                    send[2] = (qx2 & 0xff00) >> 8
                    send[3] = (qx2 & 0xff)
                    send[4] = (qy2 & 0xff00) >> 8
                    send[5] = (qy2 & 0xff)
                    send[6] = ((qy1 - qy3) & 0xff00) >> 8
                    send[7] = ((qy1 - qy3) & 0xff)
                    send[8] = 0x77
                    print(qy1 - qy3)
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(send)
                except:
                    pass

            elif (unit == 4):
                # 第二轮暂存区码垛时借助物料做位置校准
                try:
                    poz_x1, poz_y1 = color_blocks_position_WL(img0, 'blue', 3000)
                    poz_x2, poz_y2 = color_blocks_position_WL(img0, 'green', 3000)
                    poz_x3, poz_y3 = color_blocks_position_WL(img0, 'red', 3000)

                    measured_x1 = poz_x1
                    measured_y1 = poz_y1

                    measured_x2 = poz_x2
                    measured_y2 = poz_y2

                    measured_x3 = poz_x3
                    measured_y3 = poz_y3

                    z1 = np.array([[measured_x1], [measured_y1]], dtype=np.float32)
                    x1 = kalman_filter(z1)
                    qx1 = int(x1[0][0])
                    qy1 = int(x1[1][0])

                    z2 = np.array([[measured_x2], [measured_y2]], dtype=np.float32)
                    x2 = kalman_filter_2(z2)
                    qx2 = int(x2[0][0])
                    qy2 = int(x2[1][0])

                    z3 = np.array([[measured_x3], [measured_y3]], dtype=np.float32)
                    x3 = kalman_filter_3(z3)
                    qx3 = int(x3[0][0])
                    qy3 = int(x3[1][0])
                    
                    send[1] = 0x04
                    
                    send[2] = (qx2 & 0xff00) >> 8
                    send[3] = (qx2 & 0xff)
                    send[4] = (qy2 & 0xff00) >> 8
                    send[5] = (qy2 & 0xff)
                    send[6] = ((qy1 - qy3) & 0xff00) >> 8
                    send[7] = ((qy1 - qy3) & 0xff)
                    send[8] = 0x77
                    print(qx2, qy2)
                    print(qy1 - qy3)
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(send)

                except:
                    pass

            elif (unit == 5):
                # 决赛物料盘定位，这里直接使用圆环识别定位中心（可选）
                try:
                    poz_x1, poz_y1, poz_x2, poz_y2, poz_x3, poz_y3 = color_circle_position(img0)
                    poz_x0 = int((poz_x1 + poz_x2 + poz_x3) / 3)
                    poz_y0 = int((poz_y1 + poz_y2 + poz_y3) / 3)
                    measured_x = poz_x0
                    measured_y = poz_y0
                    z = np.array([[measured_x], [measured_y]], dtype=np.float32)
                    x = kalman_filter(z)
                    qx0 = int(x[0][0])
                    qy0 = int(x[1][0])
                    send[1] = 0x05
                    
                    send[2] = (qx0 & 0xff00) >> 8
                    send[3] = (qx0 & 0xff)
                    send[4] = (qy0 & 0xff00) >> 8
                    send[5] = (qy0 & 0xff)
                    send[6] = 0x77
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(send)
                except:
                    pass

            elif (unit == 8):
                # 决赛按从左到右颜色顺序识别抓取，在粗加工区识别三个地上的物料返回物料颜色顺序发送给下位机直接抓取
                try:
                    poz_x1, poz_y1 = color_blocks_position_WL(img0, 'blue',2000)
                    poz_x2, poz_y2 = color_blocks_position_WL(img0, 'green',2000)
                    poz_x3, poz_y3 = color_blocks_position_WL(img0, 'red',2000)

                    measured_x1 = poz_x1
                    measured_y1 = poz_y1

                    measured_x2 = poz_x2
                    measured_y2 = poz_y2

                    measured_x3 = poz_x3
                    measured_y3 = poz_y3

                    z1 = np.array([[measured_x1], [measured_y1]], dtype=np.float32)
                    x1 = kalman_filter(z1)  # 卡尔曼
                    qx1 = int(x1[0][0])
                    qy1 = int(x1[1][0])

                    z2 = np.array([[measured_x2], [measured_y2]], dtype=np.float32)
                    x2 = kalman_filter_2(z2)
                    qx2 = int(x2[0][0])
                    qy2 = int(x2[1][0])

                    z3 = np.array([[measured_x3], [measured_y3]], dtype=np.float32)
                    x3 = kalman_filter_3(z3)
                    qx3 = int(x3[0][0])
                    qy3 = int(x3[1][0])

                    sorted_indices = Judgeposition(qx1, qx2, qx3)  # 获取排序后的索引
                    order = Judgeorder(qx1, qx2, qx3)  # 获取颜色顺序码
                    print(qx1, qx2, qx3)#BGR顺序
                    print(order)

                    # 根据排序后的索引获取中间位置的 y 坐标
                    middle_y = [qy1, qy2, qy3][sorted_indices[1]]
                    middle_x = [qx1, qx2, qx3][sorted_indices[1]]

                    # 计算左边减去右边的 y 差值
                    left_y = [qy1, qy2, qy3][sorted_indices[0]]
                    right_y = [qy1, qy2, qy3][sorted_indices[2]]
                    y_diff = left_y - right_y

                    send[1] = 0x08
                    
                    send[2] = (middle_x & 0xff00) >> 8
                    send[3] = (middle_x & 0xff)
                    send[4] = (middle_y & 0xff00) >> 8
                    send[5] = (middle_y & 0xff)
                    send[6] = (y_diff & 0xff00) >> 8
                    send[7] = (y_diff & 0xff)
                    send[8] = order
                    send[9] = 0x77
                    FH = bytearray(send)
                    write_len = ser.write(FH)
                    print(qx1, qx2, qx3)#BGR顺序
                    print(order)
                    print(send)
                    fps = fps + 1
                    #print(fps)

                except:
                    pass

            elif (unit == 9):
                # OpenCV二维码检测器
                qrDecoder = cv2.QRCodeDetector()

                # 打开二号摄像头
                cap2 = cv2.VideoCapture(2)
                if not cap2.isOpened():
                    print("无法打开摄像头")

                else:
                    try:
                        # 从摄像头读取一帧图像
                        success, img0 = cap2.read()
                        if not success:
                            print("无法读取摄像头图像")
                            continue

                        # 检测并解码二维码
                        # 使用 detectAndDecode 方法，它会返回三个值：数据、边界框和矫正后的图像
                        # 但我们只需要前两个
                        data, bbox, _ = qrDecoder.detectAndDecode(img0)

                        # 复制原始图像用于绘制
                        # display_img = img0.copy()

                        if data and bbox is not None:
                            print(f"检测到二维码: 数据 = '{data}'")

                            # 在画面上绘制边界框和中心点
                            display(img0, bbox)
                            cv2.imshow("Results", img0)

                            send[1] = 0x09
                            
                            send[2] = int(data[0])
                            send[3] = int(data[1])
                            send[4] = int(data[2])
                            send[5] = int(data[4])
                            send[6] = int(data[5])
                            send[7] = int(data[6])
                            send[8] = 0x77
                            print(send)
                            FH = bytearray(send)
                            write_len = ser.write(FH)

                        else:
                            print("未检测到二维码")
                            cv2.imshow("Results", img0)
                    except:
                        print("error")
                    cap2.release()
                    cv2.destroyAllWindows()

            else:
                send[1] = 0xAA
                send[2] = 0X03
                send[3] = 0x77
                FH = bytearray(send)
                write_len = ser.write(FH)
                print(send)
            cv2.imshow("videoo", img0)
        else:
            break

        m_key = cv2.waitKey(1) & 0xFF

        if m_key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
