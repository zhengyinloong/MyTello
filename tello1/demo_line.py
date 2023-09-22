import cv2
import numpy as np
from robomaster import robot
import threading
import queue


def start_video():
    # 开启摄像头q
    # m1芯片下h264库有问题 改为通过udp获取视频
    tl_camera = tl_drone.camera
    tl_camera.start_video_stream(display=True)
    # cap = cv2.VideoCapture('udp://192.168.10.1:11111?overrun_nonfatal=1&fifo_size=50000000')
    cap = cv2.VideoCapture('udp://192.168.10.1:11111')
    # cap = cv2.VideoCapture(0)
    print('开启摄像头')
    while True:
        ret, image = cap.read()
        if not ret:
            continue
        q.put(image)
        if not video_flag:
            print('关闭摄像头')
            tl_camera.stop_video_stream()
            cap.release()
            break


def control_tello(data):
    # 找到最后一行所有黑色位置
    line = np.where(data == 0)
    line = line[0]
    if len(line) > 0:
        # 黑色范围中位数与中间差值从而控制转向角度
        i = len(data) // 2 - line[len(line) // 2]
        i = i // 3
        print('control the tello fly ',i)
        # 子线程控制姿态
        flight_P = threading.Thread(target=tl_flight.rc, args=(0, 30, 0, -i,))
        flight_P.start()


# 连接无人机
robot.config.LOCAL_IP_STR = "192.168.10.2"
tl_drone = robot.Drone()
tl_drone.initialize()

# 获取飞机版本信息
drone_version = tl_drone.get_sdk_version()
print("Drone sdk version: {0}".format(drone_version))

# 获取电量
tl_battery = tl_drone.battery
print('电量', tl_battery.get_battery())


# 起飞
tl_flight = tl_drone.flight
tl_flight.takeoff().wait_for_completed(timeout=5)
tl_flight.rc(0, 0, 0, 0)
tl_flight.down(distance=60).wait_for_completed(timeout=5)


# 子线程开启摄像头 否则一边获取视频数据一边处理会延迟
video_flag = True
q = queue.Queue()
p1 = threading.Thread(target=start_video)
p1.start()


while True:
    if q.empty():
        continue
    frame = q.get()

    # 尺寸缩小
    shape = frame.shape
    frame = cv2.resize(frame, dsize=(shape[1] // 2, shape[0] // 2))
    # 水平反转
    # frame = cv2.flip(frame, 1, dst=None)
    # 转化为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 大津法二值化
    retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    # 膨胀，白区域变大
    dst = cv2.dilate(dst, None, iterations=2)
    # # 腐蚀，白区域变小 #
    dst = cv2.erode(dst, None, iterations=6)

    # 图像最后一行数据提交到无人机控制函数中进行处理

    control_tello(dst[-1])

    cv2.imshow('dst', dst)
    cv2.moveWindow("dst", 80, 400)
    cv2.imshow('img', frame)
    key = cv2.waitKey(1)

    if ord('q') == key:
        break
video_flag = False
tl_flight.land()
tl_drone.close()
cv2.destroyAllWindows()
