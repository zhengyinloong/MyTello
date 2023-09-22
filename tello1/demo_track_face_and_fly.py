import time
import cv2
import numpy as np
from robomaster import robot
import threading
import queue

avg_x = [0] * 5
avg_y = [0] * 5
avg_r = [0] * 5


def mark_face(image):
    g_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_detect.detectMultiScale(g_img, 1.2, minNeighbors=5, minSize=(20, 20))
    return faces
    # if faces is not None:
    #     return faces
    # else:
    #     return False


def start_video():
    # 摄像头 m1芯片下h264库有问题改为通过udp获取视频
    tl_camera = tl_drone.camera
    tl_camera.start_video_stream(display=True)
    cap = cv2.VideoCapture('udp://192.168.10.1:11111?overrun_nonfatal=1&fifo_size=50000000')
    # cap = cv2.VideoCapture('udp://192.168.10.1:11111')
    print('开启摄像头')
    while True:
        ret, image = cap.read()
        if not ret:
            continue
        # 只保留最近30帧图像，避免检测慢，造成较高的延迟
        if q.full():
            print('丢弃')
            q.get()
        q.put(image)
        if not video_flag:
            print('关闭摄像头')
            tl_camera.stop_video_stream()
            cap.release()
            break


def control_tello(flag, frame=None, x=None, y=None, size=None):
    if flag:
        # 更具人脸大小位置控制无人机运动
        width = frame.shape[1]
        height = frame.shape[0]

        # 根据人脸位置计算与中心点偏差
        speed_r = (width / 2 - x) // 5
        speed_y = (height / 2 - y) // 5
        # 目标距离
        target_size = 90
        speed_fw = target_size - size
    else:
        speed_fw, speed_y, speed_r = 0, 0, 0
    # 子线程控制姿态
    flight_P = threading.Thread(target=tl_flight.rc, args=(0, speed_fw, speed_y, -speed_r,))
    flight_P.start()


# 连接无人机
robot.config.LOCAL_IP_STR = "192.168.10.2"  # 192.168.10.3
tl_drone = robot.Drone()
tl_drone.initialize()

# 获取飞机版本信息
drone_version = tl_drone.get_sdk_version()
print("Drone sdk version: {0}".format(drone_version))

# 获取电量
tl_battery = tl_drone.battery
print('电量', tl_battery.get_battery())

# # 配置led
# tl_led = tl_drone.led
# tl_led.set_mled_graph('0')

# 起飞
tl_flight = tl_drone.flight
tl_flight.takeoff().wait_for_completed(timeout=5)
tl_flight.rc(0, 0, 0, 0)
print('起飞完成')

# 子线程开启摄像头 否则一边获取视频数据一边处理会延迟
video_flag = True
q = queue.Queue(maxsize=30)
p1 = threading.Thread(target=start_video)
p1.start()

# 加载官方人脸分类器
face_detect = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_alt2.xml')

while True:
    if q.empty():
        # print('空了')
        continue
    frame = q.get()

    # cap = cv2.VideoCapture(0)
    # ret, frame = cap.read()

    # 尺寸缩小
    shape = frame.shape
    frame = cv2.resize(frame, dsize=(shape[1] // 2, shape[0] // 2))
    # 水平反转
    # frame = cv2.flip(frame, 1, dst=None)

    # 人脸检测
    data = mark_face(frame)
    # print(len(data))
    if len(data) > 0:
        for x, y, w, h in data:
            # 标记人脸
            cv2.rectangle(frame, (x, y), (x + w, y + h), color=(0, 0, 255), thickness=5)
            control_tello(True, frame, (2 * x + w) // 2, (2 * y + h) // 2, (w + h) // 2)
            continue
    else:
        print('未检测到人脸')
        control_tello(False)

    cv2.imshow('img', frame)
    key = cv2.waitKey(1)
    if ord('q') == key:
        break
video_flag = False
p1.join()
tl_flight.land()
tl_drone.close()
cv2.destroyAllWindows()
