import time

import cv2
import numpy as np
from robomaster import robot
import threading
import queue

l_h, l_s, l_v, u_h, u_s, u_v = 0, 0, 0, 0, 0, 0

avg_x = [0] * 5
avg_y = [0] * 5
avg_r = [0] * 5


def nothing(x):
    pass


# 创建HSV控制条
cv2.namedWindow('tracking')
cv2.createTrackbar('LH', 'tracking', 0, 255, nothing)
cv2.createTrackbar('LS', 'tracking', 0, 255, nothing)
cv2.createTrackbar('LV', 'tracking', 105, 255, nothing)

cv2.createTrackbar('UH', 'tracking', 255, 255, nothing)
cv2.createTrackbar('US', 'tracking', 74, 255, nothing)
cv2.createTrackbar('UV', 'tracking', 255, 255, nothing)
cv2.moveWindow('tracking', 1000, 0)


def hsv_img(image):
    global l_h, l_s, l_v, u_h, u_s, u_v
    l_h = cv2.getTrackbarPos('LH', 'tracking')
    l_s = cv2.getTrackbarPos('LS', 'tracking')
    l_v = cv2.getTrackbarPos('LV', 'tracking')

    u_h = cv2.getTrackbarPos('UH', 'tracking')
    u_s = cv2.getTrackbarPos('US', 'tracking')
    u_v = cv2.getTrackbarPos('UV', 'tracking')

    low_b = np.array([l_h, l_s, l_v])
    up_b = np.array([u_h, u_s, u_v])
    result = cv2.inRange(image, low_b, up_b)
    return result


def get_ball(mask_img):
    global avg_x, avg_r, avg_y
    circles = cv2.HoughCircles(mask_img, cv2.HOUGH_GRADIENT, 2, 10, param1=120, param2=20, minRadius=20, maxRadius=100)
    if circles is not None:
        # 1 获取圆的圆心和半径
        # 使用remove_shake简单处理下抖动
        x = remove_shake(avg_x, int(circles[0][0][0]))
        y = remove_shake(avg_y, int(circles[0][0][1]))
        r = remove_shake(avg_r, int(circles[0][0][2]))
        print('x:', x, 'y:', y, 'r:', r)
        return [x, y, r]

    return False


# 高级到方法不会
# 小学二年级平均法去抖动
def remove_shake(lst, value):
    lst.pop(0)
    lst.append(value)
    return int(np.mean(lst))


# 获取点阵屏相对位置的字符串数据
def get_pic8_8String(x, y):
    x = int(8 * x)
    y = int(8 * y)
    pic8_8 = ['0'] * 64
    index = y * 8 + x
    pic8_8[index] = 'r'
    pic64 = ''.join(pic8_8)
    return pic64


def start_video():
    # 摄像头 m1芯片下h264库有问题改为通过udp获取视频
    tl_camera = tl_drone.camera
    tl_camera.start_video_stream(display=True)
    cap = cv2.VideoCapture('udp://192.168.10.2:11111?overrun_nonfatal=1&fifo_size=50000000')
    # cap = cv2.VideoCapture('udp://192.168.10.2:11111')
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


def control_tello(frame, x, y, size):
    # 更具圆心位置控制无人机运动
    width = frame.shape[1]
    height = frame.shape[0]

    # 根据圆心位置计算与中心点偏差
    speed_r = (width / 2 - x) // 5
    speed_y = (height / 2 - y) // 5
    # 目标距离
    target_size = 40
    speed_fw = target_size - size
    # 子线程控制姿态
    flight_P = threading.Thread(target=tl_flight.rc, args=(0, speed_fw//2, speed_y, -speed_r,))
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

# 配置led
tl_led = tl_drone.led
tl_led.set_mled_graph('0')

# 起飞
tl_flight = tl_drone.flight
tl_flight.takeoff().wait_for_completed(timeout=5)
tl_flight.rc(0, 0, 0, 0)
print('起飞完成')

# # 子线程开启摄像头 否则一边获取视频数据一边处理会延迟
video_flag = True
# q = queue.Queue()
# p1 = threading.Thread(target=start_video)
# p1.start()
cap = cv2.VideoCapture('udp://192.168.10.2:11111')
print('摄像头开启完成')

while True:
    # if q.empty():
    #     continue
    # frame = q.get()
    ret, frame = cap.read()
    if not ret:
        continue
    # 尺寸缩小
    shape = frame.shape
    frame = cv2.resize(frame, dsize=(shape[1] // 2, shape[0] // 2))
    # 水平反转
    # frame = cv2.flip(frame, 1, dst=None)
    # hsv处理图像
    hsvImg = hsv_img(frame)

    # 标记圆
    data = get_ball(hsvImg)
    if data:
        cv2.circle(frame, (data[0], data[1]), data[2], (255, 0, 255), 5)
        cv2.circle(frame, (data[0], data[1]), 10, (255, 0, 0), -1)
        # 点阵屏显示相对数据
        # led_str = get_pic8_8String(data[0] / frame.shape[1], data[1] / frame.shape[0])
        # # set_mled_graph是同步方法，有时候会卡，so子线程发送数据
        # led_p = threading.Thread(target=tl_led.set_mled_graph, args=(led_str,))
        # led_p.start()
        control_tello(frame, data[0], data[1], data[2])

    cv2.imshow('hsvImg', hsvImg)
    cv2.moveWindow("hsvImg", 500, 0)
    cv2.imshow('img', frame)
    key = cv2.waitKey(1)

    if ord('q') == key:
        print('lh', l_h, 'ls', l_s, 'lv', l_v)
        print('uh', u_h, 'us', u_s, 'uv', u_v)
        break
    if ord('s') == key:
        cv2.imwrite('123.png',frame)
        break
video_flag = False
p1.join()
tl_flight.land()
tl_drone.close()
cv2.destroyAllWindows()
