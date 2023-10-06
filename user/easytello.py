# -*- coding:utf-8 -*-
# easytello.py in MyTello
# zhengyinloong
# 2023/9/22

import cv2
import numpy as np
from tello1.robomaster import robot
import threading
import queue

TELLO_IP = "192.168.10.2"


class EasyTello:

    def __init__(self):
        self.tello = None
        self.connect_tello()
        self.video_flag = False

    def connect_tello(self):
        # 连接无人机
        robot.config.LOCAL_IP_STR = TELLO_IP
        self.tello = robot.Drone()
        self.tello.initialize()
        self.flight = self.tello.flight
        self.camera = self.tello.camera
        self.battery = self.tello.battery
        self.target_size = 60

    def get_battery(self):
        # 获取电量
        battery_num = self.battery.get_battery()
        print('电量', battery_num)
        return battery_num

    def takeoff(self):
        # 起飞
        self.flight.takeoff().wait_for_completed(timeout=5)
        # self.flight.rc(0, 0, 0, 0)
        print('起飞完成')

    def land(self):
        self.flight.land()
        print('已着陆')


    def start_video(self):
        # 摄像头 m1芯片下h264库有问题改为通过udp获取视频
        self.camera.start_video_stream(display=True)
        cap = cv2.VideoCapture('udp://192.168.10.1:11111?overrun_nonfatal=1&fifo_size=50000000')
        # cap = cv2.VideoCapture('udp://192.168.10.1:11111')
        print('开启摄像头')
        while True:
            ret, image = cap.read()
            if not ret:
                continue
            # 只保留最近30帧图像，避免检测慢，造成较高的延迟
            if self.video_queue.full():
                print('丢弃')
                self.video_queue.get()
            self.video_queue.put(image)
            if not self.video_flag:
                print('关闭摄像头')
                self.camera.stop_video_stream()
                cap.release()
                break

    def track_face(self):
        def mark_face(image):
            g_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = face_detect.detectMultiScale(g_img, 1.2, minNeighbors=5, minSize=(20, 20))
            return faces
            # if faces is not None:
            #     return faces
            # else:
            #     return False

        # def control_tello(flag, frame=None, x=None, y=None, size=None):
        #     if flag:
        #         # 更具人脸大小位置控制无人机运动
        #         width = frame.shape[1]
        #         height = frame.shape[0]
        #
        #         # 根据人脸位置计算与中心点偏差
        #         speed_r = (width / 2 - x) // 5
        #         speed_y = (height / 2 - y) // 5
        #         # 目标距离
        #         target_size = self.target_size
        #         speed_fw = target_size - size
        #     else:
        #         speed_fw, speed_y, speed_r = 0, 0, 0
        #     # 子线程控制姿态
        #     flight_P = threading.Thread(target=self.flight.rc, args=(0, speed_fw, speed_y, -speed_r,))
        #     flight_P.start()

        def control_tello(flag, frame=None, x=None, y=None, size=None):
            if flag:
                # 更具人脸大小位置控制无人机运动
                width = frame.shape[1]
                height = frame.shape[0]

                # 根据人脸位置计算与中心点偏差
                speed_y = (width / 2 - x) // 8
                speed_z = (height / 2 - y) // 5
                # 目标距离
                target_size = self.target_size
                speed_x = (target_size - size)//2

                speed_z = 0
            else:
                speed_x, speed_y, speed_z = 0, 0, 0
            # 子线程控制姿态
            flight_P = threading.Thread(target=self.flight.rc, args=(-speed_y, speed_x, speed_z, 0,))
            flight_P.start()


        # 子线程开启摄像头 否则一边获取视频数据一边处理会延迟
        self.video_flag = True
        # 只保留最近30帧图像，避免检测慢，造成较高的延迟
        self.video_queue = queue.Queue(maxsize=30)
        track_face_thread = threading.Thread(target=self.start_video)
        track_face_thread.start()

        # 加载官方人脸分类器
        face_detect = cv2.CascadeClassifier('/home/loong/MyTello/tello1/haarcascades/haarcascade_frontalface_alt2.xml')

        while True:
            if self.video_queue.empty():
                # print('空了')
                continue
            frame = self.video_queue.get()

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
        self.video_flag = False
        track_face_thread.join()
        self.flight.land()
        self.tello.close()
        cv2.destroyAllWindows()
