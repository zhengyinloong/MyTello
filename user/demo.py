# -*- coding:utf-8 -*-
# demo.py in MyTello
# zhengyinloong
# 2023/9/22

"""
For track face
"""

from easytello import *

TELLO_IP = "192.168.10.2"

tello = EasyTello()
tello.get_battery()
tello.takeoff()
tello.track_face()
tello.land()

