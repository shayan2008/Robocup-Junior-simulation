import sys
sys.path.append("C:\\Program Files\\Webots\\lib\\controller\\python")
import logging
import logging.config
import math
import os
import sqlite3
from tkinter import *
import pyfiglet
import sys
import time
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6 import QtWidgets
from PyQt6.QtCore import QFile
from PyQt6.uic import loadUi
from PyQt6.QtCore import QThread, pyqtSignal, Qt
import tkinter as tk
from simple_pid import PID
# to intall copy "pip install simple_pid" to your cmd
pidR = PID(1, 0.1, 0.05, setpoint=6.28)
pidL = PID(1, 0.1, 0.05, setpoint=6.28)
import struct
import time
import sys
import shutil
import traceback
from datetime import datetime
from colorama import Fore, Style
import pygame
from time import sleep
import random
from math import sin, cos, radians, sqrt, pi, atan
from queue import PriorityQueue
import cv2 as cv
import numpy as np
from controller import Robot # type: ignore
import pickle
from tkinter import Tk, Frame, Menu
import tkinter as tk
from pyfiglet import figlet_format
from colorama import Fore, Style, init
from threading import Thread

# **********************global vision settings**********************

models_path = '../../../../models'

TRAIN_WIDTH = 81
TRAIN_HEIGHT = 81

THRESHOLD_VALUE = 160
THRESHOLD_METHOD = cv.THRESH_BINARY

# _______aspect ratio_______#
MAX_ASPECT_RATIO = 2.2
MIN_ASPECT_RATIO = 0.8

MIN_AREA_THRESHOLD = 90
MAX_AREA_THRESHOLD = 80000000000
# ********************remaning tasks********************


        
patterns = {
        'H': [


              np.array([[1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 1, 1, 1, 1, 1, 1],
                        [1, 1, 1, 1, 1, 1, 1, 1, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1],
                        [1, 1, 1, 0, 0, 0, 0, 0, 1]])],

        "hh":[  np.array([[1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 1, 1, 1, 1, 1, 1],
                          [1, 1, 1, 1, 1, 1, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1],
                          [1, 1, 1, 0, 0, 0, 1, 1, 1]]),
        ],
        'hhh':[  np.array([ [0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0]]),
        ],
        'hhhh':[ np.array([ [0, 1, 1, 0, 0, 1, 1, 1, 1],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 0, 1, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0]]),

        ],
        'h5':[  np.array([  [0, 1, 1, 0, 0, 1, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 0, 1, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0]]),

        ],
        'h6':[  np.array([  [1, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0],
                            [0, 1, 1, 1, 1, 1, 1, 0, 0],
                            [0, 1, 1, 1, 1, 1, 1, 0, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0],
                            [0, 1, 1, 0, 0, 1, 1, 0, 0]]),

        ],
        'h7':[  np.array([  [1, 1, 1, 1, 1, 1, 1, 1, 1],
                            [0, 1, 1, 0, 0, 0, 0, 1, 1],
                            [0, 1, 1, 0, 0, 0, 0, 1, 1],
                            [0, 1, 1, 0, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 1, 1, 1, 1, 1],
                            [0, 1, 1, 1, 0, 1, 1, 1, 1],
                            [0, 1, 1, 0, 0, 0, 0, 1, 1],
                            [0, 1, 1, 0, 0, 0, 0, 1, 1],
                            [1, 1, 0, 0, 0, 0, 0, 1, 1]]),

        ],
        'h8':[   np.array([ [1, 1, 1, 1, 1, 1, 0, 0, 0],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 1, 1, 1, 1, 1],
                            [0, 1, 1, 1, 1, 1, 1, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1]]),

        ],

      
        'S': [ 

                np.array([  [0, 0, 0, 1, 1, 1, 1, 0, 0],
                            [0, 0, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 1, 1, 0, 0, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 0, 0, 0],
                            [0, 0, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 0, 0, 0, 1, 1, 1, 0],
                            [0, 0, 1, 1, 0, 0, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 0, 1, 1, 1, 1, 1, 0]])],

            "ss":[
                np.array([  [0, 0, 0, 1, 1, 1, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 1, 1, 0, 1, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 0, 0, 0],
                            [0, 0, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 0, 0, 1, 1, 1, 1, 0],
                            [0, 1, 1, 1, 0, 0, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 1, 0, 0]]),
        


        ],

        'sss':[ 
            np.array([  [0, 0, 1, 1, 1, 1, 0, 0, 0],
                        [0, 1, 1, 1, 1, 1, 1, 0, 0],
                        [0, 1, 1, 1, 0, 1, 1, 1, 0],
                        [0, 0, 0, 1, 1, 1, 1, 1, 0],
                        [0, 1, 1, 1, 1, 1, 1, 0, 0],
                        [0, 1, 1, 1, 1, 0, 1, 0, 0],
                        [0, 1, 1, 0, 0, 1, 1, 1, 0],
                        [0, 1, 1, 1, 1, 1, 1, 1, 0],
                        [0, 0, 1, 1, 1, 1, 1, 0, 0]]),

        ],

        
        
        'U': [ 
                np.array([  [1, 1, 1, 1, 0, 1, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 0, 0, 0, 1, 1, 1],
                            [1, 1, 1, 1, 0, 1, 1, 1, 1],
                            [1, 1, 1, 1, 1, 1, 1, 1, 1]]),


        ],
        'uu':[np.array([    [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 0, 1],
                            [1, 1, 1, 0, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 1, 1, 1, 1, 1],
                            [0, 0, 1, 1, 1, 1, 1, 1, 1]]),
        ],
        'uuu':[
                np.array([  [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 0, 0, 0, 1, 1, 0],
                            [0, 1, 1, 1, 0, 1, 1, 1, 0],
                            [0, 1, 1, 1, 1, 1, 1, 1, 0],
                            [0, 0, 1, 1, 1, 1, 1, 0, 0]]),

        ],
        'uuuu':[ np.array([ [0, 0, 0, 0, 1, 1, 1, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 0, 1, 1, 1, 1, 1, 1, 1]]),

        ],
        'u5':[  np.array([  [0, 0, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 0, 1, 1],
                            [0, 1, 1, 1, 0, 0, 1, 1, 1],
                            [0, 0, 1, 1, 1, 1, 1, 1, 1],
                            [0, 0, 0, 0, 1, 1, 1, 0, 0]]),

        ]
                    
    }


# **********************debug part**********************
current_directory = os.getcwd()

file_name = "log.log"

file_path = os.path.join(current_directory, file_name)

if os.path.exists(file_path) and os.path.isfile(file_path):
    os.remove(file_path)
    print(f"Deleted file: {file_path}")
else:
    pass


remaining_time_value = float('inf')
class RemainingTimeFilter(logging.Filter):
    def filter(self, record):
        global remaining_time_value
        record.remaining_time = remaining_time_value
        return True
        
# LOG_FILE = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.log")
logging.basicConfig(
    filename="log.log",
    filemode="a",
    format="%(levelname)s:%(asctime)s:::%(remaining_time)s:::%(message)s",
    level=logging.NOTSET
)
logging.getLogger().addFilter(RemainingTimeFilter())

logging.info("Program Started")



def exception_hook(type: Exception, value, tb):
    et = traceback.extract_tb(tb)
    traceback_text = "\n"
    for t in et:
        line = t.lineno
        if t.end_lineno != line:
            line = f"{line}:{t.end_lineno}"
        col = t.colno
        if t.end_colno != col:
            col = f"{col}:{t.end_colno}"
        traceback_text += f'File "{t.filename}", line {line}, col {col} in {t.name}\n'
        traceback_text += f'\t{t.line}\n'

    traceback_text += f'{type.__qualname__}: {value}'
    traceback_text += "\n"
    if issubclass(Warning, type):
        logging.warning(traceback_text)
    else:
        logging.error(traceback_text)

    print(traceback_text)


sys.excepthook = exception_hook

# # Define the filename
# filename = "numpy.txt"


# if not os.path.exists(filename):

#     with open(filename, 'w') as file:
#         file.close()
#     print(f"{filename} has been created.")
# else:
#     print(f"{filename} already exists.")


# **********************global variables**********************
mapping_size = 200
GRID_SIZE = 7
tilenum = 0
r2_map = []
x = mapping_size // 2
trap = 0
y = mapping_size // 2
x_max = x
max_velocity = 6.28*0.87
room = 1
gridam_count=0
noPathLOP_count=0
task=[]
avoid_status=[-1,-2,-3]
avoid_type=[-1,-2,2]
Astar_info=[]
vision_debug=False
forward_flag=1
runAstar=False
xpre = 100
ypre = 100
y_max = y
x_min = x
room4checkup=0
reminder=[]
rooms_chenged_in_whatroom=[]
cordinent = ()
lastx=100
lasty=100
multyply_cost=[]
y_min = y
LOPonTraget=[]
wheel_speed=()
base_k = 18
tryingtogetout_g=0
k_angel = 0.83
inform=[]
lopNoPathCount = 0
lopNoPathTargets = []
lastWhatRoomChecked = (0, 0)
PID_K=0.04
flage_pid=0
lastx=100
lasty=100
error_tooli=0
multyply_cost=[]
flag_tooli=0
lastVisitedCheckpoint = (x, y)
checkForFarTokens: bool = True
victimDetectionDistance: float = 8
flag = 0
inf = float("inf")
startp = (x, y)
off_or_on=0

"""

tile types:
0: not visited
1: normal tile
2: trap (hole)
3: swamp
4: checkpoint
6: blue
7: purple
8: red
9: green
10: orange
11: yellow


status:
-1: not visited
-2: can not visit (giridam)
1: visited
0: seen and can visit

"""

for i in range(mapping_size):
    bricks = []
    for j in range(mapping_size):
        bricks.append( 
            {"walls": np.ones([5, 5], dtype='int32'), "type": 0, "status": -1, "room": 0, "visit_count": 0,
             "victim": np.zeros([5, 5], dtype='U1'), "vic_pos": [],
             "walls count": np.zeros([5, 5], dtype="int32"), "HSU": "none"})
    r2_map.append(bricks)

# **********************devices**********************
robot = Robot()
wheel1 = robot.getDevice("wheel1 motor")
wheel2 = robot.getDevice("wheel2 motor")
enc1 = robot.getDevice("wheel1 sensor")
enc2 = robot.getDevice("wheel2 sensor")
wheel1.setPosition(float("inf"))
wheel2.setPosition(float("inf"))
timeStep = 8

# **********************enabling**********************
gps = robot.getDevice("gps")
emitter = robot.getDevice("emitter")
gyro = robot.getDevice("imu")
cs = robot.getDevice("colour_sensor")
lidar = robot.getDevice("lidar")
receiver = robot.getDevice("receiver")
camera1 = robot.getDevice("Rcam")
camera2 = robot.getDevice("Lcam")

class Report:

    def __init__(self,victim_type:str):
        self.victimtype = victim_type.capitalize()
        self.cord_list = []
    def append(self,cord:list):
        self.cord_list.append(cord)
    def rectify(self,new_vic:list):

        report_victim(self.victimtype)
H=Report("H")
S=Report("S")
U=Report("U")
P=Report("P")
O=Report("O")
C=Report("C")
F=Report("F")

DM = robot.getDevice("DM")
DR = robot.getDevice("DR")
DL = robot.getDevice("DL")
R45 = robot.getDevice("R45")
L45 = robot.getDevice("L45")

DM.enable(timeStep)
DR.enable(timeStep)
DL.enable(timeStep)
R45.enable(timeStep)
L45.enable(timeStep)

gps.enable(timeStep)
gyro.enable(timeStep)
cs.enable(timeStep)
enc1.enable(timeStep)
enc2.enable(timeStep)
lidar.enable(timeStep)
camera1.enable(timeStep)
camera2.enable(timeStep)
receiver.enable(timeStep)

logging.info("Initializing Completed.")


a_data = np.zeros([210, 210])


class Vision:
    def __init__(self):
        # self.model = svm.LinearSVC()
        self.conn = sqlite3.connect('../../../../datasets/wall.db')
        self.cur = self.conn.cursor()
        if not os.path.exists(models_path):
            os.mkdir(models_path)

    def create_table(self):
        self.cur.execute('CREATE TABLE IF NOT EXISTS letters(img BLOB, mean INTEGER, label TEXT)')

    def add_blob(self, blob, mean, label):
        self.cur.execute('INSERT INTO letters (img, mean, label) VALUES (?, ?, ?)', (blob, mean, label))
        self.conn.commit()

    def convertImgToBinary(self, file):
        with open(file, 'rb') as image:
            self.binary_image = image.read()
        return self.binary_image

    def convertBinaryToImg(self, binary_data, file_name):
        with open(file_name, 'wb') as image:
            image.write(binary_data)
    
    def resize(self, img_org, resize_shape):
        if type(img_org) != bool:
            block_x = int(img_org.shape[1] / resize_shape[1])
            block_y = int(img_org.shape[0] / resize_shape[0])

            new_image = np.ones((resize_shape[1], resize_shape[0], 3), dtype=np.uint8)

            for x in range(block_x):
                for y in range(block_y):
                    for c in range(3):
                        block_mean = int(np.mean(img_org[block_x*(x) : block_x*(x+1), block_y*(y) : block_y*(y+1), c]))
                        new_image[x, y, c] = block_mean

            return new_image
        return False

    def warpFrame(self, img_org, camera_side):
        global vision_debug
        height, width, channels = img_org.shape
        camera_num = 0
         
        if camera_side == 'L':
            camera_num = 1408
        elif camera_side == 'R':
            camera_num = 1152
        kernel = np.ones((3, 3), dtype=np.uint8)
        range_min = (90, 50, 20)
        range_max = (100, 175, 175)

        img_hsv = cv.cvtColor(img_org, cv.COLOR_BGR2HSV)
        img_threshold = cv.inRange(img_hsv, range_min, range_max)
        img_threshold = cv.bitwise_not(img_threshold)

        drawing = np.copy(img_org)
        image_rect_thresh=img_threshold
        image_rect=img_org
        contours, hierarchy = cv.findContours(img_threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Draw the contour on the blank 
        # vision_debug=True
        if vision_debug==True:
            for contour in contours:
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(image_rect, (x, y), (x + w, y + h), (0, 255, 0), 1)
            # Resize the image for display (e.g., scale it by 2x)
            scale_factor = 15  # Adjust as needed for the desired size
            height, width = image_rect.shape[:2]
            image_rect = cv.resize(image_rect, (width * scale_factor, height * scale_factor))
            # Show the result
            cv.imshow(f"Contours with Bounding Rectangles {camera_side}", image_rect)
            # Wait for a key press and close the window
            # cv.imshow("thersh",img_threshold)
            cv.waitKey(0)

        for contour in contours:
            contour_area = cv.contourArea(contour)
            if contour_area >= MIN_AREA_THRESHOLD and contour_area <= MAX_AREA_THRESHOLD:

                x, y, w, h = cv.boundingRect(contour)
                contour_center = x + (w / 2)
                img_center = img_org.shape[1] / 2

                aspect_ratio = abs(h) / abs(w)
           
                if vision_debug:
                    print(f"aspect_ratio: {aspect_ratio} y+h/2: {y+h/2} y+h: {y+h} x: {x} y: {y} x+w: {x+w} detection_side: {camera_side} w: {w} h: {h}")

                if aspect_ratio >= MIN_ASPECT_RATIO and aspect_ratio <= MAX_ASPECT_RATIO  and ((x>=1 and y >= 1 and x+w < 63 and y+h < 39 and y+h/2 > 10 and y+h/2 < 45) or ((x<11 and x+w>55 and h>20) or (y+h>30 and y<8 and w>25))) :
                    # logging.info(f"aspect_ratio: {aspect_ratio} y+h/2: {y+h/2} x: {x} y: {y} x+w: {x+w} detection_side: {camera_side}")
                    epsilon = 0.05*cv.arcLength(contour, False)
                    approx = cv.approxPolyDP(contour,epsilon,True)  
                    if len(approx) == 4:
                        sort_corner = sortContour(approx)

                        if vision_debug == True:
                            contour_img = img_org.copy()
                            cv.drawContours(contour_img, [sort_corner], 0, (255,255,0), 1)
                            scale_factor = 15 
                            contour_img = cv.resize(contour_img, (width * scale_factor, height * scale_factor))
                            cv.imshow("contor",contour_img)
                            cv.waitKey(0)

                        warp_array = np.array([[TRAIN_WIDTH, 0], [0,0], [0, TRAIN_HEIGHT], [TRAIN_WIDTH, TRAIN_HEIGHT]], dtype=np.float32)

                        centerDistancePixels = contour_center - img_center
                        centerDistanceDegree = centerDistancePixels / 64 * 60
                        center_distance = round(centerDistanceDegree * 512 / 360)

                        box = np.float32(sort_corner)
                        box_image = cv.getPerspectiveTransform(box, warp_array)

                        warped_window = cv.warpPerspective(img_org, box_image, (TRAIN_HEIGHT, TRAIN_HEIGHT))

                        # cv.imshow("warped win",warped_window)
                        # cv.waitKey(0)

                        return warped_window, center_distance
        return False, False

    def split_RGB(self, img_org, camera_side):
        if type(img_org) != bool:
            img_gray = cv.cvtColor(img_org, cv.COLOR_BGR2GRAY)
            img_mean = int(np.mean(img_gray))

            R = img_org[:,:,2]
            G = img_org[:,:,1]
            B = img_org[:,:,0]

            _,R = cv.threshold(R, 180, 255, cv.THRESH_BINARY)
            _,G = cv.threshold(G, 180, 255, cv.THRESH_BINARY)
            _,B = cv.threshold(B, 125, 255, cv.THRESH_BINARY)
            thresholds = np.hstack((R, G, B))


            return thresholds, img_mean
        else:
            return False, False

    def train(self, frame, mean):
        if type(frame) != bool:
            # print(frame.shape)
            cv.imshow('frame', frame)
            key = cv.waitKey(0)

            cv.imwrite("letter.jpg", frame)
            blob = self.convertImgToBinary("letter.jpg")

            if key == ord('h'):
                self.add_blob(blob, mean, 'H')

            elif key == ord('s'):
                self.add_blob(blob, mean, 'S')

            elif key == ord('u'):
                self.add_blob(blob, mean, 'U')

            elif key == ord('f'):
                self.add_blob(blob, mean, 'F')

            elif key == ord('p'):
                self.add_blob(blob, mean, 'P')

            elif key == ord('c'):
                self.add_blob(blob, mean, 'C')

            elif key == ord('o'):
                self.add_blob(blob, mean, 'O')

            elif key == ord('n'):
                self.add_blob(blob, mean, 'N')

            elif key == ord('q'):
                pass

    def predict(self, frame, mean, model):
        if type(frame) != bool:
            img = frame.reshape(243)
            img = np.append(img, mean)

            prediction = model.predict([img])
            return prediction[0]
        else:
            prediction = 'N'
            return prediction

    def save_model(self, model, file_name):
        pickle.dump(model, open(f'{file_name}.pkl', 'wb'))

    def load_model(self, file_name):
        return pickle.load(open(f'{file_name}.pkl', 'rb'))

    def close(self):
        try:
            os.remove('letter.jpg')
            os.remove('blob.jpg')
            os.remove('frame.jpg')

            cv.destroyAllWindows()
        except:
            pass
#********************todos_fixbug****************************
class remind():
    def __init__(self):
        pass
    def todo(self,input):
        global reminder
        reminder.append(f"todo => {input}")
    def bug(self,input):
        global reminder
        reminder.append(f"bug => {input}")
    def documentation(self ,input):
        global reminder
        reminder.append(f"documentation => {input}")
    def information(self,input):
        global inform
        inform.append(input)
    def Astar_information(self,input):
        global Astar_info
        Astar_info.append(input)
def bug(input):
    rem=remind()
    rem.bug(input)
def doc(input):
    rem=remind()
    rem.documentation(input)
def todo(input):
    rem=remind()
    rem.todo(input)
def info(input):
    rem=remind()
    rem.information(input)
def Ainfo(input):
    # print(input)
    global Astar_info
    Astar_info.append(input)
# **********************nav defs**********************

class GridBase:
    def __init__(self, grid_size: int = 5) -> None:
        self.GRID_SIZE = grid_size
        self.lastGrid = (None, None)
        self.tiles_grid = np.zeros((mapping_size // self.GRID_SIZE, mapping_size // self.GRID_SIZE), dtype='int32')

    def get_grid(self, curX: int, curY: int) -> tuple[int, int]:
        return curX // self.GRID_SIZE, curY // self.GRID_SIZE

    def set_grid(self, curX: int, curY: int, grid_count: int) -> None:
        current_grid = self.get_grid(curX, curY)
        self.tiles_grid[current_grid] = grid_count

    def is_new_grid(self, curX: int, curY: int) -> bool:
        current_grid = self.get_grid(curX, curY)
        if current_grid != self.lastGrid:
            self.lastGrid = current_grid
            return True
        return False

    def get_grid_count(self, curX: int, curY: int) -> int:
        current_grid = self.get_grid(curX, curY)
        return self.tiles_grid[current_grid]

    def update_grid(self, curX: int, curY: int) -> None:
        gridI, gridJ = self.get_grid(curX, curY)
        minX = gridI * 5
        maxX = (gridI + 1) * 5
        minY = gridJ * 5
        maxY = (gridJ + 1) * 5

        grid_count = 0
        for i in range(minX, maxX):
            for j in range(minY, maxY):
                if r2_map[i][j]["status"] == 0:
                    if self.get_grid(i, j) == self.get_grid(curX, curY):
                        grid_count += 1
        self.set_grid(curX, curY, grid_count)
def detect_hole():
    DM_d = 1000*DM.getValue()
    DR_d = 1000*DR.getValue()
    DL_d = 1000*DL.getValue()
    DR45_d = 1000*R45.getValue()
    DL45_d = 1000*L45.getValue()
#*********************************************************90 daaaaaaagaaaaaaary

    if DM_d>490 and DR_d>490 and DL_d>490 and DR45_d>290 and DL45_d>290 and x%2==0 and y%2==0:
        print_blue("there is a big black hole in front of me!!!!")
        return True
    if  DM_d>90 and DR_d>590 and DL_d>90 and DR45_d>290 and DL45_d>90:
        print_blue("there is a big black hole in the front right of me")
        return True
    if DM_d>90 and DR_d>90 and DL_d>590 and DR45_d>90 and DL45_d>300:
        print_blue("there is a big black hole in the front left of me")
        return True

#*********************************************************45 daaaaaaagaaaaaaary
    if DM_d>390 and DR_d>90 and DL_d>590  and DR45_d>90 and DL45_d>90 and x%2==0 and y%2==0:
        print_blue("i see a big black hole in the 45 deg range")
        return True
    if DM_d>390 and DR_d>700 and DL_d>90 and DR45_d>290 and DL45_d>100:
        print_blue("i see a big black hole in the 45 deg range")
        return True
    if DM_d>390 and DR_d>690 and DL_d>90 and DR45_d>290 and DL45_d>90:
        print_blue("i see a big black hole in the 45 deg range")
        return True
    if DM_d>490 and DR_d>90 and DL_d>390 and DR45_d>90 and DL45_d>290:
        print_blue("i see a big black hole in the 45 deg range")
        return True
    return False
class DeleteObstacles:
    def __init__(self):
        self.zeros = np.zeros((5, 5), dtype='int32')

        # room 1
        self.r1_pattern1 = self.zeros.copy()  # wall up
        self.r1_pattern1[0, :5] = 1

        self.r1_pattern2 = self.zeros.copy()  # wall right
        self.r1_pattern2[:5, 4] = 1

        self.r1_pattern3 = self.zeros.copy()  # wall down
        self.r1_pattern3[4, :5] = 1

        self.r1_pattern4 = self.zeros.copy()  # wall left
        self.r1_pattern4[:5, 0] = 1

        self.r1_patterns = [
            self.r1_pattern1,
            self.r1_pattern2,
            self.r1_pattern3,
            self.r1_pattern4,
        ]

        # room 2
        self.r2_pattern1 = np.array([
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ], dtype='int32')

        self.r2_pattern2 = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 1]
        ], dtype='int32')

        self.r2_pattern3 = np.array([
            [1, 1, 1],
            [0, 0, 0],
            [0, 0, 0]
        ], dtype='int32')

        self.r2_pattern4 = np.array([
            [1, 0, 0],
            [1, 0, 0],
            [1, 0, 0]
        ], dtype='int32')

        self.r2_patterns = [
            self.r2_pattern1,
            self.r2_pattern2,
            self.r2_pattern3,
            self.r2_pattern4,
        ]

        # room 3
        self.r3_pattern1 = np.array([
            [1, 1, 0],
            [0, 0, 1],
            [0, 0, 1]
        ], dtype='int32')

        self.r3_pattern2 = np.array([
            [0, 1, 1],
            [1, 0, 0],
            [1, 0, 0]
        ], dtype='int32')

        self.r3_pattern3 = np.array([
            [0, 0, 1],
            [0, 0, 1],
            [1, 1, 0]
        ], dtype='int32')

        self.r3_pattern4 = np.array([
            [1, 0, 0],
            [1, 0, 0],
            [0, 1, 1]
        ], dtype='int32')

        self.r3_pattern5 = np.array([
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ], dtype='int32')

        self.r3_pattern6 = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 1]
        ], dtype='int32')

        self.r3_pattern7 = np.array([
            [1, 1, 1],
            [0, 0, 0],
            [0, 0, 0]
        ], dtype='int32')

        self.r3_pattern8 = np.array([
            [1, 0, 0],
            [1, 0, 0],
            [1, 0, 0]
        ], dtype='int32')

        self.r3_curved_patterns = [
            self.r3_pattern1,
            self.r3_pattern2,
            self.r3_pattern3,
            self.r3_pattern4
        ]

        self.r3_half_wall_patterns = [
            self.r3_pattern5,
            self.r3_pattern6,
            self.r3_pattern7,
            self.r3_pattern8
        ]

        # room 1 and 2, for odd x AND y
        self.odd_pattern1 = np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0]
        ], dtype='int32')

        self.odd_pattern2 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0]
        ], dtype='int32')

        self.odd_pattern3 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype='int32')

        self.odd_pattern4 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.odd_patterns = [
            self.odd_pattern1,
            self.odd_pattern2,
            self.odd_pattern3,
            self.odd_pattern4,
        ]

        # room 1 and 2, for odd X and even Y
        self.x_odd_pattern1 = np.array([
            [1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32') 

        self.x_odd_pattern2 = np.array([
            [0, 0, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.x_odd_pattern3 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1],
        ], dtype='int32')

        self.x_odd_pattern4 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 0, 0],
        ], dtype='int32')

        self.x_odd_pattern5 = np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.x_odd_pattern6 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype='int32')

        self.x_odd_patterns = [
            self.x_odd_pattern1,
            self.x_odd_pattern2,
            self.x_odd_pattern3,
            self.x_odd_pattern4,
            self.x_odd_pattern5,
            self.x_odd_pattern6,
        ]

        # room 1 and 2, for even x and odd y
        self.y_odd_pattern1 = np.array([
            [1, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.y_odd_pattern2 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
        ], dtype='int32')

        self.y_odd_pattern3 = np.array([
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.y_odd_pattern4 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1],
        ], dtype='int32')

        self.y_odd_pattern5 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.y_odd_pattern6 = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype='int32')

        self.y_odd_patterns = [
            self.y_odd_pattern1,
            self.y_odd_pattern2,
            self.y_odd_pattern3,
            self.y_odd_pattern4,
            self.y_odd_pattern5,
            self.y_odd_pattern6,
        ]

    @staticmethod
    def isMatrixInMatrix(checkMatrix: np.ndarray, fullMatrix: np.ndarray):
        if checkMatrix.shape != fullMatrix.shape:
            raise OverflowError("The shape of check matrix is not equal to the shape of full matrix."
                                f" check matrix: {checkMatrix.shape}, full matrix: {fullMatrix.shape}")
        checkShapeX, checkShapeY = checkMatrix.shape
        for i in range(checkShapeX):
            for j in range(checkShapeY):
                if checkMatrix[i, j] in [1, 2] and fullMatrix[i, j] == 0:
                    return False

        return True
   
    def room1_delete_obstacles(self, walls: np.ndarray):
        for pattern in self.r1_patterns:
            if self.isMatrixInMatrix(pattern, walls):
                for i in range(5):
                    for j in range(5):
                        if pattern[i, j] == 1:
                            walls[i, j] = 2

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls

    def room2_delete_obstacles(self, walls: np.ndarray):
        for i in range(2):
            for j in range(2):
                area = walls[i * 2: i * 2 + 3, j * 2: j * 2 + 3]

                for pattern in self.r2_patterns:
                    if self.isMatrixInMatrix(pattern, area):
                        for k in range(3):
                            for t in range(3):
                                if pattern[k, t] == 1:
                                    area[k, t] = 2

                walls[i * 2: i * 2 + 3, j * 2: j * 2 + 3] = area

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls

    def room3_delete_obstacles(self, walls: np.ndarray):
        for i in range(2):
            for j in range(2):
                area = walls[i * 2: i * 2 + 3, j * 2: j * 2 + 3]
                area[1, 1] = 0

                for pattern in self.r3_curved_patterns:
                    if self.isMatrixInMatrix(pattern, area):
                        for k in range(3):
                            for t in range(3):
                                if pattern[k, t] == 1:
                                    area[k, t] = 2
                        break

                for pattern in self.r3_half_wall_patterns:
                    if self.isMatrixInMatrix(pattern, area):
                        for k in range(3):
                            for t in range(3):
                                if pattern[k, t] == 1:
                                    area[k, t] = 2

                walls[i * 2: i * 2 + 3, j * 2: j * 2 + 3] = area

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls

    def room1_2_odd_pos_delete_obstacles(self, walls: np.ndarray):
        for pattern in self.odd_patterns:
            if self.isMatrixInMatrix(pattern, walls):
                for i in range(5):
                    for j in range(5):
                        if pattern[i, j] == 1:
                            walls[i, j] = 2

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls

    def x_odd_delete_obstacles(self, walls: np.ndarray):
        for pattern in self.x_odd_patterns:
            if self.isMatrixInMatrix(pattern, walls):
                for i in range(5):
                    for j in range(5):
                        if pattern[i, j] == 1:
                            walls[i, j] = 2

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls

    def y_odd_delete_obstacles(self, walls: np.ndarray):
        for pattern in self.y_odd_patterns:
            if self.isMatrixInMatrix(pattern, walls):
                for i in range(5):
                    for j in range(5):
                        if pattern[i, j] == 1:
                            walls[i, j] = 2

        walls[walls == 1] = 0
        walls[walls > 1] = 1

        return walls



def gameinfo():
    global remaining_time_value
    time = float('inf')
    score = float('inf')
    receiver.nextPacket()

    # Send message to request game information
    message = struct.pack('c', 'G'.encode())  # 'G' indicates game information request
    emitter.send(message)
    delay(1)

    if receiver.getQueueLength() > 0:  # If receiver queue is not empty
        receivedData = receiver.getBytes()
        # logging.info(f"Received Data: {receivedData}")  # Log the raw received data
        
        if len(receivedData) >= 13:  # Ensure the packet is at least 13 bytes long
            try:
                tup = struct.unpack('c f i i', receivedData)  # Unpack data into char, float, int, int
                if tup[0].decode("utf-8") == 'G':
                    # Process the received game data
                    time = tup[2]
                    score = tup[1]
                    real_time = tup[3]
                    
                    # Handle case where real_time is less than the expected time
                    if real_time < time:
                        logging.info(f"Changed the time to real time. real time: {real_time}. time: {time}")
                        time = real_time
            except struct.error as e:
                # logging.error(f"Unpack error: {e}")
                return time, score
        else:
            # logging.warning(f"Received data length is insufficient: {len(receivedData)} bytes. Expected 13 bytes.")
            pass
    remaining_time_value = time
    return time, score

def gametime():
    global remaining_time_value
    time = float('inf')
    receiver.nextPacket()

    # Send message to request game information
    message = struct.pack('c', 'G'.encode())  # 'G' indicates game information request
    emitter.send(message)
    delay(1)

    if receiver.getQueueLength() > 0:  # If receiver queue is not empty
        receivedData = receiver.getBytes()
        
        if len(receivedData) >= 13:  # Ensure the packet is at least 13 bytes long
            try:
                # Unpack only the time (3rd element in the original unpacking tuple)
                tup = struct.unpack('c f i i', receivedData)  # Unpack data into char, float, int, int
                if tup[0].decode("utf-8") == 'G':
                    time = tup[2]  # Extract the time value
            except struct.error as e:
                # Handle unpacking error gracefully
                return time
        else:
            # Handle case where received data is insufficient
            pass

    remaining_time_value = time
    return time



def lackofprog(forceLop: bool = False, rotate_when_lop: bool = False):
    global checkForFarTokens, lastWhatRoomChecked, room
    print_map(r2_map, 0)
    tileNum = tileType()
    checkForFarTokens = True
    now = getgps(3)
    logging.warning(f"LACK OF PROGRESS. tile: {tilenum}. forceLop: {forceLop}. rotate: {rotate_when_lop}. cur: ({x}, {y})")
    
    lastWhatRoomChecked = (0, 0)
    
    if (not (tileNum == 4 and x % 2 == 0 and y % 2 == 0) and (x, y) != startp) or forceLop:
        if rotate_when_lop:
            rotation_for_degrees(180)

        message = struct.pack('c', 'L'.encode())
        emitter.send(message)
        delay(17)
        logging.info("LOP SENDED.")
        while robot.step(timeStep) != -1:
            if receiver.getQueueLength() > 0:
                break
        _ = receiver.getBytes()
        receiver.nextPacket()
        while robot.step(timeStep) != -1:
            getgps()
            setLocation()
            lidarProMax(True)
            break
        if r2_map[x][y]["room"] != 0:
            new_room = r2_map[x][y]["room"]
            logging.info(f"Room changed to {new_room} from {room} after LOP")
            print_yellow(f"Room changed to {new_room}")
            room = r2_map[x][y]["room"]
    else:
        print_red("We are in checkpoint, no need for lop")
        logging.info("We are in checkpoint, no need for lop")


def checkForLOP(target):
    if receiver.getQueueLength() > 0:  # If receiver queue is not empty
        receivedData = receiver.getBytes()
        
        # Check if receivedData has exactly 1 byte before unpacking
        if len(receivedData) == 1:
            tup = struct.unpack('c', receivedData)  # Parse data into character
            
            if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                print_green("Detected Lack of Progress!")
                logging.warning("Detected Lack of Progress!")
                receiver.nextPacket()
                x_target, y_target = give_location(*target)
                r2_map[x_target][y_target]["status"] = -2
                r2_map[x_target][y_target]["type"] = 1
                
                while robot.step(timeStep) != -1:
                    setLocation()
                    lidarProMax(True)
                    break
                
                return True
        else:
            # Handle the case when the data is not the expected size
            # logging.warning(f"Unexpected data size: {len(receivedData)} bytes")
            return False
            
    return False


def exitProcess(checkroom4=True):
    global room4checkup
    guess_room()
    checkroom0()
    # guess_room()
    # guess_room()
    logging.info("Sending the map...")
    for i in range(3):
        final_f_map=print_map(r2_map, 0, True)
        checkroom0()
    
    
    
    final_f_map[(2 * (startp[0] - y_min)) + 3, (2 * (startp[1] - x_min)) + 1] = 5
    final_f_map[(2 * (startp[0] - y_min)) + 1, (2 * (startp[1] - x_min)) + 1] = 5
    final_f_map[(2 * (startp[0] - y_min)) + 3, (2 * (startp[1] - x_min)) + 3] = 5
    final_f_map[(2 * (startp[0] - y_min)) + 1, (2 * (startp[1] - x_min)) + 3] = 5
    
    time,_=gameinfo()
    if time>1*60 and not room4checkup>2 and checkroom4:
        room4checkup+=1
        print_big_green("we are going back for secounds :)")
        for i in range (x_min,x_max):
            for j in range(y_min,y_max):
                if r2_map[i][j]["room"] in [4,0]:
                    r2_map[i][j]["status"]=-1
                if r2_map[i][j]["type"] in [8,9,10]:
                    r2_map[i][j]["status"]=0
        return
    send_map(final_f_map)
    logging.info("Sending Exit signal to server....")
    print_green("Exiting process started.")
    try_times = 0
    while True:
        if try_times >= 1:
            print(f"Trying exiting again. Tries: {try_times + 1} times.")
            logging.info(f"Trying exiting again. Tries: {try_times + 1} times.")
        emitter.send(bytes('E', "utf-8"))
        delay(64)
        try_times += 1


def hoho():
    imagerange = get_lidar()
    teta = 0
    mina = min(imagerange)
    k = 1.5
    kl = 1
    min1 = 3.7 # 3.7 ham khoob bod bejoz zamin 1-4 2024

    if 384 < imagerange.index(mina) < 512 or 0 < imagerange.index(mina) < 128:
        teta = imagerange.index(mina) * 0.703125
        range_shild = (min1 * cos(math.radians(teta)))

        # print("teta",teta)
        # print("the index is :",imagerange.index(mina ))
        # print(range_shild)

        kl = (mina - range_shild) * k

        # print(" range shild", range_shild)
        # print("kl==",kl)
        if kl < 0:
            kl = 0
        if kl > 1:
            kl = 1

    return kl    

def hehe():
    yaw = getyaw()
    angel = 0
    k = 0.1
    error = 0
    imagerange = get_lidar()
    teta = 0

    mina = min(imagerange)
    if 384 < imagerange.index(mina) < 512 and 6 > mina > 4.4:
        teta = imagerange.index(mina) * 0.703125
        error = 360 - teta
        angel = error * k
        # print('angel',angel)
        # print("teta",teta)
        # print("index is",imagerange.index(mina))
    elif 0 < imagerange.index(mina) < 128 and 6 > mina > 4.4:
        teta = imagerange.index(mina) * 0.703125
        error = teta - 90
        angel = error * k

    return angel
def avoid_tooli():
    global flag_tooli
    range_shild=4
    instanceR=0
    instanceL=0
    instanceM=0
    error_angle=0
    imagerange=get_lidar()
    for i in range(50):
        index=i+32
        if imagerange[index]<range_shild:
            instanceR+=1
                
    for i in range(50):
        index=i+430
        if imagerange[index]<range_shild:
            instanceL+=1
    for i in range(42):
        index=i+470
        if imagerange[index]<range_shild:
            instanceM+=1 
    for i in range(42):
        index=i
        if imagerange[index]<range_shild:
            instanceM+=1
    # print(f"inctace toooooooli {instanceL,instanceM ,instanceR}")
    
    if instanceM>30:
        error_angle=0
        return -1
    elif instanceR>20:
        error_angle=-instanceR
    elif instanceL>20:
        error_angle=instanceL
    if instanceR>20 and instanceL>20 and instanceM<20:
        if instanceL>instanceR:
            error_angle=-instanceL
        else:
            error_angle=instanceR
    if abs(error_angle)<30:
        error_angle=0
        
    # print(error_angle) 
    return error_angle*1.1
        
    
    
def read_color_sensor() -> tuple[int, int, int]:
    image = cs.getImage()
    red = cs.imageGetRed(image, 1, 0, 0)
    green = cs.imageGetGreen(image, 1, 0, 0)
    blue = cs.imageGetBlue(image, 1, 0, 0)
    
    return red, green, blue

def forward(target, doVision: bool = True,detect_tile=True,tryingtogetout=False):
    global base_k,k_angel,wheel_speed,flage_pid,runAstar,gridam_count,lastx,lasty,forward_flag,tryingtogetout_g,tilenum, lastWhatRoomChecked, r2_map,multyply_cost,error_tooli,flag_tooli, pc
    error_tooli=0
    fsp1 = (0, 0)
    ang = 0
    flag_tooli=1

    bbc = 0
    first_cordx, first_cordy = getgps()
    girkardim = 0.004
    gircount = 0
    f_hole=0
    precision = 0.5
    speed_margin = 0.05
    pidR.reset()
    pidL.reset()
    print_map(r2_map)
    if tileType() < 6:
        lastWhatRoomChecked = (0, 0)

    lidarProMax(True)
    r2_map[x][y]["visit_count"] += 1
    tilesGrid.update_grid(x, y)

    nextX, nextY = give_location(*target)
    room=what_room(nextX, nextY)
    if r2_map[x][y]["room"]==0:
        r2_map[x][y]["room"]=room
    else:
        room=r2_map[x][y]["room"]
    
    deviation_count = 0
    if forward_flag==1:
        ascii_art = pyfiglet.figlet_format("The Kavosh team presents:", font="slant")
        print(Fore.GREEN + ascii_art + Style.RESET_ALL)
        forward_flag=0
    tooli_count=0
    # what_room(nextX, nextY) 
    while robot.step(timeStep) != -1:

        if rollPitchHaveDeviation():
            if deviation_count>7:
                gps=getgps()
                giridam_x, giridam_y = give_location(gps[0],gps[1])
                lackofprog(forceLop=True)
                go(0.5,0.5)
                roll, pitch = getRoll(), getPitch()
                logging.warning(f"Forward high deviation. LOP. Roll: {roll}. Pitch: {pitch}")
                r2_map[giridam_x][giridam_y]["type"] = 1
                r2_map[giridam_x][giridam_y]["status"] = -2
                r2_map[giridam_x][giridam_y]["room"] = room
                logging.info("hight diviation lop")
                return 0
            else:
                deviation_count+=1
                logging.warning("we have high deviation")
        if detect_tile:
            # tileColorDitaction()
            pass
        _, _, locationUpdated = setLocation()
        lidarProMax(False)
        if doVision:
            vision()

        if checkForLOP(target):
            return 0

        xN, yN = getgps()
        yaw = getyaw()
        fsp2 = (xN, yN)
        ang2 = yaw
        error = sqrt(abs(target[0] - xN) ** 2 + abs(target[1] - yN) ** 2)
        error_angel = angleFinder((target[0] - xN), (target[1] - yN))

        tileNum = tileType()
        if tileNum == 2:
            if bbc >= 1:
                bbc = 0
                red, green, blue = read_color_sensor()
                targetX, targetY = give_location(target[0],target[1])
                xN, yN = getgps(0)
                targetGPSX, targetGPSY = target
                distanceFromTarget = sqrt((targetGPSX - xN) ** 2 + (targetGPSY - yN) ** 2)/10
                logging.info(f"Trap Detected with rgb colors: ({red}, {green}, {blue}) in ({x}, {y}) with target ({targetX}, {targetY}) target x,y {target} and distance from target: {distanceFromTarget}")
                print_yellow("Trap detected.")
                x_target, y_target = give_location(*target)
                r2_map[x_target][y_target]["room"] = room
                r2_map[x][y]["room"] = room
                colorTileMapping(target[0], target[1], 2, 1, room)
                backward_distance = sqrt(((first_cordx - xN) ** 2) + ((first_cordy - yN) ** 2))/10
                logging.info(f"backwards dis : {backward_distance}")
                lastWhatRoomChecked = (0, 0)
                backward(backward_distance)
                return 0
            else:
                bbc += 1
                logging.info(f"Detected Trap {bbc} time(s)")
            
        # TODO tune k s
        deg_dif = hehe()

        rangeshild = hoho()

        if tryingtogetout:
            deg_dif=0
            rangeshild=0
        angel_speed = error_angel * k_angel
        speed_base = error * base_k * rangeshild

        if speed_base > max_velocity - speed_margin:
            speed_base = max_velocity - speed_margin
        speedL = speed_base + angel_speed - deg_dif
        speedR = speed_base - angel_speed + deg_dif

        #set the min and max of the avaleble sppel for each weel
        if speedL > max_velocity:
            speedL = max_velocity
        if speedL < -max_velocity:
            speedL = -max_velocity
        if speedR > max_velocity:
            speedR = max_velocity
        if speedR < -max_velocity:
            speedR = -max_velocity
#just checking
        control_L=0
        control_R=0
        
        if not (error_angel>35 or error_angel<-35) :
            control_L = pidL(speedL)*PID_K
            control_R =pidR(speedR)*PID_K
        else:
            pidR.reset()
            pidL.reset()
            flage_pid=1
        if flage_pid ==1:
            if abs(error_angel)>10:
                pidR.reset()
                pidL.reset()
            else :
                flage_pid=0
        if rangeshild<0.1:
            control_L=0
            control_R=0
            pidR.reset()
            pidL.reset()

        targetGPSX, targetGPSY = target
        # print(f"error 
        if  abs(error_angel)<12.5 and  sqrt((targetGPSX - xN) ** 2 + (targetGPSY - yN) ** 2)>4.5:
            # print(abs(error_angel))
            if detect_hole():
                go(0.5,0.5)
                targetX, targetY = give_location(*target)
                xN, yN = getgps(0)
                distanceFromTarget = sqrt((targetGPSX - xN) ** 2 + (targetGPSY - yN) ** 2)
                print_yellow("Trap detected with DS system.")
                logging.info("Trap detected with DS system.")
                x_target, y_target = give_location(*target)
                
                r2_map[x_target][y_target]["room"] = room
                r2_map[x][y]["room"] = room
                colorTileMapping(target[0], target[1], 2, 1, room)
                backward_distance = sqrt(((first_cordx - xN) ** 2) + ((first_cordy - yN) ** 2))
                lastWhatRoomChecked = (0, 0)
                backward(backward_distance)

                return 0
                #  return 0

    # if abs(error_angel)<10:


        speedL+=control_L
        speedR+=control_R


        #set the min and max of the avaleble sppel for each weel
        if speedL > max_velocity:
            speedL = max_velocity
        if speedL < -max_velocity:
            speedL = -max_velocity
        if speedR > max_velocity:
            speedR = max_velocity
        if speedR < -max_velocity:
            speedR = -max_velocity


        # print(control_R,control_L)
        # print(error_angel)
        
        
        # if flag_tooli:
        #     if avoid_tooli()!=0 :
        #         error_tooli=avoid_tooli()
        #         if error_tooli==-1:
        #             error_tooli=0
        #             flag_tooli=0
        #         tooli_count=0
        #     else:
        #         tooli_count+=1
        #     if tooli_count>5:
        #         error_tooli=0
        
        
            
        # else:
        #     error_tooli=0
        
        # print(f"error tooli : {error_tooli}")
        # print(f"speedL : {error_tooli}")
        # print(f"tooli : {error_tooli}")

        go(speedL , speedR )
        wheel_speed=(speedL, speedR)

        if (abs(fsp2[0] - fsp1[0]) + abs(fsp2[1] - fsp1[1])) < girkardim and abs(ang2 - ang) < 1:
            gircount += 1
            x_target, y_target = give_location(*target)

            if gircount > 7:  
                # lackofprog(rotate_when_lop=True)
                   
                if gridam_count>7:
                    print_red("gir lop")
                    logging.warning("gir lop")
                    gridam_count=0
                    r2_map[x][y]["status"] = -2
                    lackofprog(rotate_when_lop=True)
                    return -1
                if gridam_count>4:
                    #TODO bug on map w3_3135 fix this the bug is n the ending 
                    print("trying to get out")
                    r2_map[x][y]["status"] = -2
                    r2_map[x_target][y_target]["status"] = -2
                    r2_map[x_target][y_target]["type"] = 1 
                    r2_map[x_target][y_target]["room"] = room
                    r2_map[x][y]["room"] = room
                    multyply_cost.append([x_target,y_target])
                    multyply_cost.append([x,y])
                    backward(4)
                    setLocation()
                    
                    # forward((x,y))
                    gridam_count+=1
                    
                    return -1
                if gridam_count>2:
                    backward(1)  # 1.1 bood
                    print_blue("going back 0.7cms")
                    r2_map[x][y]["status"] = -2
                    r2_map[x_target][y_target]["status"] = -2
                    r2_map[x_target][y_target]["type"] = 1 
                    r2_map[x_target][y_target]["room"] = room
                    r2_map[x][y]["room"] = room
                    multyply_cost.append([x_target,y_target])
                    multyply_cost.append([x,y])
                    r2_map[x_target][y_target]["type"] = 1 
                    r2_map[x][y]["room"] = room
                    r2_map[x_target][y_target]["room"] = room
                    gridam_count+=1
                    
                    return -1

                print("giridaaaaaaaaaaaaaaaaaaaaaaaaaaaam")
                backward(1) # 1.1 bood
                giridam_spoter([x_target,y_target])
                r2_map[x_target][y_target]["status"] = 1
                r2_map[x_target][y_target]["type"] = 1 
                r2_map[x_target][y_target]["room"] = room
                r2_map[x][y]["room"] = room
                logging.info(f"Giridam in ({x_target}, {y_target})")
                gridam_count+=1
  
                return -1
        else:
            gircount=0
        if error < precision:
            gridam_count=0
            lastx=x
            lasty=y
            check_targets()

            setLocation()
            pc = 0
            break
        
        fsp1 = fsp2
        ang = ang2

    return 1


def  angleFinder(dX, dY):

    if dX == 0:
        logging.error("Division by zero in angle finder function.")
        raise ZeroDivisionError("Division by zero in angle finder function.")

    now_angel = atan(dY / dX)

    now_angel = math.degrees(now_angel)

    if now_angel > 0:
        if dX > 0 and dY > 0:
            now_angel = 90 - now_angel
        else:
            now_angel = 270 - now_angel
    else:
        if dX > 0 and dY < 0:
            now_angel = abs(now_angel) + 90
        if dX < 0 and dY > 0:
            now_angel = abs(now_angel) + 270
            
    now_angel -= getyaw()
    
    if now_angel > 180:
        now_angel = -360 + now_angel
    if now_angel < -180:
        now_angel = 360 + now_angel

    return (now_angel)



def backward(cm):
    k = -24
    girkardim = 0.0002
    gircount = 0
    back_precision = 0.7
    x_old, y_old = getgps()
    fsp1 = (0, 0)
    deviation_count = 0
    while robot.step(timeStep) != -1:
        if rollPitchHaveDeviation(high_deviation=True):
            if deviation_count > 7:
                roll, pitch = getRoll(), getPitch()
                logging.warning(f"Forward high deviation. LOPP. Roll: {roll}. Pitch: {pitch}")
                lackofprog()
                return
            else:
                deviation_count += 1
        fsp2 = getgps()
        tileNum = tileType()
        ang2 = getyaw()
        x_new, y_new = getgps()
        distance = abs(x_old - x_new) + abs(y_old - y_new)
        error = cm - distance
        speed = error * k
        if speed < -max_velocity:
            speed = -max_velocity
        elif speed > max_velocity:
            speed = max_velocity
            
        go(speed, speed)
    
        if abs(x_old - x_new) + abs(y_old - y_new) > abs(cm) - back_precision and tileNum != 2:
            go(0, 0)
            return
        if (abs(fsp2[0] - fsp1[0])) + abs(fsp2[1] - fsp1[1]) < girkardim and abs(ang2 - ang) < 1:
            gircount += 1
            if gircount > 7:
                print("We are giridim in the hole")
                print("LOP Gir")
                logging.warning("We are giridam in the hole.")
                logging.warning("LOP Gir")
                # lackofprog()
                return -1

        fsp1 = fsp2
        ang = ang2


def go(speedL, speedR):
    if speedL > max_velocity:
        speedL = max_velocity
    if speedL < -max_velocity: 
        speedL = -max_velocity
    if speedR > max_velocity:
        speedR = max_velocity
    if speedR < -max_velocity:
        speedR = -max_velocity
    wheel1.setVelocity(speedL)
    wheel2.setVelocity(speedR)


go(0, 0)


def getyaw():
    gyrod = gyro.getRollPitchYaw()
    yaw = gyrod[2] * 180 / pi + 180
    return yaw


def getRoll():
    gyrod = gyro.getRollPitchYaw()
    roll = gyrod[0] * 180 / pi
    return roll


def getPitch():
    gyrod = gyro.getRollPitchYaw()
    pitch = gyrod[1] * 180 / pi
    return pitch


def getgps(round_number: int | None = None):
    gpsd = gps.getValues()
    # print(gpsd[1]*100) 
    if round_number is None:
        return (gpsd[0] * 100, gpsd[2] * 100)
    else:
        return round(gpsd[0] * 100, round_number), round(gpsd[2] * 100, round_number)
    
  

def findTargets():
    global startp, x, y
    targets = []
    for i in range(x_min, x_max + 1):
        for j in range(y_min, y_max + 1):
            if r2_map[i][j]["status"] == 0:
                targets.append((i, j))
    # print(targets)
    return (targets)


def target_sorting_algorithm(array):
    return abs(x - array[:, 0]) + abs(y - array[:, 1])


def sortTargets(targets):
    if not targets:
        return []
    targets = np.array(targets)
    predicate = target_sorting_algorithm(targets)
    order = np.argsort(predicate)
    targets = targets[order]
    targets = list(map(tuple, targets.tolist()))
    return targets


def h_cost(start, target):
    global x, y, x_min, y_min, x_max, y_max 
    start_x, start_y = start
    target_x, target_y = target
    hcost = abs(target_x - start_x) + abs(target_y - start_y)
    return hcost


def astar_wall_cost_tileways(walls):
    # WARNING: DO NOT USE THIS FUNCTION ANYWHERE EXCEPT FOR THE WALLS COST IN A*.

    up, down, left, right, ul, ur, dl, dr = True, True, True, True, True, True, True, True

    if walls[1: 3, 0: 1].all() or walls[2: 4, 0: 1].all() or walls[1: 3, 1: 2].all() or walls[2: 4, 1: 2].all():
        left = False

    if walls[1: 3, 3: 4].all() or walls[2: 4, 3: 4].all() or walls[1: 3, 4: 5].all() or walls[2: 4, 4: 5].all():
        right = False

    if walls[0: 1, 1: 3].all() or walls[0: 1, 2: 4].all() or walls[1: 2, 1: 3].all() or walls[1: 2, 2: 4].all():
        up = False

    if walls[3: 4, 1: 3].all() or walls[3: 4, 2: 4].all() or walls[4: 5, 1: 3].all() or walls[4: 5, 2: 4].all():
        down = False

    if walls[0:2, 0:2].any() or( walls[2, 1] and walls[1, 2]):
        ul = False

    if walls[0:2, 3:5].any() or( walls[1, 2] and walls[2, 3]):
        ur = False

    if walls[3:5, 3:5].any() or( walls[2, 3] and walls[3, 2]):
        dr = False

    if walls[3:5, 0:2].any() or (walls[2, 1] and walls[3, 2]):
        dl = False

    return up, down, left, right, ul, ur, dl, dr


def Astar(start, target, maxcost, visit_cost: bool = False, walls_cost: bool = True):
    # t5=time.time()
    # t1 = time.time()
    openq = PriorityQueue()

    # tiles=[]
    # for i in range(x_min,x_max+1):
    #     for j in range(y_min,y_max+1):
    #        tiles.append((i,j))
    tilefg = np.full((mapping_size, mapping_size, 2), np.inf)

    tilefg[start[0], start[1], 0] = h_cost(start, target)
    tilefg[start[0], start[1], 1] = 0
    absy = abs_yaw()
    openq.put((tilefg[start[0], start[1], 0], h_cost(start, target), start, absy))
    way = {}
    while not openq.empty():
        data = openq.get()
        cell = data[2]
        cellangle = data[3]
        if cell == target:
            break
        
        cellf = data[0]
        cellh = data[1]
        cellg = cellf - cellh
        
        if cellg > maxcost:
            return {}, inf

        up, down, left, right, ul, ur, dl, dr = tileways(*cell)
        l = [up, down, left, right, ul, ur, dl, dr]
        for i in range(8):
            if l[i] == True:
                cellx, celly = cell
                chcell = (cellx, celly)
                angle = 0
                forward_cost = 1
                turn_cost = 0
                # degree_45_cost = 0
                
                if i == 0:
                    chcell = (cellx, celly - 1)
                    angle = 180
                elif i == 1:
                    chcell = (cellx, celly + 1)
                    angle = 360
                elif i == 2:
                    chcell = (cellx - 1, celly)
                    angle = 270
                elif i == 3:
                    chcell = (cellx + 1, celly)
                    angle = 90
                elif i == 4:
                    chcell = (cellx - 1, celly - 1)
                    angle = 225
                    forward_cost *= 1.4
                elif i == 5:
                    chcell = (cellx + 1, celly - 1)
                    angle = 135
                    forward_cost *= 1.4
                elif i == 6:
                    chcell = (cellx - 1, celly + 1)
                    angle = 315
                    forward_cost *= 1.4
                elif i == 7:
                    chcell = (cellx + 1, celly + 1)
                    angle = 45
                    forward_cost *= 1.4

                if r2_map[chcell[0]][chcell[1]]["status"] in [-2, -1, 2, 0] and chcell != target:
                    # print("skiiip", chcell)s
                    continue
                elif r2_map[chcell[0]][chcell[1]]["type"] == 2 and chcell != target:
                    continue

                h_chc = h_cost(chcell, target)
                area = r2_map[chcell[0]][chcell[1]]

                g_chc = cellg
                
                a1 = angle
                a2 = cellangle
                error_angle = abs(a1 - a2)
                if error_angle > 180:
                    error_angle -= 180
                
                
                                
                if cellangle != angle:
                    # forward_cost = 0
                    if error_angle == 45:
                        turn_cost = 1.4/2
                    elif error_angle == 90:
                        turn_cost = 1.68/2
                    elif error_angle == 135:
                        turn_cost = 1.96/2
                    elif error_angle == 180:
                        turn_cost = 2.24/2
                    
                if area["type"] == 3:
                    forward_cost *= 5
                    turn_cost *= 5
                    # degree_45_cost *= 5

                g_chc += forward_cost
                g_chc += turn_cost
                # g_chc += degree_45_cost
                
                f_chc = h_chc + g_chc

                if tilefg[chcell[0], chcell[1], 0] > f_chc:
                    tilefg[chcell[0], chcell[1], 0] = f_chc
                    tilefg[chcell[0], chcell[1], 1] = g_chc
                    openq.put((f_chc, h_chc, chcell, angle))

                    way[chcell] = cell

    path = {}
    cell = target

    while cell != start:
        try:
            path[way[cell]] = cell
            cell = way[cell]
        except:
            logging.warning(f"Trapped A* from {start} to {target}")
            # lackofprog()
            return {}, -1

    # t6=time.time()
    # print("aastar : ", t6-t5)

    # t2 = time.time()
    # print((t2-t1)*1000)
    
    final_g_cost = tilefg[target[0], target[1], 1]
    
    # wall cost
    # target_walls = astar_wall_cost_tileways(r2_map[target[0]][target[1]]["walls"])
    target_walls = tileways(*target)
    target_walls = np.array(target_walls)
    walls_cost = round(1 * len(target_walls[target_walls == True]), 3)
    final_g_cost += walls_cost
    # wall cost
    
    return path, final_g_cost


def convert_cost_to_time(cost: float, remaining_time: int = 5):
    timeK = 0.6
    time = cost * timeK
    return time + remaining_time


def checkTimeIsLow(time: int, movementForTime: bool = True):
    global lopNoPathCount, lopNoPathTargets
    if time != inf:
        way2, cost = Astar((x, y), (startpx, startpy), inf)
        cost_to_time = convert_cost_to_time(cost)
        
        logging.info(f"Time to get to start: {cost_to_time}, time: {time}, cost: {cost}")
        if time <= int(round(cost_to_time)):
            if time <= 2:
                logging.info(f"Time is less than 2 seconds. time: {time} and cost to time: {cost_to_time}. the current position: ({x}, {y})")
                print("Time is low. GIVE UP")
                exitProcess()
            
            if not movementForTime:
                return
            
            print(
                f"We don't have enough time for continuing the search. Returning to start. Time: {time}, "
                f"Cost to Time: {int(round(cost_to_time))}, and A* Cost: {cost}")
            logging.info(
                f"We don't have enough time for continuing the search. Returning to start. Time: {time}, "
                f"Cost to Time: {int(round(cost_to_time))}, and A* Cost: {cost}")

            _, costCheckpoint = Astar(lastVisitedCheckpoint, (startpx, startpy), inf)
            cost_to_time_checkpoint = convert_cost_to_time(costCheckpoint)

            logging.info(f"Cost to time checkpoint: {cost_to_time_checkpoint}, cost checkpoint: {costCheckpoint}")

            if time < cost_to_time_checkpoint:
                if round(costCheckpoint) + 7 < round(cost):
                    logging.info(f"LOP to get closer to the start. we don't have enough time. Time: {time},"
                                 f" Cost to time checkpoint: {int(round(cost_to_time_checkpoint))}, "
                                 f"and cost checkpoint: {costCheckpoint}")
                    print("LOP to get closer to the start. we don't have enough time.")
                    rotation_for_degrees(180)
                    vision()
                    lackofprog()
                    while robot.step(timeStep) != -1:
                        getgps()
                        setLocation()
                        lidarProMax(True)
                        break

            lopNoPathCount = 0
            rotation_for_degrees(90)
            while robot.step(timeStep) != -1:
                setLocation()
                way2, cost = Astar((x, y), (startpx, startpy), inf)
                time, score = gameinfo()
                cost_to_time = convert_cost_to_time(cost)
                if time <= 2 and int(round(cost_to_time)) > 2:
                    logging.info(f"Time is less than 2 seconds. time: {time} and cost to time: {cost_to_time}. the current position: ({x}, {y})")
                    print("Time is low. GIVE UP")
                    exitProcess()
                if not way2:
                    lopDecision([])
                    continue
                else:
                    lopNoPathCount = 0
                    lopNoPathTargets = []
                if astarmovment(way2, False, False):
                    logging.info("We are at the start...")
                    break

            exitProcess()


def astarnearest():
    path = {}
    unSortedTargets = findTargets()
    targets = sortTargets(unSortedTargets)
    gcost = []
    minTarget = ()

    if targets:
        max = inf
        path = {}
        for i in range(len(targets)):
            p, g = Astar((x, y), targets[i], max)
            gcost.append([p, g, targets[i]])
            if gcost[i][1] < max and gcost[i][1] not in [inf, -1]:
                max = gcost[i][1]
                path = gcost[i][0]
                minTarget = gcost[i][2]
            elif gcost[i][1] == max:
                logging.info(f"Astarnearest: Second A* {minTarget}, {targets[i]}")
                pre_p_start, pre_g_start = Astar(minTarget, startp, inf, visit_cost=True)
                new_p_start, new_g_start = Astar(targets[i], startp, inf, visit_cost=True)

                if new_g_start > pre_g_start not in [inf, -1] and new_g_start not in [inf, -1]:
                    max = gcost[i][1]
                    path = gcost[i][0]
                    minTarget = gcost[i][2]
    nopath = False
    costs = np.array([g for _, g, _ in gcost])
    if (costs == -1).all() and gcost and targets:
        nopath = True

    if nopath:
        logging.warning(f"No path found but there are targets LOP {targets}")
        return -1, targets
    else:
        return path, targets


def astarmovment(path, doVision: bool = True, checkForTime: bool = True) -> bool:
    """
    Track the path that A* gives us.
    @param path: The path we want to track
    We don't want to run vision in that time.
    @return: Returns that the tracking the path completed or not.

    Args:
        @param doVision: Do the vision task in forward or not. Used for returning to start because of running out of time.
        @param checkForTime: Check for low time or not.
    """
    global x, y, x_min, y_min, x_max, y_max, xpre, ypre, r2_map
    # global lastTarget, thisTarget

    try:
        next = path[(x, y)]
    except:
        right_search()
        return False
    
    return_value = False
    for pX, pY in list(reversed(path)):
        try:
            time, _ = gameinfo()
            logging.info(f"Trying to get to {pX}, {pY}")
            
            checkTimeIsLow(time, checkForTime)
            
            if not forward(getLocation((pX, pY)), doVision):
                logging.info("Giridam? Or LOP?")
                go(0, 0)
                return False
            # lidarProMax()
            print_map(r2_map, 0)
            return_value = True
        except Exception as e:
            logging.exception(f"Forward Except... {e}")
            print("We have forward except.")
            return False

    return return_value


def tileType():
    
    global bbc
    tileNum = 1
    red, green, blue = read_color_sensor()

    # print("color : ",red,blue,green)
    # print(red,blue,green)
    # if red <= 50 and green <= 50 and blue <= 50 and red != 34 and green != 34 and blue != 34:
    if (red <= 25 or 39 <= red <= 47) and (green <= 25 or 39 <= green <= 47) and (blue <= 25 or 39 <= blue <= 47):
        # print("trap")

        tileNum = 2

        return tileNum
    elif red > 180 and red < 230 and blue > 80 and blue < 120 and green > 150 and green < 210 and blue < green < red:
        # print("swap")
        tileNum = 3
        return tileNum

    elif red > 15 and red < 109 and blue > 40 and blue < 110 and green > 20 and green < 109 and red < green < blue:
        # print("check point")
        tileNum = 4
        return tileNum

    elif red > 200 and red < 280 and blue > 55 and blue < 90 and green > 55 and green < 90 and green == blue:
        # print("red")
        tileNum = 8
        return tileNum
    elif red > 110 and red < 170 and blue > 200 and blue < 250 and green > 20 and green < 80 and green < red < blue:
        # print("purple")
        tileNum = 7
        return tileNum
    elif red > 50 and red < 80 and blue > 200 and blue < 280 and green > 50 and green < 80 and green == red:
        # print("blue")
        tileNum = 6
        return tileNum
    elif 20 < red < 50 and blue > 25 and blue < 45 and green > 220 and green < 260 and red == blue:
        # print("green")
        tileNum = 9
        return tileNum
    elif red > 200 and red < 280 and green > 200 and green < 280 and blue > 50 and blue < 90 and blue < green < red:
        # print("orange")
        tileNum = 10
        return tileNum
    elif red > 200 and red < 280 and green > 220 and green < 300 and blue > 50 and blue < 90 and blue < green == red:
        # print("yellow")
        tileNum = 11
        return tileNum

    # print("the m : " , tileNum)
    return tileNum


def what_room(nextx, nexty):
    global room, lastWhatRoomChecked, xpre, ypre,rooms_chenged_in_whatroom,r2_map

    tile_type = tileType()
    
    if tile_type < 6:
        return room
    
    if not (x % 2 == 0 or y % 2 == 0):
        logging.info(f"Returning from what_room becuase of odd x and y. ({x}, {y})")
        return room
    
    if (x % 2 == 0 and y % 2 == 0):
        gps_xpre, gps_ypre = getLocation((xpre, ypre))
        gps_nextx, gps_nexty = getLocation((nextx, nexty))

        nodes_distance = sqrt((gps_xpre - gps_nextx) ** 2 + (gps_ypre - gps_nexty) ** 2)
    
        if round(nodes_distance) < 8:
            logging.info(f"Returning from what_room becuase of nodes_distance {nodes_distance}")
            return room
        
    elif (x % 2 == 0 or y % 2 == 0):
        gps_nextx, gps_nexty = getLocation((nextx, nexty))
        xN, yN = getgps(0)    
        nodes_distance = sqrt((xN - gps_nextx) ** 2 + (yN - gps_nexty) ** 2) 
        if round(nodes_distance) < 8:
            logging.info(f"Returning from what_room becuase of nodes_distance {nodes_distance}")
            return room   
            
    # TODO
    if (xpre == nextx or ypre == nexty):
        if (x % 2 == 0 and y % 2 == 1) and (xpre % 2 == 1 and ypre % 2 == 0) and (nextx % 2 == 1 and nexty % 2 == 0) and nexty == ypre:
            return room
        if (x % 2 == 1 and y % 2 == 0) and (xpre % 2 == 0 and ypre % 2 == 1) and (nextx % 2 == 0 and nexty % 2 == 1) and nextx == xpre:
            return room
    
    gps_xpre, gps_ypre = getLocation((xpre, ypre))
    gps_nextx, gps_nexty = getLocation((nextx, nexty))

    nodes_distance = sqrt((gps_xpre - gps_nextx) ** 2 + (gps_ypre - gps_nexty) ** 2)
    
    if round(nodes_distance) < 8:
        logging.info(f"Returning from what_room becuase of nodes_distance {nodes_distance}")
        return room

    # TODO
    
    lastX, lastY = lastWhatRoomChecked
    gps_lastX, gps_lastY = getLocation((lastX, lastY))
    xN, yN = getgps(0)
    distanceFromLast = sqrt((gps_lastX - xN) ** 2 + (gps_lastY - yN) ** 2)
    
    if round(distanceFromLast) <= 13:
        logging.info(f"Returning from what_room becuase of distanceFromLast {distanceFromLast}")
        return room

    x_vars = [x, nextx, xpre]
    x_vars = list(set(x_vars))
    y_vars = [y, nexty, ypre]
    y_vars = list(set(y_vars))
    if len(x_vars) == 1 or len(y_vars) == 1:
        if not (x % 2 == 0 and y % 2 == 0):
            logging.info(f"Returning from what_room becuase of pre, cur, and next are in a same direction but robot is not in an even position. pre: ({xpre}, {ypre}), cur: ({x}, {y}), next: ({nextx}, {nexty})")
            return room

    exactX, exactY = getLocation((x, y))
    xN, yN = getgps()
    distanceFromCenter = sqrt((exactX - xN) ** 2 + (exactY - yN) ** 2)
    
    if round(distanceFromCenter) >= 4:
        logging.info(f"Returning from what_room becuase of distanceFromCenter {distanceFromCenter}")
        return room
# TODO
    logging.info(f"What Room Information. distanceFromCenter: {distanceFromCenter}, distanceFromLast: {distanceFromLast}, "
                 f"nodes_distance: {nodes_distance}, pre: ({xpre}, {ypre}), cur: ({x}, {y}), next: ({nextx}, {nexty}), tilenum: {tilenum}, room: {room}")
    
    # print(distanceFromLast, distanceFromCenter, nodes_distance, (xpre, ypre), (x, y), (nextx, nexty))
    
    lastWhatRoomChecked = (x, y)
    if room == 1 and tile_type == 6:
        print("we are in room 2")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 2")
        room = 2
    elif room == 1 and tile_type == 11:
        print("we are in room 3")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 3")
        room = 3
    elif room == 1 and tile_type == 9:
        print("we are in room 4 (scaryyy)")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 4 (scaryyy)")
        room = 4

    elif room == 2 and tile_type == 6:
        print("we are in room 1")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 1")
        room = 1
    elif room == 2 and tile_type == 7:
        print("we are in room 3")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 3")
        room = 3
    elif room == 2 and tile_type == 10:
        print("we are in room 4 (scarrry)")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 4 (scarrry)")
        room = 4

    elif room == 3 and tile_type == 8:
        print("we are in room 4 (scaryy)")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 4 (scaryy)")
        room = 4
    elif room == 3 and tile_type == 7:
        print("we are in room 2")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 2")
        room = 2
    elif room == 3 and tile_type == 11:
        print("we are in room 1")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 1")
        room = 1

    elif room == 4 and tile_type == 8:
        print("we are in room 3")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 3")
        room = 3
    elif room == 4 and tile_type == 10:
        print("we are in room 2")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 2")
        room = 2
    elif room == 4 and tile_type == 9:
        print("we are in room 1")
        rooms_chenged_in_whatroom=[]
        logging.info("we are in room 1")
        room = 1
    else:
        print("WHAT ROOM BUGGGGG.")
        for i in rooms_chenged_in_whatroom:
            r2_map[i[0]][i[1]]["room"]=0
            logging.info(f"rooms reset : {i}")
        logging.warning(f"WHAT ROOM BUGGGGG. tilenum {tile_type}. room: {room}. ({x}, {y})")

    return room


def abs_yaw():
    yaw = getyaw()
    margin = 22.5
    if 90 - margin < yaw < 90 + margin:
        yaw = 90
    if 180 - margin < yaw < 180 + margin:
        yaw = 180
    if 270 - margin < yaw < 270 + margin:
        yaw = 270
    if 360 - margin < yaw or yaw < 0 + margin:
        yaw = 360
    if 45 - margin < yaw < 45 + margin:
        yaw = 45
    if 135 - margin < yaw < 135 + margin:
        yaw = 135
    if 225 - margin < yaw < 225 + margin:
        yaw = 225
    if 315 - margin < yaw < 315 + margin:
        yaw = 315
    return (yaw)


def send_map(f_map):
    s = f_map.shape
    ## Get shape as bytes
    s_bytes = struct.pack('2i', *s)

    ## Flattening the matrix and join with ','
    flatMap = ','.join(f_map.flatten())
    ## Encode
    sub_bytes = flatMap.encode('utf-8')

    ## Add togeather, shape + map
    a_bytes = s_bytes + sub_bytes

    ## Send map data
    emitter.send(a_bytes)

    # STEP3 Send map evaluate request
    map_evaluate_request = struct.pack('c', b'M')
    emitter.send(map_evaluate_request)


def rotation(final_deg, doVision: bool = True):
    if final_deg == 0:
        final_deg = 360
    if final_deg == abs_yaw():
        # print()
        return
    if final_deg == 360:
        final_deg = 0

    yaw = getyaw()
    presition = 0.1
    k = 0.5
    girkardim = 0.002
    gircount = 0
    
    fsp1 = (0, 0)
    ang = 0

    speed = max_velocity
    while robot.step(timeStep) != -1:
        if doVision:
            vision()
        yaw = getyaw()

        error = yaw - final_deg
        
        fsp2 = getgps()
        ang2 = getyaw()

        # print(error)
        if error > 0:
            speed = (abs(error) * k)
            if abs(speed) > max_velocity:
                speed = max_velocity
            if abs(error) > abs(180):
                speed = -speed
            go(-speed, speed)
            if abs(speed) < presition:
                go(0, 0)
                break
        elif error < 0:

            error = yaw - final_deg
            (speed) = (abs(error) * k)
            if abs(speed) > max_velocity:
                speed = max_velocity
            if abs(error) > abs(180):
                speed = -speed
            go(speed, -speed)
            if abs(speed) < presition:
                go(0, 0)
                break
        
        if (abs(fsp2[0] - fsp1[0])) + abs(fsp2[1] - fsp1[1]) < girkardim and abs(ang2 - ang) < 1:
            gircount += 1
            if gircount > 7:
                print("We are giridim in the rotation")
                logging.warning("We are giridim in rotation.")
                go(0, 0)
                return
        
        fsp1 = fsp2
        ang = ang2
    lidarProMax()


def rotation_for_degrees(degrees: int, doVision: bool = True):
    _final_deg = getyaw() + degrees
    _final_deg = _final_deg - 360 if _final_deg > 360 else _final_deg
    _final_deg = _final_deg + 360 if _final_deg < 0 else _final_deg
    rotation(_final_deg, doVision=doVision)
    go(0, 0)


def remove_duplicates(input_list):
    flattened_items = [tuple(item) if isinstance(item, list) else item for item in input_list]
    unique_items = list(set(flattened_items))
    return unique_items


doObstacleResetLop: bool = True
def lopDecision(targets: list | None):
    global lopNoPathCount, lopNoPathTargets, doObstacleResetLop
    
    logging.warning(f"lop decision. targets: {targets}, lop count: {lopNoPathCount}, lop targets: {lopNoPathTargets}")
    if lopNoPathCount >= 4 and targets is not None:
        print("Auto GiveUp")
        logging.warning(f"Auto Give UP. ({x}, {y})")
        exitProcess()
    elif lopNoPathCount >= 3 and targets is not None:
        logging.warning(f"Removing the targets.... {lopNoPathTargets}")
        lopNoPathCount += 1
        if targets is not None:
            lopNoPathTargets.append(targets)
        
        for i in range(len(lopNoPathTargets)):
            noPathTargets = remove_duplicates(lopNoPathTargets)
            for i in range(len(noPathTargets)):
                for j in range(len(noPathTargets[i])):
                    tX, tY = noPathTargets[i][j]
                    r2_map[tX][tY]["status"] = -2
                    
        lopNoPathTargets = []
        
    elif lopNoPathCount >= 2 and doObstacleResetLop:
        logging.warning("Resetting -2 type rooms")
        lopNoPathCount += 1
        doObstacleResetLop = False
        if targets is not None:
            lopNoPathTargets.append(targets)
            
        for i in range(x_min, x_max + 1):
            for j in range(y_min, y_max + 1):
                if r2_map[i][j]["status"] == -2:
                    r2_map[i][j]["status"] = 1
                    r2_map[i][j]["type"] = 1
                    r2_map[i][j]["walls"] = np.zeros((5, 5), dtype='int32')
    elif lopNoPathCount >= 2 and not doObstacleResetLop:
        logging.info("Adding a number to lop no path count.")
        lopNoPathCount += 1
    else:
        lopNoPathCount += 1
        lopNoPathTargets.append(targets)
        logging.warning(f"No pass LOP {lopNoPathCount}")
        
        output = np.array(lidar_output())
        output[output < 7] = True
        output[output >= 7] = False
        lackofprog(rotate_when_lop=output.any())
    while robot.step(timeStep) != -1:
        getgps()
        setLocation()
        break
    

def lopOnHole():
    if r2_map[x][y]["type"] == 2:
        print("LOP ON HOLE")
        logging.warning(f"LOP POSITION ON HOLE ({x}, {y})")
        lackofprog()
        return True
    return False


def right_search():
    global startpx, startpy, startp, lopNoPathCount, lopNoPathTargets, r2_map, x, y
    startpx, startpy = startp
    setLocation()

    if lopOnHole():
        return
    
    logging.info(f"Searching Started... Location: {x}, {y}")
    path, targets = astarnearest()
    logging.info(f"AStar Nearest Completed with result. Path: {path} and Targets: {targets}")
    if path == -1:
        lopDecision(targets)
        return
    else:
        lopNoPathCount = 0
        lopNoPathTargets = []

    time, score = gameinfo()
    logging.info(f"Game Info. Time: {time} and Score: {score}")
    logging.info(f"Position: {x}, {y}")

    checkTimeIsLow(time, True)
    
    if not path:
        go(0, 0)
        # way2, cost = Astar((x, y), (startpx, startpy), inf)
        # wayCheckpoint, costCheckpoint = Astar(lastVisitedCheckpoint, (startpx, startpy), inf)

        logging.info("There is no targets to go...")
        print("We searched all of the tiles...")
        
        lopNoPathCount = 0
        while robot.step(timeStep) != -1 and (x, y) != (startpx, startpy):
            setLocation()
            way2, cost = Astar((x, y), (startpx, startpy), inf)
            if not way2:
                lopDecision([])
                continue
            else:
                lopNoPathCount = 0
                lopNoPathTargets = []
            if astarmovment(way2, True):
                logging.info("We are at the start...")
                break

        exitProcess()

    logging.info("Regular Movement.")
    
    astarmovment(path, True)

    return


# **********************mapping**********************
def rollPitchHaveDeviation(high_deviation: bool = False) -> bool:
    precision: float = 3.7 if not high_deviation else 6
    roll, pitch = getRoll(), getPitch()
    
    if abs(roll) > precision or abs(pitch) > precision:
        # print(f"we have high deviation with roll : {abs(roll)}  and pitch: {abs(pitch)} ")
        return True
    return False


def colorTileMapping(p, q, tile_type: int | None, tile_status: int | None, tile_room: int | None):
    global cordinent, x, y
    yaw = abs_yaw()
    global r2_map
    p = (mapping_size // 2) + round((p - cordinent[0]) / 6)
    q = (mapping_size // 2) + round((q - cordinent[1]) / 6)

    logging.info(f"Mapping color tile with type: {tile_type}")

    if p % 2 == 0 and q % 2 == 0:
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if r2_map[p + i][q + j]["type"] in [3, 4,2] and tile_type!=2:# and tile_type != 2:
                    logging.info(f"Color tile mapping continue in loop 1. tile type: {r2_map[p + i][q + j]['type']}, (i, j): ({i}, {j}), (p, q): ({p}, {q})")
                    continue
                if tile_type is not None:
                    r2_map[p + i][q + j]["type"] = tile_type
                if tile_room is not None and r2_map[p + i][q + j]["room"] in [0,4]:
                    r2_map[p + i][q + j]["room"] = tile_room
                if tile_status is not None and r2_map[p + i][q + j]["status"] != -1:
                    r2_map[p + i][q + j]["status"] = tile_status
    elif not (p % 2 == 1 and q % 2 == 1):
        if p % 2 == 1:
            if yaw in [90, 45, 135]:
                for i in range(3):
                    for j in range(3):
                        if r2_map[p + j][q + i - 1]["type"] in [3, 4,2] and tile_type!=2:#and tile_type != 2:
                            logging.info(f"Color tile mapping continue in loop 2. tile type: {r2_map[p + j][q + i - 1]['type']}, (i, j): ({i}, {j}), (p, q): ({p}, {q})")
                            logging.info(f"the tiles map is{r2_map[p + j][q + i - 1]}")
                            continue
                        if tile_type is not None:
                            r2_map[p + j][q + i - 1]["type"] = tile_type
                        if tile_room is not None and r2_map[p + j][q + i - 1]["room"]  in [0,4]:
                            r2_map[p + j][q + i - 1]["room"] = tile_room
                        if tile_status is not None and r2_map[p + j][q + i - 1]["status"] != 1:
                            r2_map[p + j][q + i - 1]["status"] = tile_status
                        # print((p+j,q+i))
            elif yaw in [270, 225, 315]:
                for i in range(3):
                    for j in range(3):
                        if r2_map[p - j][q + i - 1]["type"] in [3, 4,2] and tile_type!=2:#and tile_type != 2:
                            logging.info(f"Color tile mapping continue in loop 3. tile type: {r2_map[p - j][q + i - 1]['type']}, (i, j): ({i}, {j}), (p, q): ({p}, {q})")
                            continue
                        if tile_type is not None:
                            r2_map[p - j][q + i - 1]["type"] = tile_type
                        if tile_room is not None and r2_map[p - j][q + i - 1]["room"] in [0,4]:
                            r2_map[p - j][q + i - 1]["room"] = tile_room
                        if tile_status is not None and r2_map[p - j][q + i - 1]["status"] != 1:
                            r2_map[p - j][q + i - 1]["status"] = tile_status
                        # print((p-j,q+i))
            else:
                if r2_map[p][q]["type"] != 4 or tile_type==2:  # Maybe have bug...
                    if tile_type is not None:
                        r2_map[p][q]["type"] = tile_type
                    if tile_room is not None and r2_map[p ][q ]["room"] in [0,4]:
                        r2_map[p][q]["room"] = tile_room
                    if tile_status is not None and r2_map[p][q]["status"] != 1:
                        r2_map[p][q]["status"] = tile_status
                else:
                    logging.info(f"Color tile mapping continue in condition 1. tile type: {r2_map[p][q]['type']}, (p, q): ({p}, {q})")
        else:
            if yaw in [180, 225, 135]: 
                for i in range(3):
                    for j in range(3):
                        if r2_map[p + i - 1][q - j]["type"] in [3, 4,2] and tile_type!=2:#and tile_type != 2:
                            logging.info(f"Color tile mapping continue in loop 4. tile type: {r2_map[p + i - 1][q - j]['type']}, (i, j): ({i}, {j}), (p, q): ({p}, {q})")
                            continue
                        if tile_type is not None:
                            r2_map[p + i - 1][q - j]["type"] = tile_type
                        if tile_room is not None and r2_map[p + i - 1][q - j]["room"] in [0,4]:
                            r2_map[p + i - 1][q - j]["room"] = tile_room
                        if tile_status is not None and r2_map[p + i - 1][q - j]["status"] != 1:
                            r2_map[p + i - 1][q - j]["status"] = tile_status
                        # print((p+i,q-j))

            elif yaw in [360, 45, 315]:
                for i in range(3):
                    for j in range(3):
                        if r2_map[p + i - 1][q + j]["type"] in [3, 4,2] and tile_type!=2:#and tile_type != 2:
                            logging.info(f"Color tile mapping continue in loop 5. tile type: {r2_map[p + i - 1][q + j]['type']}, (i, j): ({i}, {j}), (p, q): ({p}, {q})")
                            continue
                        if tile_type is not None:
                            r2_map[p + i - 1][q + j]["type"] = tile_type
                        if tile_room is not None and r2_map[p + i - 1][q + j]["room"] in [0,4]:
                            r2_map[p + i - 1][q + j]["room"] = tile_room
                        if tile_status is not None and r2_map[p + i - 1][q + j]["status"] != 1:
                            r2_map[p + i - 1][q + j]["status"] = tile_status
                        # print((p+i,q+j))
            else:
                if r2_map[p][q]["type"] != 4 or tile_type==2:   # Maybe have bug...
                    if tile_type is not None:
                        r2_map[p][q]["type"] = tile_type
                    if tile_room is not None and r2_map[p ][q ]["room"] in [0,4]:
                        r2_map[p][q]["room"] = tile_room
                    if tile_status is not None and r2_map[p][q]["status"] != 1:
                        r2_map[p][q]["status"] = tile_status
                else:
                    logging.info(f"Color tile mapping continue in condition 2. tile type: {r2_map[p][q]['type']}, (p, q): ({p}, {q})")
    else:
        if r2_map[p][q]["type"] != 4 or tile_type==2:  # Maybe have bug...
            if tile_type is not None:
                r2_map[p][q]["type"] = tile_type
            if tile_room is not None and r2_map[p][q]["room"] in [0,4]:
                r2_map[p][q]["room"] = tile_room
            if tile_status is not None and r2_map[p][q]["status"] != 1:
                r2_map[p][q]["status"] = tile_status
        else:
            logging.info(f"Color tile mapping continue in condition 3. tile type: {r2_map[p][q]['type']}, (p, q): ({p}, {q})")


def get_lidar():
    rangeImage = lidar.getRangeImage()
    rangeImage = rangeImage[1024:1536]
    rangeImage = np.array(rangeImage)
    rangeImage = rangeImage * 100
    rangeImage = rangeImage.tolist()
    return rangeImage


def lidar_output():
    rangeImage = lidar.getRangeImage()
    f = 100 * rangeImage[1024]
    r = 100 * rangeImage[1152]
    b = 100 * rangeImage[1280]
    l = 100 * rangeImage[1408]
    f_signal = rangeImage[1024]
    r_signal = rangeImage[1048]
    l_signal = rangeImage[1512]
    r_signal = r_signal * cos(radians(16.875))
    l_signal = l_signal * cos(radians(-16.875))
    l_signal = l_signal * 100
    r_signal = r_signal * 100
    f_signal = f_signal * 100
    spacing_control = []
    spacing_control.append([r_signal, l_signal, f_signal])
    f = min(spacing_control[0])
    return f, r, l, b


def setLocation():
    global cordinent, x, y, xpre, ypre
    now = getgps()
    new_x = (mapping_size // 2) + round((now[0] - cordinent[0]) / 6)
    new_y = (mapping_size // 2) + round((now[1] - cordinent[1]) / 6)

    changed = False
    if (x, y) != (new_x, new_y):
        xpre, ypre = x, y
        x, y = new_x, new_y
        changed = True

    return x, y, changed


def  give_location(xin, yin):
    global cordinent
    xx = (mapping_size // 2) + round((xin - cordinent[0]) / 6)
    yy = (mapping_size // 2) + round((yin - cordinent[1]) / 6)
    return xx, yy


def getLocation(cords):
    achual_x = cordinent[0] + ((cords[0] - (mapping_size // 2)) * 6)
    achual_y = cordinent[1] + ((cords[1] - (mapping_size // 2)) * 6)
    return (achual_x, achual_y)


def minMax(curX, curY):
    global x_max, x_min, y_max, y_min
    if curX > x_max:
        x_max = curX
    if x_min > curX:
        x_min = curX
    if curY > y_max:
        y_max = curY
    if y_min > curY:
        y_min = curY
def check_targets():
    return
    if room4checkup:
        return
    global r2_map
    room_fill=np.zeros((5,5),dtype=np.int8)
    for i in range(x_min,x_max+1):
        for j in range(y_min,y_max+1):
            if r2_map[i][j]["status"]==0 and r2_map[i][j]["room"]==4:
                # print(i,j)
                # print(tilewayslst)
                is_true=not np.any(r2_map[i][j]["walls"])
                if is_true:
                    # print(r2_map[i][j]["status"])
                    r2_map[i][j]["status"]=1
                    # print(r2_map[i][j]["status"])
                    # print("is changed")

                

def  lidarProMax(mapColorTiles: bool = True):
    global r2_map, x, y, room, lastVisitedCheckpoint,rooms_chenged_in_whatroom

    if rollPitchHaveDeviation():
        logging.warning("Roll / Pitch have deviation in lidarProMax.")
        tilenum = tileType()
        if mapColorTiles and tilenum > 2:
            if tilenum == 3:
                logging.info("Swamp Visited.")
            elif tilenum == 4:
                logging.info("Checkpoint Visited.")
            
            colorTileMapping(*getLocation((x, y)), tilenum, None, room if tilenum < 6 else 0)
        return

    xN, yN = getLocation((x, y))
    x_stable, y_stable = getgps(0)
    distanceFromCenter = sqrt((xN - x_stable) ** 2 + (yN - y_stable) ** 2)

    if distanceFromCenter >= 4:
        return
 
    r = 1.1
    if room == 4 or r2_map[x][y]["room"] == 4:
        r = 0.6

    mapping = np.zeros([13, 13], dtype='int32')
    wallCountPrecisiton = 6
    rangeImage = get_lidar()
    deg = 0
    yaw = getyaw() - 180
    yaw = math.radians(yaw)
    rot_map = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])

    xN, yN = getgps()
    x_stable, y_stable = getLocation((x, y))

    for i in rangeImage:
        if i == float("inf"):
            deg += 360 / 512
            continue
        i = i + 0.5
        rad = math.radians(deg)
        xxx = i * sin(rad)
        yyy = i * cos(rad)
        l_cord = np.dot(rot_map, [xxx, yyy]).tolist()

        yy = l_cord[1]
        xx = l_cord[0]
        xx = xx - (x_stable - xN)
        yy = yy + (y_stable - yN)
        xx = xx + 19.5
        yy = yy + 19.5
        the_exact_xx = xx
        the_exact_yy = yy
        xx = xx // 3
        yy = yy // 3
        if not (0 <= xx <= 12 and 0 <= yy <= 12):
            deg += 360 / 512
            continue
        xx_cm = (xx * 3) + 1.5
        yy_cm = (yy * 3) + 1.5

        if (((the_exact_xx - xx_cm) ** 2 + (the_exact_yy - yy_cm) ** 2)) > float(r ** 2):
            deg += 360 / 512
            continue
        yy = 12 - yy

        i_index = int(yy)
        i0_index = int(xx)
        mapping[i_index, i0_index] += 1
        deg += 360 / 512

    for i in range(5):
        for j in range(5):
            y_index = (j - 2) + y
            x_index = (i - 2) + x
            r2_index_x = (j + 1) * 2
            r2_index_y = (i + 1) * 2                                                   

            r2_map[x_index][y_index]["walls count"] += mapping[r2_index_x - 2:r2_index_x + 3,
                                                               r2_index_y - 2:r2_index_y + 3]

    for i in range(13):
        for j in range(13):
            if mapping[i][j] > wallCountPrecisiton:
                mapping[i][j] = 1
            else:
                mapping[i][j] = 0

    tilenum = tileType()
    # TODO
    r2_map[x][y]["walls"] = mapping[4:9, 4:9]
    r2_map[x][y]["status"] = 1
    
    if r2_map[x][y]["type"] == 0:
        r2_map[x][y]["type"] = 1
    up, down, left, right, ul, ur, dl, dr = tileways(x, y)
    # TODO
    PredictPresition = 100
    # PredictPresition = 200 # 300 ham khoobe
    
    for i in range(3):
        for j in range(3):
            x_use = i - 1
            y_use = j - 1
            # TODO
            if r2_map[x + x_use][y + y_use]["status"] in [0, -1, 1]:
                r2_map[x + x_use][y + y_use]["walls"][r2_map[x + x_use][y + y_use]["walls count"] < PredictPresition] = 0
                r2_map[x + x_use][y + y_use]["walls"][r2_map[x + x_use][y + y_use]["walls count"] >= PredictPresition] = 1

    if up == True and r2_map[x][y - 1]["status"] == -1 and not r2_map[x][y - 1]["walls"][1, 2].any():
        r2_map[x][y - 1]["status"] = 0

            
        minMax(x, y - 1)
    elif up == False and r2_map[x][y - 1]["status"] == -1 and r2_map[x][y - 1]["walls"][1, 2].any():
        if room == 4 and r2_map[x][y - 1]["room"] == 0:
            r2_map[x][y - 1]["room"] = 4

    if down == True and r2_map[x][y + 1]["status"] == -1 and not r2_map[x][y + 1]["walls"][3, 2].any():
        r2_map[x][y + 1]["status"] = 0

        minMax(x, y + 1)
    elif down == False and r2_map[x][y + 1]["status"] == -1 and r2_map[x][y + 1]["walls"][3, 2].any():
        if room == 4 and r2_map[x][y + 1]["room"] == 0:
            r2_map[x][y + 1]["room"] = 4

    if right == True and r2_map[x + 1][y]["status"] == -1 and not r2_map[x + 1][y]["walls"][2, 1].any():
        r2_map[x + 1][y]["status"] = 0

        minMax(x + 1, y)
    elif right == False and r2_map[x + 1][y]["status"] == -1 and r2_map[x + 1][y]["walls"][2, 1].any():
        if room == 4 and r2_map[x + 1][y]["room"] == 0:
            r2_map[x + 1][y]["room"] = 4

    if left == True and r2_map[x - 1][y]["status"] == -1 and not r2_map[x - 1][y]["walls"][2, 3].any():
        r2_map[x - 1][y]["status"] = 0


        minMax(x - 1, y)
    elif left == False and r2_map[x - 1][y]["status"] == -1 and r2_map[x - 1][y]["walls"][2, 3].any():
        if room == 4 and r2_map[x - 1][y]["room"] == 0:
            r2_map[x - 1][y]["room"] = 4

    if ul == True and r2_map[x - 1][y - 1]["status"] == -1 and not r2_map[x - 1][y - 1]["walls"][1:4, 1:4].any():
        r2_map[x - 1][y - 1]["status"] = 0

        minMax(x - 1, y - 1)
    elif ul == False and r2_map[x - 1][y - 1]["status"] == -1 and r2_map[x - 1][y - 1]["walls"][1:4, 1:4].any():
        if room == 4 and r2_map[x - 1][y - 1]["room"] == 0:
            r2_map[x - 1][y - 1]["room"] = 4

    if ur == True and r2_map[x + 1][y - 1]["status"] == -1 and not r2_map[x + 1][y - 1]["walls"][1:4, 1:4].any():
        r2_map[x + 1][y - 1]["status"] = 0

        minMax(x + 1, y - 1)
    elif ur == False and r2_map[x + 1][y - 1]["status"] == -1 and r2_map[x + 1][y - 1]["walls"][1:4, 1:4].any():
        if room == 4 and r2_map[x + 1][y - 1]["room"] == 0:
            r2_map[x + 1][y - 1]["room"] = 4

    if dl == True and r2_map[x - 1][y + 1]["status"] == -1 and not r2_map[x - 1][y + 1]["walls"][1:4, 1:4].any():
        r2_map[x - 1][y + 1]["status"] = 0

        minMax(x - 1, y + 1)
    elif dl == False and r2_map[x - 1][y + 1]["status"] == -1 and r2_map[x - 1][y + 1]["walls"][1:4, 1:4].any():
        if room == 4 and r2_map[x - 1][y + 1]["room"] == 0:
            r2_map[x - 1][y + 1]["room"] = 4

    if dr == True and r2_map[x + 1][y + 1]["status"] == -1 and not r2_map[x + 1][y + 1]["walls"][1:4, 1:4].any():
        r2_map[x + 1][y + 1]["status"] = 0

        minMax(x + 1, y + 1)
    elif dr == False and r2_map[x + 1][y + 1]["status"] == -1 and r2_map[x + 1][y + 1]["walls"][1:4, 1:4].any():
        if room == 4 and r2_map[x + 1][y + 1]["room"] == 0:
            r2_map[x + 1][y + 1]["room"] = 4

    # if mapColorTiles and r2_map[x][y]["type"] != tilenum and tilenum > 2 and x % 2 == 0 and y % 2 == 0:
    #     if tilenum == 3:
    #         logging.info("Swamp Visited.")
    #     elif tilenum == 4:
    #         logging.info("Checkpoint Visited.")
    
    tilenum = tileType()
    if mapColorTiles and tilenum > 2:
        if tilenum == 3:
            logging.info("Swamp Visited.")
        elif tilenum == 4:
            logging.info("Checkpoint Visited.")

        colorTileMapping(*getLocation((x, y)), tilenum, None, room if tilenum < 6 else 0)

    if x % 2 == 0 and y % 2 == 0 and tilenum == 4:
        lastVisitedCheckpoint = (x, y)

    if r2_map[x][y]["room"] == 0 and r2_map[x][y]["type"] < 6 and tilenum < 6:
        r2_map[x][y]["room"] = room
        rooms_chenged_in_whatroom.append([x,y])
    elif r2_map[x][y]["type"] < 6 and tilenum < 6 and x % 2 == 0 and y % 2 == 0:
        if r2_map[x][y]["room"] != room:
            print(f"Room changed to {r2_map[x][y]['room']}")
            logging.info(f"Room changed to {r2_map[x][y]['room']} from {room}")
        room = r2_map[x][y]["room"]

def guess_room_tileways(p, q):
    ul, ur, dl, dr, up, down, left, right = True, True, True, True, True, True, True, True

    area = r2_map[p][q]["walls"]

    if area[1:4, 0:2].all():
        left = False
    if area[1:4, 3:5].all():
        right = False
    if area[0:2, 1:4].all():
        up = False
    if area[3:5, 1:4].all():
        down = False
    if area[0, 0:2].all() and area[2, 1] and area[1, 2] and area[1,0]:
        ul = False

    if area[0, 3:5].all() and area[1, 2] and area[2, 3] and area[1,3]:
        ur = False

    if area[4, 3:5].all() and area[2, 3] and area[3, 2] and area[3,4]:
        dr = False

    if area[4, 0:2].all() and area[2, 1]and area[3, 2] and area[3,0]:
        dl = False
    return up, down, left, right, ul, ur, dl, dr
def get_available_ways_guess(final_pos):

    available_ways = []
    # index_list = ["180", "360", "270", "90", "225", "135", "315", "45"]

    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if i == 0 and j == 0:
                continue

            dirdin = mapdin2dirdin(i, j)
            tilewaysLST = guess_room_tileways(final_pos[0] + i, final_pos[1] + j)
            tilewaysLST_check = guess_room_tileways(final_pos[0], final_pos[1])
            tilewaysdin = tilewaysdin_alternative(i, j)

            if tilewaysLST[tilewaysdin] or tilewaysLST_check[dirdin]:
                available_ways.append(True)
            else:
                available_ways.append(False)
                
                

    return available_ways

def guess_room():

    global r2_map
    needGuessing = []
    guessed=[]

    for i in range(x_min, x_max + 1):
        for j in range(y_min, y_max + 1):
            if r2_map[i][j]["room"] == 0 and r2_map[i][j]["type"] < 6 :
                needGuessing.append((i, j))

    logging.info(f"Nodes that need room guessing: {needGuessing}")

    for gX, gY in needGuessing:
        roomWaysCountDict = {}

        
        # tileWays = get_available_ways_guess([gX, gY])
        tileWays=guess_room_tileways(gX,gY)

        for i in range(8):
            if tileWays[i] is True:
                cellx, celly = gX, gY
                chcell = (cellx, celly)
                if i == 0:
                    chcell = (cellx, celly - 1)
                elif i == 1:
                    chcell = (cellx, celly + 1)
                elif i == 2:
                    chcell = (cellx - 1, celly)
                elif i == 3:
                    chcell = (cellx + 1, celly)
                elif i == 4:
                    chcell = (cellx - 1, celly - 1)
                elif i == 5:
                    chcell = (cellx + 1, celly - 1)
                elif i == 6:
                    chcell = (cellx - 1, celly + 1)
                elif i == 7:
                    chcell = (cellx + 1, celly + 1)
                cellx, celly = chcell

                thisRoom = r2_map[cellx][celly]["room"]
                if thisRoom == 0:
                    continue
                if r2_map[cellx][celly]["type"]>=6 or r2_map[cellx][celly]["type"]==2:
                    continue 

                if thisRoom in roomWaysCountDict:
                    roomWaysCountDict[thisRoom] += 1
                else:
                    roomWaysCountDict[thisRoom] = 1

        roomWays = list(roomWaysCountDict.items())  # [(room, count), (room, count)]
        roomWays = np.array(roomWays, dtype='int32')  # [[room, count] [room, count]]
        # print("Room ways: ", gX, gY, roomWays)
        if len(roomWays) > 0:
            maxRoomIndex = np.argmax(roomWays[:, 1])
            # if roomWays[maxRoomIndex,1]in roomWays[:,1]:
                # continue
            guessedRoom = roomWays[maxRoomIndex, 0]
            # if roomWays[maxRoomIndex,1]>0:
            # print("this is getting guessed", guessedRoom)
            r2_map[gX][gY]["room"] = guessedRoom
            guessed.append((gX,gY,"gussed room: ",guessedRoom))
    logging.info(f"tiles we room gussed: {guessed}")


def removeObstacles(f_map: np.ndarray, wallCountPrecision: int = 5):
    global r2_map
    obstacleRemoved_tiles = []
    for i in range(x_min, x_max + 1):
        for j in range(y_min, y_max + 1):
            if i % 2 == 0 and j % 2 == 0:
                f_map_x, f_map_y = i - x_min, j - y_min
                walls = r2_map[i][j]["walls count"].copy()
                walls[walls < wallCountPrecision] = 0
                walls[walls >= wallCountPrecision] = 1
                originalWalls = walls.copy()
                curRoom = r2_map[i][j]["room"]
                if curRoom == 1:
                    walls_clean = obstacleRemover.room1_delete_obstacles(walls)
                elif curRoom == 2:
                    walls_clean = obstacleRemover.room2_delete_obstacles(walls)
                elif curRoom == 3:
                    walls_clean = obstacleRemover.room3_delete_obstacles(walls)
                else:
                    walls[0, 0] = originalWalls[0, 0]
                    walls[0, 4] = originalWalls[0, 4]
                    walls[4, 0] = originalWalls[4, 0]
                    walls[4, 4] = originalWalls[4, 4]
                    f_map[f_map_y * 2: f_map_y * 2 + 5, f_map_x * 2: f_map_x * 2 + 5] = walls.copy()
                    continue

                walls_clean[0, 0] = originalWalls[0, 0]
                walls_clean[0, 4] = originalWalls[0, 4]
                walls_clean[4, 0] = originalWalls[4, 0]
                walls_clean[4, 4] = originalWalls[4, 4]
                r2_map[i][j]["walls"] = walls_clean.copy()
                walls_count = r2_map[i][j]["walls count"].copy()
                walls_count[walls == 0] = 0
                r2_map[i][j]["walls count"] = walls_count.copy()

                obstacleRemoved_tiles.append((i, j))

                # applying changes to the f_map
                f_map[f_map_y * 2: f_map_y * 2 + 5, f_map_x * 2: f_map_x * 2 + 5] += walls_clean.copy()

    f_map[f_map > 1] = 1

    return f_map


def print_map(r2_map, deleteObstacle=False, sending_map: bool = False):
    global x_min,x_max,y_min,y_max
    # victim_map=np.zeros((((y_max-y_min+1)*2)+3,((x_max-x_min+1)*2)+3), dtype="int32")
    # y_max = y_max if y_max % 2 == 0 else y_max + 1
    # y_min = y_min if y_min % 2 == 0 else y_min - 1
    # x_max = x_max if x_max % 2 == 0 else x_max + 1
    # x_min = x_min if x_min % 2 == 0 else x_min - 1
    # y_max, y_min, x_max, x_min = y_max, y_min, x_max, x_min

    if sending_map:
        logging.info("Started room guessing.")
        guess_room()

    # y_max, y_min, x_max, x_min = y_max, y_min, x_max, x_min
    f_map = np.zeros((((y_max - y_min + 1) * 2) + 3, ((x_max - x_min + 1) * 2) + 3), dtype="int32")
    
    # TODO
    wallsCountPrecision = 10
    if sending_map:
        logging.info("Started removing obstacles.")
        f_map = removeObstacles(f_map, wallsCountPrecision)
    else:
        for i in range((x_max + 1) - x_min):
            for j in range((y_max + 1) - y_min):
                f_map[2 * j:2 * j + 5, 2 * i:2 * i + 5] += r2_map[x_min + i][y_min + j]["walls count"]
            # f_map[2*j:2*j+5,2*i:2*i+5]+=r2_map[x_min+i][y_min+j]["victim"]

        f_map[f_map < wallsCountPrecision] = 0
        f_map[f_map >= wallsCountPrecision] = 1

    f_map = f_map.astype('U3')

    types_dict = {
        0: 0,
        1: 1,
        2: 2,
        3: 3,
        4: 4,
        6: 'b',
        7: 'p',
        8: 'r',
        9: 'g',
        10: 'o',
        11: 'y'
    }
    for i in range(x_min, x_max + 1):
        for j in range(y_min, y_max + 1):
            if i % 2 == 0 and j % 2 == 0:
                # f_map[2*j+2,2*i+2]=r2_map[x_min+i][y_min+j]["type"]
                tile_type = r2_map[i][j]["type"]
                if tile_type >= 2:
                    tile_type = types_dict[tile_type]
                elif tile_type < 2: 
                    continue
                f_map[2 * (j - y_min) + 3, 2 * (i - x_min) + 1] = tile_type
                f_map[2 * (j - y_min) + 1, 2 * (i - x_min) + 1] = tile_type
                f_map[2 * (j - y_min) + 3, 2 * (i - x_min) + 3] = tile_type
                f_map[2 * (j - y_min) + 1, 2 * (i - x_min) + 3] = tile_type
            

            # if r2_map[i][j]["type"] < 2:
            #     f_map[2*(j-y_min)+2,2*(i-x_min)+2]=r2_map[i][j]["type"]

            # print each tile is which room?????????????????????????????????????????
            # if f_map[2*j+2,2*i+2] != 1:
            #     f_map[2*j+2,2*i+2] = r2_map[x_min+i][y_min+j]["room"]
    room4_fill = np.full((5, 5), "*")

    for i in range((x_max + 1) - x_min):
        for j in range((y_max + 1) - y_min):
            if (i + x_min) % 2 == 0 and (j + y_min) % 2 == 0 and r2_map[i + x_min][j + y_min]["room"] == 4:#
                if r2_map[i + x_min][j + y_min]["type"] < 6 :
                    if (i + x_min) % 2 == 1 or (j + y_min) % 2 == 1:
                        continue
                    f_map[2 * j:2 * j + 5, 2 * i:2 * i + 5] = room4_fill

    # Victim Mapping
    wall_tokens = ["H", "S", "U", "O", "F", "P", "C"]           
    for i in range((x_max + 1) - x_min):
        for j in range((y_max + 1) - y_min):
            if r2_map[i + x_min][j + y_min]["HSU"] != "none":
                victim_mat = f_map[2 * j:2 * j + 5, 2 * i:2 * i + 5]
                for k in range(5):
                    for l in range(5):
                        if r2_map[i + x_min][j + y_min]["victim"][k, l] != '':
                            if f_map[2 * j + k, 2 * i + l] == '1':
                                victim_mat[k, l] = r2_map[i + x_min][j + y_min]["victim"][k, l]
                            elif f_map[2 * j + k, 2 * i + l] in wall_tokens:
                                victim_mat[k, l] += r2_map[i + x_min][j + y_min]["victim"][k, l]

                f_map[2 * j:2 * j + 5, 2 * i:2 * i + 5] = victim_mat
                # f_map[2*(j-y_min)+k,2*(i-x_min)+l] = r2_map[i][j]["victim"][k, l]

    for i in range((x_max + 1) - x_min):
        for j in range((y_max + 1) - y_min):
            if (i + x_min) % 2 == 0 and (j + y_min) % 2 == 0:
                victim_mat = f_map[2 * j:2 * j + 5, 2 * i:2 * i + 5]
                if victim_mat[0, 2] in wall_tokens:
                    if victim_mat[0, 1] != '0':
                        victim_mat[0, 1] = victim_mat[0, 2] if victim_mat[0, 1] == '1' \
                            else victim_mat[0, 1] + victim_mat[0, 2]
                        victim_mat[0, 2] = '1'
                if victim_mat[2, 0] in wall_tokens:
                    if victim_mat[1, 0] != '0':
                        victim_mat[1, 0] = victim_mat[2, 0] if victim_mat[1, 0] == '1' \
                            else victim_mat[1, 0] + victim_mat[2, 0]
                        victim_mat[2, 0] = '1'
                if victim_mat[4, 2] in wall_tokens:
                    if victim_mat[4, 1] != '0':
                        victim_mat[4, 1] = victim_mat[4, 2] if victim_mat[4, 1] == '1' \
                            else victim_mat[4, 1] + victim_mat[4, 2]
                        victim_mat[4, 2] = '1'
                if victim_mat[2, 4] in wall_tokens:
                    if victim_mat[1, 4] != '0':
                        victim_mat[1, 4] = victim_mat[2, 4] if victim_mat[1, 4] == '1' \
                            else victim_mat[1, 4] + victim_mat[2, 4]
                        victim_mat[2, 4] = '1'

    f_map[(2 * (startp[0] - y_min)) + 3, (2 * (startp[1] - x_min)) + 1] = 5
    f_map[(2 * (startp[0] - y_min)) + 1, (2 * (startp[1] - x_min)) + 1] = 5
    f_map[(2 * (startp[0] - y_min)) + 3, (2 * (startp[1] - x_min)) + 3] = 5
    f_map[(2 * (startp[0] - y_min)) + 1, (2 * (startp[1] - x_min)) + 3] = 5

    np.savetxt('numpy.txt', f_map, '%s')

    return f_map


def tileways(p, q):
    # import time
    if p > x_max or p < x_min or q > y_max or q < y_min:
        return False, False, False, False, False, False, False, False

    ul, ur, dl, dr, up, down, left, right = True, True, True, True, True, True, True, True
    area = r2_map[p][q]["walls"]
    # time1 = time.time()

    # Directly check specific indices instead of slices
    left = not (area[1, 0] or area[2, 0] or area[3, 0]) # for rayan is sth different
    #left = not (area[1, 0] or area[3, 0]  or area[2, 0] )

    right = not (area[1, 4] or area[2, 4] or area[3, 4])
    up = not (area[0, 1] or area[0, 2] or area[0, 3])
    down = not (area[4, 1] or area[4, 2] or area[4, 3])

    ul = not (area[0, 0] or area[0, 1] or area[1, 0] or (area[2, 1] and area[1, 2]))
    ur = not (area[0, 3] or area[0, 4] or area[1, 4] or (area[1, 2] and area[2, 3]))
    dl = not (area[3, 0] or area[4, 0] or area[4, 1] or (area[2, 1] and area[3, 2]))
    dr = not (area[3, 4] or area[4, 3] or area[4, 4] or (area[2, 3] and area[3, 2]))

    # time2 = time.time()
    # elapsed_time = (time2 - time1) * 1000

    # if elapsed_time > 0.1:
        # print_light_red(area)
    
    # print_yellow(f"just fucking tileways is taking {elapsed_time}")
    return up, down, left, right, ul, ur, dl, dr


def victim_map(vic, deg):
    global r2_map, checkForFarTokens, victimDetectionDistance
    setLocation()
    rangeImage = get_lidar()
    vic_matrix = r2_map[x][y]["victim"]
    x_stable, y_stable = getLocation((x, y))
    xN, yN = getgps()

    yaw = getyaw() - 180
    deg = deg - 1024
    point_deg = deg * 0.703125
    point_deg = math.radians(point_deg)

    yaw = math.radians(yaw)
    rot_map = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
    rangeImage = get_lidar()
    l_r = rangeImage[int(deg)]
    vic_movment_pos=l_r-5
    vic_lidar_index=vic_movment_pos
    xxx = (l_r + 0.5) * sin(point_deg)
    yyy = (l_r + 0.5) * cos(point_deg)
    vic_posX=(vic_movment_pos)*sin(point_deg)
    vic_posY=(vic_movment_pos)*cos(point_deg)
    # print("xxx,yyy",xxx,yyy)
    l_r = np.dot(rot_map, [xxx, yyy]).tolist()
    vic_movment_pos = np.dot(rot_map, [vic_posX, vic_posY]).tolist()
    vicPos = np.zeros(2, dtype='float64')
    l_r[0] = l_r[0] - (x_stable - xN)
    l_r[1] = l_r[1] + (y_stable - yN)
    vic_movment_pos_gps=[xN + vic_movment_pos[0],yN-vic_movment_pos[1]]
    vicPos[0] = xN + l_r[0]
    vicPos[1] = yN - l_r[1]
    # print("past past",l_r, getgps(3), vicPos, rangeImage[int(deg)])
    try:
        l_r[0] = int((l_r[0] + 7.5) // 3)
        l_r[1] = int((l_r[1] + 7.5) // 3)
        l_r[1] = 4 - (l_r[1])
    except:
        return
    victimDistance = rangeImage[int(deg)]

    if round(victimDistance, 3) <= victimDetectionDistance:
        
        print_green(f"Victim detected: {vic}")

        logging.info(f"Victim Detected. l_r: {l_r}")

        print_green(f"Victim reporting: {vic}")
        logging.info(f"Victim reporting: {vic}, victim distance: {victimDistance}, l_r: {l_r}")
        time , score =gameinfo()

        report_victim(vic, *vicPos)
        time_2 , score_2 = gameinfo()
        
        # print_green("the ditection wes correct")
        roundPos = tuple(map(lambda pos: round(pos), vicPos))
        if roundPos in r2_map[x][y]["vic_pos"]:
            print(f"Victim already seen.")
            logging.info(f"Victim already seen.")
            return
        if -1 < l_r[0] < 5 and -1 < l_r[1] < 5:
            logging.info(
                f"Victim Detected: {vic}, roundPos: {roundPos}, x, y: {x}, {y}, vic_pos: {r2_map[x][y]['vic_pos']}, "
                f"l_r: {l_r}")
            vic_matrix[l_r[1], l_r[0]] = vic
            r2_map[x][y]["HSU"] = vic
            r2_map[x][y]["victim"] = vic_matrix
            r2_map[x][y]["vic_pos"].append(roundPos)
        # r2_map[x][y]["room"] = room
    # report_victim(vic, *getgps(1))
       
    else:
        # if vic_lidar_index:
        #     forward(vic_movment_pos_gps,doVision=False)
        #     xN, yN = getgps()
        #     vicPos[0] =  vic_movment_pos_gps[0] -(x_stable - xN)
        #     vicPos[1] =  vic_movment_pos_gps[1]+ (y_stable - yN)
        #     print_green(f"Victim reporting: {vic}")
        #     logging.info(f"Victim reporting: {vic}, victim distance: {victimDistance}, l_r: {l_r}")
        #     time , score =gameinfo()
        #     nowgps=getgps()
        #     report_victim(vic, *nowgps)
        #     time_2 , score_2 = gameinfo()
        #     if score_2>score:
        #         print_green("the ditection wes correct")
        #     else:
        #         print_red("we had a misidentificarion")
        #     forward([x_stable, y_stable],doVision=False)

        #     # report_victim(vic, *getgps(1))
        # else:
        #     print_red("we had a misidentificarion")
        print_yellow(f"Victim Detected {vic} but it is not in our range.")
        logging.info(f"Victim Detected {vic} but it is not in our range. distance: {victimDistance}")


def report_victim(word, xw, yw):
    go(0, 0)
    # print("its getting reported")
    delay(1400)
    victimType = bytes(word, "utf-8")  # The victim type being sent is the letter 'H' for harmed victim

    # position = gps.getValues()  # Get the current gps position of the robot
    # xw = int(position[0] * 100)  # Get the xy coordinates, multiplying by 100 to convert from meters to cm
    # yw = int(position[2] * 100)  # We will use these coordinates as an estimate for the victim's position

    xw = int(xw)
    yw = int(yw)

    message = struct.pack("i i c", xw, yw, victimType)  # Pack the message.

    emitter.send(message)
    delay(100)

    logging.info(f"Victim {word} reported successfully in position {xw}, {yw}")

    # print("reporting completed.")

# **********************vision**********************

def sortContour(approx):
    a = []
    b = []
    #Optimization
    # print("approx",approx)
    for i in range(4):
        a.append([approx[i,0,0],approx[i,0,1]])
    # print("AAAAAAAA",a)
    #Optimization
    for i in range(4):
        min = [100,100]
        for j in range(len(a)):
            if a[j][0]<min[0]:
                min = a[j]
                ir = j
        b.append(min)
        a.pop(ir)
    if(b[0][1]>b[1][1]):
        a = b[0]
        b[0]= b[1]
        b[1]= a
    if(b[2][1]>b[3][1]):
        a = b[2]
        b[2] = b[3]
        b[3]= a
    # print(np.array([[b[2]],[b[0]],[b[1]],[b[3]]]))
    return np.array([[b[2]],[b[0]],[b[1]],[b[3]]]) 

# def vision():
#     image1 = camera1.getImage()
#     image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
#     frame1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)

#     image2 = camera2.getImage()
#     image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
#     frame2 = cv.cvtColor(image2, cv.COLOR_BGRA2BGR)

#     left_warp, left_distance = vision_model.warpFrame(frame1, 1)
#     right_warp, right_distance = vision_model.warpFrame(frame2, 2)

#     left_resize = vision_model.resize(left_warp, (9,9))
#     right_resize = vision_model.resize(right_warp, (9,9))

#     left_splits, left_mean = vision_model.split_RGB(left_resize, 1)
#     right_splits, right_mean = vision_model.split_RGB(right_resize, 2)
    
#     cv.waitKey(0)
    
#     vision_model.train(left_splits, left_mean)
#     vision_model.train(right_splits, right_mean)

#     return



# **********************app**********************
class task_worker(QThread):
    update_ui_signal = pyqtSignal(list)  # Emit a list of items

    def run(self):
        # Simulate a delay
        # Simulate fetching or creating a list of items
        self.update_ui_signal.emit(reminder)

class CmdReader(QThread):
    # Signal to send updated text from `cmd`
    text_changed_signal = pyqtSignal(str)

    def __init__(self, cmd_widget):
        super().__init__()
        self.cmd_widget = cmd_widget
        self.running = True

    def run(self):
        previous_text = ""
        while self.running:
            # Read the current text in the QTextEdit `cmd`
            current_text = self.cmd_widget.toPlainText()
            # If text has changed, emit it
            if current_text != previous_text:
                self.text_changed_signal.emit(current_text)
                previous_text = current_text

            time.sleep(1)  # Check every second

    def stop(self):
        self.running = False
# bug("this dosent fucking work ")
class SubwaySurfersGame:
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Screen dimensions
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 800, 600
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("Subway Surfers Clone")

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 50, 50)
        self.GREEN = (50, 255, 50)
        self.GREY = (180, 180, 180)

        # Clock and FPS
        self.clock = pygame.time.Clock()
        self.FPS = 60

        # Player settings
        self.PLAYER_WIDTH, self.PLAYER_HEIGHT = 50, 100
        self.player_x = self.SCREEN_WIDTH // 2 - self.PLAYER_WIDTH // 2
        self.player_y = self.SCREEN_HEIGHT - self.PLAYER_HEIGHT - 20
        self.current_lane = 1  # Start in the middle lane (index 1)
        self.lanes = [
            self.SCREEN_WIDTH // 4 - self.PLAYER_WIDTH // 2,
            self.SCREEN_WIDTH // 2 - self.PLAYER_WIDTH // 2,
            3 * self.SCREEN_WIDTH // 4 - self.PLAYER_WIDTH // 2,
        ]

        # Set player's initial position to the middle lane
        self.player_x = self.lanes[self.current_lane]

        # Obstacle settings
        self.obstacle_speed = 5
        self.obstacle_max_speed = 15  # Cap obstacle speed
        self.obstacles = []
        self.obstacle_timer = 0
        self.spawn_delay = 60  # Adjust based on speed

        # Background animation
        self.bg_scroll_speed = 5
        self.bg_y = 0

        # Fonts
        self.font = pygame.font.SysFont("arial", 30)
        self.large_font = pygame.font.SysFont("arial", 70, bold=True)

        # Score
        self.score = 0
        self.high_score = 0

        # gameinfo state
        self.running = True
        self.game_over = False

        # Lane switch settings
        self.switch_delay = 150  # Delay in frames between lane switches
        self.last_switch_time = 0

        # Jump settings
        self.is_jumping = False
        self.jump_height = 100
        self.jump_velocity = -15
        self.gravity = 1

        # Power-up settings
        self.speed_boosts = []
        self.speed_boost_duration = 200  # How long the boost lasts
        self.boost_timer = 0
        self.is_boosted = False

        # Sound settings


    def draw_player(self):
        """Draw the player character."""
        pygame.draw.rect(
            self.screen,
            self.GREEN,
            (self.player_x, self.player_y, self.PLAYER_WIDTH, self.PLAYER_HEIGHT),
            border_radius=10,
        )

    def draw_obstacles(self):
        """Draw all the obstacles."""
        for obstacle in self.obstacles:
            pygame.draw.rect(
                self.screen,
                self.RED,
                (obstacle["x"], obstacle["y"], obstacle["width"], obstacle["height"]),
                border_radius=10,
            )

    def draw_lanes(self):
        """Draw lane markers on the screen."""
        lane_width = self.SCREEN_WIDTH // 4
        for i in range(1, 4):
            pygame.draw.line(
                self.screen,
                self.GREY,
                (i * lane_width, 0),
                (i * lane_width, self.SCREEN_HEIGHT),
                5,
            )

    def display_score(self):
        """Display the current score and high score."""
        score_text = self.font.render(f"Score: {self.score}", True, self.WHITE)
        high_score_text = self.font.render(f"High Score: {self.high_score}", True, self.WHITE)
        self.screen.blit(score_text, (10, 10))
        self.screen.blit(high_score_text, (10, 50))

    def generate_obstacle(self):
        """Generate a new obstacle in a random lane."""
        lane = random.choice([0, 1, 2])
        x = self.lanes[lane]
        y = -100
        width = random.randint(40, 60)
        height = random.randint(80, 120)
        if not self.obstacles or self.obstacles[-1]["y"] > height + 50:
            self.obstacles.append({"x": x, "y": y, "width": width, "height": height})

    def generate_power_up(self):
        """Generate a speed boost power-up."""
        if random.randint(0, 100) < 5:  # 5% chance of spawning a power-up
            lane = random.choice([0, 1, 2])
            x = self.lanes[lane]
            y = -100
            self.speed_boosts.append({"x": x, "y": y, "width": 40, "height": 40})

    def check_collision(self):
        """Check for collisions between the player and obstacles."""
        for obstacle in self.obstacles:
            if (
                self.player_x < obstacle["x"] + obstacle["width"]
                and self.player_x + self.PLAYER_WIDTH > obstacle["x"]
                and self.player_y < obstacle["y"] + obstacle["height"]
                and self.player_y + self.PLAYER_HEIGHT > obstacle["y"]
            ):

                self.game_over = True

    def show_game_over_menu(self):
        """Display the gameinfo-over screen."""
        self.screen.fill(self.BLACK)
        game_over_text = self.large_font.render("gameinfo OVER", True, self.RED)
        score_text = self.font.render(f"Your Score: {self.score}", True, self.WHITE)
        high_score_text = self.font.render(f"High Score: {self.high_score}", True, self.WHITE)
        restart_text = self.font.render("Press R to Restart or Q to Quit", True, self.WHITE)
        quit_text = self.font.render("Press ESC to Quit", True, self.WHITE)

        self.screen.blit(
            game_over_text,
            (self.SCREEN_WIDTH // 2 - game_over_text.get_width() // 2, 150),
        )
        self.screen.blit(
            score_text,
            (self.SCREEN_WIDTH // 2 - score_text.get_width() // 2, 250),
        )
        self.screen.blit(
            high_score_text,
            (self.SCREEN_WIDTH // 2 - high_score_text.get_width() // 2, 300),
        )
        self.screen.blit(
            restart_text,
            (self.SCREEN_WIDTH // 2 - restart_text.get_width() // 2, 350),
        )

        pygame.display.flip()

    def restart_game(self):
        """Reset the gameinfo to start over."""
        self.game_over = False
        self.player_x = self.lanes[self.current_lane]  # Start at the middle lane
        self.obstacles.clear()
        self.bg_y = 0
        self.high_score = max(self.high_score, self.score)
        self.score = 0
        self.obstacle_speed = 5
        self.spawn_delay = 60

    def update_background(self):
        """Create a scrolling background effect."""
        self.bg_y += self.bg_scroll_speed
        if self.bg_y >= self.SCREEN_HEIGHT:
            self.bg_y = 0
        pygame.draw.rect(self.screen, self.GREY, (0, self.bg_y - self.SCREEN_HEIGHT, self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.draw.rect(self.screen, self.GREY, (0, self.bg_y, self.SCREEN_WIDTH, self.SCREEN_HEIGHT))

    def handle_events(self):
        """Handle all player inputs."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        keys = pygame.key.get_pressed()
        if self.game_over:
            if keys[pygame.K_r]:  # Restart gameinfo
                self.restart_game()
            if keys[pygame.K_q]:  # Quit gameinfo
                self.running = False
        else:
            if keys[pygame.K_LEFT] and self.current_lane > 0:
                # Only allow lane change if enough time has passed
                if pygame.time.get_ticks() - self.last_switch_time > self.switch_delay:
                    self.current_lane -= 1
                    self.player_x = self.lanes[self.current_lane]
                    self.last_switch_time = pygame.time.get_ticks()  # Reset timer
            if keys[pygame.K_RIGHT] and self.current_lane < 2:
                # Only allow lane change if enough time has passed
                if pygame.time.get_ticks() - self.last_switch_time > self.switch_delay:
                    self.current_lane += 1
                    self.player_x = self.lanes[self.current_lane]
                    self.last_switch_time = pygame.time.get_ticks()

            # Jumping action
            if keys[pygame.K_SPACE] and not self.is_jumping:
                self.is_jumping = True
                self.jump_sound.play()

    def update_game(self):
        """Update the gameinfo state."""
        if not self.game_over:
            # Generate obstacles and power-ups
            if pygame.time.get_ticks() % self.spawn_delay == 0:
                self.generate_obstacle()
                self.generate_power_up()

            # Move obstacles
            for obstacle in self.obstacles[:]:
                obstacle["y"] += self.obstacle_speed
                if obstacle["y"] > self.SCREEN_HEIGHT:
                    self.obstacles.remove(obstacle)

            # Check for collisions
            self.check_collision()

            # Update score and handle speed boosts
            if not self.is_boosted:
                self.score += 1  # Increase score over time
            if self.is_boosted:
                self.boost_timer -= 1
                if self.boost_timer <= 0:
                    self.is_boosted = False
                    self.obstacle_speed -= 2  # Reset speed back to normal

            if self.is_jumping:
                self.player_y += self.jump_velocity
                self.jump_velocity += self.gravity
                if self.player_y >= self.SCREEN_HEIGHT - self.PLAYER_HEIGHT - 20:
                    self.player_y = self.SCREEN_HEIGHT - self.PLAYER_HEIGHT - 20
                    self.is_jumping = False
                    self.jump_velocity = -15  # Reset jump

        # Draw everything
        self.screen.fill(self.BLACK)
        self.update_background()
        self.draw_lanes()
        self.draw_player()
        self.draw_obstacles()
        self.display_score()

        if self.game_over:
            self.show_game_over_menu()

        pygame.display.flip()

    def run(self):
        """Run the gameinfo loop."""
        while self.running:
            self.handle_events()
            self.update_game()
            self.clock.tick(self.FPS)

        pygame.quit()
        sys.exit()
def run_vision_debug():
    global vision_debug
    ascii_art = pyfiglet.figlet_format("vision debug mode:")
    print(Fore.YELLOW + ascii_art + Style.RESET_ALL)
    if vision_debug:
        vision_debug=False
        ascii_art = pyfiglet.figlet_format("off")
        print(Fore.RED + ascii_art + Style.RESET_ALL)
    else:
        vision_debug=True
        ascii_art = pyfiglet.figlet_format("on")
        print(Fore.GREEN + ascii_art + Style.RESET_ALL)

def subway():
    if __name__ == "__main__":
        game = SubwaySurfersGame()
        game.run()

class UpdateThread(QThread):
    update_info_signal = pyqtSignal(list)  # Signal to update the `info` QListWidget
    update_astar_signal = pyqtSignal(list)  # Signal to update the `a_star` QListWidget

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        global inform, Astar_info
        while self.running:
            sleep(0.5)
            try:
                if inform:
                    self.update_info_signal.emit(inform)
                    inform = []  # Clear inform after processing
                
                if Astar_info:

                    self.update_astar_signal.emit(Astar_info)
                    Astar_info = []  # Clear Astar_info after processing
            except Exception as e:
                print(f"Error in UpdateThread: {e}")

    def stop(self):
        self.running = False

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        try:
            super().__init__()
            ui_file = QFile("../../../../debug app.ui")
            ui_file.open(QFile.OpenModeFlag.ReadOnly)
            loadUi(ui_file, self)
            ui_file.close()

            # Create a task_worker thread for updating the list widget
            self.task_worker = task_worker()

            self.task_worker.update_ui_signal.connect(self.update_ui)  # Connect signal to update_ui

            self.start_thread()

            self.imbord_flappy.clicked.connect(im_bored)
            self.im_a_snake.clicked.connect(snake_game)
            self.imbord.clicked.connect(subway)
            self.avalebility_map.clicked.connect(avalebilty)
            self.wheel_l.clicked.connect(create_graph)
            self.wheel_r.clicked.connect(create_graph2)
            self.MC.clicked.connect(run_mincraft)
            self.TET.clicked.connect(run_tet)
            self.vision_debug_butten.clicked.connect(run_vision_debug)

            # Create a reader thread to monitor the QTextEdit `cmd`
            self.cmd_reader = CmdReader(self.cmd)  # Pass the QTextEdit widget `cmd`
            self.cmd_reader.text_changed_signal.connect(self.handle_text_change)
            self.cmd_reader.start()

            # Start the CmdReader thread
            
            # Create the update thread
            self.update_thread = UpdateThread()
            self.update_thread.update_info_signal.connect(self.update_info_list)
            self.update_thread.update_astar_signal.connect(self.update_astar_list)
            self.update_thread.start()

        except Exception as e:
            print(f"Error in MainWindow init: {e}")

    def start_thread(self):
        self.task_worker.start()  # Start the task_worker thread

    def update_ui(self, items, where_to_update="tasks"):
        global Astar_info
        # This method updates the QListWidget named 'tasks'
        if where_to_update == "tasks":
            for item in items:
                self.tasks.addItem(item)  # Add each item to the QListWidget
        if where_to_update == "info":
            for item in items:
                self.info.addItem(item)

    def handle_text_change(self, text):
        enter_list = []
        self.command = ""
        try:
            if text[-1] == "\n":
                if text.count("\n") == 1:
                    self.command = text[0:-1]
                else:
                    for index, i in enumerate(text):
                        if i == "\n":
                            enter_list.append(index)
                    enter_len = len(enter_list)
                    self.command = text[enter_list[enter_len - 2] + 1 : enter_list[enter_len - 1]]
                derivetive = self.command.split(".")
                if derivetive[0].strip() == "add":
                    derivetive = derivetive[1].split("=")
                    self.update_ui([derivetive[1]], derivetive[0].strip())
                elif self.command == "cls":
                    self.clear_cmd()
                elif derivetive[0].strip() == "run":
                    derivetive = eval(derivetive[1])
                    exec(derivetive)
                else:
                    self.info_label.setText("invalid command")
        except Exception as e:
            print(f"Error in handle_text_change: {e}")

    def clear_cmd(self):
        # Clear the QTextEdit widget (`cmd`)
        self.cmd.clear()

    def update_info_list(self, items):
        for item in items:
            self.info.addItem(item)

    def update_astar_list(self, items):
        for item in items:
            self.a_star.addItem(item)

    def closeEvent(self, event):
        # Stop the threads when the main window is closed
        self.cmd_reader.stop()
        self.cmd_reader.wait()

        self.update_thread.stop()
        self.update_thread.wait()
        
        super().closeEvent(event)

#*******************pyqt_startup*****************
def run_pyqt():
    if __name__ == "__main__":
        app = QtWidgets.QApplication(sys.argv)
        window = MainWindow()
        window.show()
        worker=task_worker()
        worker.run()
        
        sys.exit(app.exec())
#************************************************
def delay(ms):
    initTime = robot.getTime()  # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms:  # If time elapsed (converted into ms) is greater than value
            # passed in
            break
class FlappyBirdGame:
    def __init__(self):
        # Initialize Pygame
        pygame.init()
        
        # Game window dimensions
        self.WIDTH, self.HEIGHT = 400, 600
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Flappy Bird")

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)

        # Game variables
        self.gravity = 0.5
        self.bird_movement = 0
        self.game_active = True
        self.pipe_speed = 3
        self.pipe_width = 50
        self.pipe_gap = 150

        # Bird and pipe properties
        self.bird = pygame.Rect(100, self.HEIGHT // 2 - 20, 30, 30)
        self.pipe_list = []
        self.pipe_height = [200, 300, 400]

        # Font for displaying score
        self.font = pygame.font.SysFont("Arial", 40)

        # Score
        self.score = 0

        # Clock
        self.clock = pygame.time.Clock()

        # Initial pipe
        self.pipe_list.extend(self.create_pipe())

    def draw_bird(self):
        """Draw the bird on the screen."""
        pygame.draw.ellipse(self.screen, self.BLUE, self.bird)

    def create_pipe(self):
        """Create a new pipe."""
        random_pipe_height = random.choice(self.pipe_height)
        bottom_pipe = pygame.Rect(self.WIDTH, random_pipe_height, self.pipe_width, self.HEIGHT - random_pipe_height)
        top_pipe = pygame.Rect(self.WIDTH, 0, self.pipe_width, random_pipe_height - self.pipe_gap)
        return bottom_pipe, top_pipe

    def move_pipes(self, pipes):
        """Move the pipes to the left."""
        for pipe in pipes:
            pipe.centerx -= self.pipe_speed
        return pipes

    def draw_pipes(self, pipes):
        """Draw the pipes on the screen."""
        for pipe in pipes:
            pygame.draw.rect(self.screen, self.GREEN, pipe)

    def check_collision(self, pipes):
        """Check if the bird has collided with any pipe or the ground."""
        for pipe in pipes:
            if self.bird.colliderect(pipe):
                return False
        if self.bird.top <= -50 or self.bird.bottom >= self.HEIGHT:
            return False
        return True

    def display_score(self):
        """Display the current score on the screen."""
        score_surface = self.font.render(f"Score: {self.score}", True, self.BLACK)
        self.screen.blit(score_surface, (self.WIDTH // 2 - 50, 20))

    def reset_game(self):
        """Reset the game state."""
        self.pipe_list.clear()
        self.pipe_list.extend(self.create_pipe())
        self.bird.center = (100, self.HEIGHT // 2)
        self.bird_movement = 0
        self.score = 0
        self.game_active = True

    def run_game(self):
        """Run the game loop."""
        running=True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and self.game_active:
                        self.bird_movement = 0
                        self.bird_movement -= 8
                    if event.key == pygame.K_SPACE and not self.game_active:
                        self.reset_game()

            # Fill background
            self.screen.fill(self.WHITE)

            if self.game_active:
                # Bird movement
                self.bird_movement += self.gravity
                self.bird.centery += self.bird_movement
                self.draw_bird()

                # Move and draw pipes
                self.pipe_list = self.move_pipes(self.pipe_list)
                if self.pipe_list[0].centerx < -self.pipe_width:
                    self.pipe_list.pop(0)
                    self.pipe_list.pop(0)
                    self.pipe_list.extend(self.create_pipe())
                    self.score += 1

                self.draw_pipes(self.pipe_list)

                # Collision check
                self.game_active = self.check_collision(self.pipe_list)

                # Display score
                self.display_score()

            else:
                game_over_surface = self.font.render("Game Over", True, self.BLACK)
                self.screen.blit(game_over_surface, (self.WIDTH // 2 - 100, self.HEIGHT // 2 - 50))
                self.screen.blit(self.font.render(f"Score: {self.score}", True, self.BLACK), (self.WIDTH // 2 - 75, self.HEIGHT // 2 +50))

            # Update display
            pygame.display.update()

            # Frame rate
            self.clock.tick(60)

def get_data2(graph_app):
    global wheel_speed
    while True:
        # Simulate adding random data
        graph_app.add_data(wheel_speed[1])
        time.sleep(0.016)

class DynamicGraphApp2:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Graph")

        # Set canvas size
        self.canvas = tk.Canvas(self.root, width=800, height=400, bg="black")
        self.canvas.pack()

        self.data = []  # List to store data points [(timestamp, value)]
        self.max_data_points = 100  # Maximum points displayed at once

        # Start updating the graph
        self.update_graph()

    def add_data(self, value):
        """Adds new data to the graph."""
        current_time = time.time()
        self.data.append((current_time, value))

    def update_graph(self):
        """Main update loop for the graph."""
        self.canvas.delete("all")  # Clear canvas before redrawing

        # Get current time to remove old data
        current_time = time.time()
        self.data = [(t, v) for t, v in self.data if current_time - t <= 1]

        # Scale the graph size based on data
        if self.data:
            max_value = max(v for _, v in self.data)
            min_value = min(v for _, v in self.data)
        else:
            max_value, min_value = 1, 0

        # Drawing the data points as lines on the canvas
        if len(self.data) > 1:
            x_scale = 800 / self.max_data_points
            y_scale = 300 / (max_value - min_value + 1) if max_value != min_value else 1

            for i in range(1, len(self.data)):
                x1 = (i - 1) * x_scale
                y1 = 350 - (self.data[i - 1][1] - min_value) * y_scale
                x2 = i * x_scale
                y2 = 350 - (self.data[i][1] - min_value) * y_scale

                self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2)

        # Schedule the next update
        self.root.after(50, self.update_graph)
    def run(self):
        self.root.mainloop()
def create_graph2():
    if __name__ == "__main__":
        import random
        import threading

        # Initialize the app
        root = tk.Tk()
        graph_app = DynamicGraphApp2(root)

        # Start the simulated data thread
        data_thread = threading.Thread(target=get_data2, args=(graph_app,))
        data_thread.daemon = True
        data_thread.start()

        # Run the Tkinter main loop
        graph_app.run()

def get_data(graph_app):
    global wheel_speed
    while True:
        # Simulate adding random data
        graph_app.add_data(wheel_speed[0])
        time.sleep(0.016)

class DynamicGraphApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Graph")

        # Set canvas size
        self.canvas = tk.Canvas(self.root, width=800, height=400, bg="black")
        self.canvas.pack()

        self.data = []  # List to store data points [(timestamp, value)]
        self.max_data_points = 100  # Maximum points displayed at once

        # Start updating the graph
        self.update_graph()

    def add_data(self, value):
        """Adds new data to the graph."""
        current_time = time.time()
        self.data.append((current_time, value))

    def update_graph(self):
        """Main update loop for the graph."""
        self.canvas.delete("all")  # Clear canvas before redrawing

        # Get current time to remove old data
        current_time = time.time()
        self.data = [(t, v) for t, v in self.data if current_time - t <= 1]

        # Scale the graph size based on data
        if self.data:
            max_value = max(v for _, v in self.data)
            min_value = min(v for _, v in self.data)
        else:
            max_value, min_value = 1, 0

        # Drawing the data points as lines on the canvas
        if len(self.data) > 1:
            x_scale = 800 / self.max_data_points
            y_scale = 300 / (max_value - min_value + 1) if max_value != min_value else 1

            for i in range(1, len(self.data)):
                x1 = (i - 1) * x_scale
                y1 = 350 - (self.data[i - 1][1] - min_value) * y_scale
                x2 = i * x_scale
                y2 = 350 - (self.data[i][1] - min_value) * y_scale

                self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=2)

        # Schedule the next update
        self.root.after(50, self.update_graph)
    def run(self):
        self.root.mainloop()
def create_graph():
    if __name__ == "__main__":
        import random
        import threading

        # Initialize the app
        root = tk.Tk()
        graph_app = DynamicGraphApp(root)

        # Start the simulated data thread
        data_thread = threading.Thread(target=get_data, args=(graph_app,))
        data_thread.daemon = True
        data_thread.start()

        # Run the Tkinter main loop
        graph_app.run()

def create_grid(canvas, width, height, cell_size, use_x, use_y):
    """Create a grid on the canvas with dynamic scaling and real-time updates."""
    canvas.delete("all")
    
    # Calculate grid dimensions and positioning
    canvas_width = canvas.winfo_width()
    canvas_height = canvas.winfo_height()
    x_offset = (canvas_width - use_x * cell_size) / 2
    y_offset = (canvas_height - use_y * cell_size) / 2

    for row in range(use_x):
        for col in range(use_y):
            # Get current cell data
            map_x = row + width[0]
            map_y = col + height[0]
            cell_data = r2_map[map_x][map_y]
            
            # Calculate cell coordinates
            top_left_x = x_offset + row * cell_size
            top_left_y = y_offset + col * cell_size
            center_x = top_left_x + cell_size / 2
            center_y = top_left_y + cell_size / 2

            # Determine cell color
            color = "green"
            if cell_data["status"] == -1:
                color = "red"
            elif cell_data["status"] == 0:
                color = "yellow"
            elif cell_data["type"] == 2:
                color = "black"
            elif cell_data["type"] == 3:
                color = "brown"
            elif cell_data["type"] == 4:
                color = "gray"
            
            # Priority-based overrides
            if cell_data["status"] == -2:
                color = "purple"
            if cell_data["room"] == 4:
                # color = "white"
                pass
            if map_x == x and map_y == y:
                color = "blue"

            # Draw cell rectangle with borders
            canvas.create_rectangle(
                top_left_x, top_left_y,
                top_left_x + cell_size, top_left_y + cell_size,
                fill=color, outline="#030000", width=1  # Added border
            )

            # Add cell text
            text = "X" if (map_x == x and map_y == y) else f"{map_x}, {map_y}"
            canvas.create_text(center_x, center_y, 
                             text=text, 
                             font=("Arial", max(int(cell_size/6), 5)))

class MapTracker:
    def __init__(self):
        self.last_state = {
            "position": (None, None),
            "grid_bounds": (None, None, None, None),
            "data_hash": None
        }
        
    def has_changes(self):
        """Check for changes in map data or position"""
        current_position = (x, y)
        current_bounds = (x_min, x_max, y_min, y_max)
        
        # Check for position changes
        position_changed = current_position != self.last_state["position"]
        
        # Check for grid boundary changes
        bounds_changed = current_bounds != self.last_state["grid_bounds"]
        
        # Check for map data changes
        current_hash = hash(str(r2_map[x_min:x_max+1][y_min:y_max+1]))
        data_changed = current_hash != self.last_state["data_hash"]
        
        if position_changed or bounds_changed or data_changed:
            self.last_state = {
                "position": current_position,
                "grid_bounds": current_bounds,
                "data_hash": current_hash
            }
            return True
        return False

def avalebilty():
    root = tk.Tk()
    root.title("Live Grid Map")
    
    canvas = tk.Canvas(root, bg="black")
    canvas.pack(fill="both", expand=True)
    
    tracker = MapTracker()
    
    def handle_resize(event=None):
        # Calculate grid dimensions
        grid_width = (x_min, x_max)
        grid_height = (y_min, y_max)
        use_x = grid_width[1] - grid_width[0] + 1
        use_y = grid_height[1] - grid_height[0] + 1
        
        # Calculate cell size
        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()
        cell_size = min(canvas_width/use_x, canvas_height/use_y) if use_x and use_y else 40
        cell_size = max(10, cell_size)
        
        create_grid(canvas, grid_width, grid_height, cell_size, use_x, use_y)
        
        # Schedule next update
        root.after(50, check_for_updates)
    
    def check_for_updates():
        if tracker.has_changes():
            handle_resize()
        else:
            root.after(50, check_for_updates)
    
    # Set up initial bindings and start updates
    canvas.bind("<Configure>", handle_resize)
    handle_resize()  # Initial draw
    root.after(50, check_for_updates)  # Start update loop
    root.mainloop()

class get_input():

    def init(self):
        super().__init__()

        self.initUI()
    def make_window(self):
        global k_angel,base_k,off_or_on
        
        root=Tk()
        root.geometry("400x300+568+332")
        switch_frame = tk.Frame(root)
        switch_frame.pack()

        switch_variable = tk.StringVar(value="off")
        off_button = tk.Radiobutton(switch_frame, text="regular", variable=switch_variable,
                                    indicatoron=False, value="regular", width=10,command=self.regular)

        high_button = tk.Radiobutton(switch_frame, text="add", variable=switch_variable,
                                    indicatoron=False, value="add", width=10,command=self.add)
        off_button.pack(side="left")

        high_button.pack(side="left")
        text_var = tk.StringVar()
        text_var.set("forwads K")
        label = tk.Label(root, 
                 textvariable=text_var, 
                 anchor=tk.CENTER,       
                 bg="black",      
                 height=2,              
                 width=20,              
                 bd=3,                  
                 font=("Arial", 10, "bold"), 
                 cursor="hand2",   
                 fg="White",             
                                 
                 justify=tk.CENTER,    
                 relief=tk.RAISED,     
                        
                )
        label.pack()
        textBox=Text(root, height=2, width=20)
        textBox.pack()
        buttonCommit=Button(root, height=1, width=10, text="Commit", 
                            command=lambda: self.retrieve_input(textBox,off_or_on))
        #command=lambda: retrieve_input() >>> just means do this when i press the button
        buttonCommit.pack()
        text_var.set("forwads Kangel")
        label = tk.Label(root, 
                 textvariable=text_var, 
                 anchor=tk.CENTER,       
                 bg="black",      
                 height=2,              
                 width=20,              
                 bd=3,                  
                 font=("Arial", 10, "bold"), 
                 cursor="hand2",   
                 fg="White",             
                                 
                 justify=tk.CENTER,    
                 relief=tk.RAISED,     
                        
                )
        label.pack()
        textBox2=Text(root, height=2, width=20)
        textBox2.pack()
        buttonCommit2=Button(root, height=1, width=10, text="Commit", 
                            command=lambda: self.retrieve_input2(textBox2,off_or_on))
        #command=lambda: retrieve_input() >>> just means do this when i press the button
        buttonCommit2.pack()
        mainloop()
    def regular(self):
        global off_or_on
        off_or_on=0
    def add(self):
        global off_or_on
        off_or_on=1

    def retrieve_input2(self,textBox,mode):
        global k_angel,base_k
        inputValue=textBox.get("1.0","end-1c")
        if mode==1:
            base_k+=float(inputValue)
        elif mode==0:
            base_k=float(inputValue)
    def retrieve_input(self,textBox,mode):
        global k_angel,base_k
        inputValue=textBox.get("1.0","end-1c")
        if mode==1:
            k_angel+=float(inputValue)
        elif mode==0:
            k_angel=float(inputValue)
class app_start(Frame):

    def __init__(self):
        super().__init__()

        self.initUI()

    
    def initUI(self):
        self.master.title("app starter")
        windo=get_input()

        
        menubar = Menu(self.master)
        self.master.config(menu=menubar)

        
        fileMenu = Menu(menubar)
        fileMenu.add_command(label="avalability", command=avalebilty)
        fileMenu.add_command(label="edit K(s)", command=windo.make_window)
        fileMenu.add_command(label="show wheel graph right", command=create_graph)
        fileMenu.add_command(label="show wheel graph left", command=create_graph2)
        fileMenu.add_command(label="im bored (flappy bird)", command=im_bored)
        
        menubar.add_cascade(label="File", menu=fileMenu)


    def onExit(self):

        self.quit()


class SnakeGame:
    def __init__(self):
        # Initialize pygame
        pygame.init()

        # Screen dimensions
        self.width, self.height = 800, 600

        # Colors
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        self.red = (213, 50, 80)
        self.green = (0, 255, 0)
        self.blue = (50, 153, 213)

        # Snake block size and speed
        self.block_size = 10
        self.speed = 15

        # Initialize the screen
        self.dis = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Snake Game')

        # Clock to control the game speed
        self.clock = pygame.time.Clock()

        # Font styles
        self.font_style = pygame.font.SysFont("bahnschrift", 25)
        self.score_font = pygame.font.SysFont("comicsansms", 35)

        # Game state variables
        self.game_over = False
        self.game_close = False
        self.snake_list = []
        self.length_of_snake = 1

        # Snake position
        self.x1, self.y1 = self.width // 2, self.height // 2
        self.x1_change, self.y1_change = 0, 0

        # Food position
        self.foodx = self.generate_food()[0]
        self.foody = self.generate_food()[1]

    def generate_food(self):
        return (round(random.randrange(0, self.width - self.block_size) / 10.0) * 10.0,
                round(random.randrange(0, self.height - self.block_size) / 10.0) * 10.0)

    def score_display(self):
        value = self.score_font.render(f"Your Score: {self.length_of_snake - 1}", True, self.blue)
        self.dis.blit(value, [10, 10])

    def draw_snake(self):
        for block in self.snake_list:
            pygame.draw.rect(self.dis, self.green, [block[0], block[1], self.block_size, self.block_size])

    def message(self, msg, color):
        mesg = self.font_style.render(msg, True, color)
        self.dis.blit(mesg, [self.width / 6, self.height / 3])

    def game_loop(self):
        while not self.game_over:

            while self.game_close:
                self.dis.fill(self.black)
                self.message("You lost! Press Q-Quit or C-Play Again", self.red)
                self.score_display()
                pygame.display.update()

                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            self.game_over = True
                            self.game_close = False
                        if event.key == pygame.K_c:
                            self.__init__()
                            self.game_loop()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.game_over = True
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT and self.x1_change == 0:
                        self.x1_change = -self.block_size
                        self.y1_change = 0
                    elif event.key == pygame.K_RIGHT and self.x1_change == 0:
                        self.x1_change = self.block_size
                        self.y1_change = 0
                    elif event.key == pygame.K_UP and self.y1_change == 0:
                        self.y1_change = -self.block_size
                        self.x1_change = 0
                    elif event.key == pygame.K_DOWN and self.y1_change == 0:
                        self.y1_change = self.block_size
                        self.x1_change = 0

            if self.x1 >= self.width or self.x1 < 0 or self.y1 >= self.height or self.y1 < 0:
                self.game_close = True

            self.x1 += self.x1_change
            self.y1 += self.y1_change
            self.dis.fill(self.black)
            pygame.draw.rect(self.dis, self.red, [self.foodx, self.foody, self.block_size, self.block_size])

            snake_head = [self.x1, self.y1]
            self.snake_list.append(snake_head)
            if len(self.snake_list) > self.length_of_snake:
                del self.snake_list[0]

            for block in self.snake_list[:-1]:
                if block == snake_head:
                    self.game_close = True

            self.draw_snake()
            self.score_display()

            pygame.display.update()

            if self.x1 == self.foodx and self.y1 == self.foody:
                self.foodx, self.foody = self.generate_food()
                self.length_of_snake += 1

            self.clock.tick(self.speed)

        pygame.quit()
        quit()
def snake_game():
    if __name__ == "__main__":
        game = SnakeGame()
        game.game_loop()
def im_bored():
    flappy_bird_game = FlappyBirdGame()
    flappy_bird_game.run_game()
def app_starter():

    root = Tk()
    root.geometry("400x200+568+332")
    app = app_start()
    root.mainloop()

def run_mincraft():
    # with open("../../../../game_files/Minecraft/Main.py") as f:
    #     content = f.read()

    # exec(content)
    import subprocess
    subprocess.run(["python",'../../../../game_files/Minecraft/Main.py'])
def run_tet():
    import subprocess
    subprocess.run(["python",'../../../../game_files/tetres/Main.py'])

#***********************A star atempt*******************
def deg2mapdin(deg):
    lst_din=[0,0]


    match int(deg):
        case 90:
            lst_din=[1,0]
        case 270:
            lst_din=[-1,0]
        case 180:
            lst_din=[0,-1]
        case 360:
            lst_din=[0,1]
        case 45:
            lst_din=[1,1]
        case 135:
            lst_din=[1,-1] 
        case 225:
            lst_din=[-1,-1]
        case 315:
            lst_din=[-1,1]
    return lst_din


def print_banner_red(text): print(Fore.RED + figlet_format(text, font="banner"))
def print_big_green(text): print(Fore.GREEN + figlet_format(text, font="big"))
def print_block_yellow(text): print(Fore.YELLOW + figlet_format(text, font="block"))
def print_bubble_blue(text): print(Fore.BLUE + figlet_format(text, font="bubble"))
def print_digital_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="digital"))
def print_ivrit_cyan(text): print(Fore.CYAN + figlet_format(text, font="ivrit"))
def print_letters_white(text): print(Fore.WHITE + figlet_format(text, font="letters"))
def print_broadway_red(text): print(Fore.RED + figlet_format(text, font="broadway"))
def print_crazy_green(text): print(Fore.GREEN + figlet_format(text, font="crazy"))
def print_doom_yellow(text): print(Fore.YELLOW + figlet_format(text, font="doom"))
def print_drpepper_blue(text): print(Fore.BLUE + figlet_format(text, font="drpepper"))
def print_efifont_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="eftifont"))
def print_ghost_cyan(text): print(Fore.CYAN + figlet_format(text, font="ghost"))
def print_goofy_white(text): print(Fore.WHITE + figlet_format(text, font="goofy"))
def print_isometric4_red(text): print(Fore.RED + figlet_format(text, font="isometric4"))
def print_jazmine_green(text): print(Fore.GREEN + figlet_format(text, font="jazmine"))
def print_katakana_yellow(text): print(Fore.YELLOW + figlet_format(text, font="katakana"))
def print_larry3d_blue(text): print(Fore.BLUE + figlet_format(text, font="larry3d"))
def print_lean_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="lean"))
def print_lockergnome_cyan(text): print(Fore.CYAN + figlet_format(text, font="lockergnome"))
def print_mini_white(text): print(Fore.WHITE + figlet_format(text, font="mini"))
def print_modular_red(text): print(Fore.RED + figlet_format(text, font="modular"))
def print_morse_green(text): print(Fore.GREEN + figlet_format(text, font="morse"))
def print_moscow_yellow(text): print(Fore.YELLOW + figlet_format(text, font="moscow"))
def print_ncursive_blue(text): print(Fore.BLUE + figlet_format(text, font="nancyj"))
def print_ogre_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="ogre"))
def print_peaks_cyan(text): print(Fore.CYAN + figlet_format(text, font="peaks"))
def print_poison_white(text): print(Fore.WHITE + figlet_format(text, font="poison"))
def print_puffy_red(text): print(Fore.RED + figlet_format(text, font="puffy"))
def print_rectangles_green(text): print(Fore.GREEN + figlet_format(text, font="rectangles"))
def print_relievo_yellow(text): print(Fore.YELLOW + figlet_format(text, font="relief"))
def print_script_blue(text): print(Fore.BLUE + figlet_format(text, font="script"))
def print_serifcap_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="serifcap"))
def print_shadow_cyan(text): print(Fore.CYAN + figlet_format(text, font="shadow"))
def print_slscript_white(text): print(Fore.WHITE + figlet_format(text, font="slscript"))
def print_small_red(text): print(Fore.RED + figlet_format(text, font="small"))
def print_smtengwar_green(text): print(Fore.GREEN + figlet_format(text, font="smtengwar"))
def print_speed_yellow(text): print(Fore.YELLOW + figlet_format(text, font="speed"))
def print_stacey_blue(text): print(Fore.BLUE + figlet_format(text, font="stacey"))
def print_standard_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="standard"))
def print_starwars_cyan(text): print(Fore.CYAN + figlet_format(text, font="starwars"))
def print_stop_white(text): print(Fore.WHITE + figlet_format(text, font="stop"))
def print_term_red(text): print(Fore.RED + figlet_format(text, font="term"))
def print_trek_green(text): print(Fore.GREEN + figlet_format(text, font="trek"))
def print_twopoint_yellow(text): print(Fore.YELLOW + figlet_format(text, font="twopoint"))
def print_usaflag_blue(text): print(Fore.BLUE + figlet_format(text, font="usaflag"))
def print_venus_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="utopiab"))
def print_wow_cyan(text): print(Fore.CYAN + figlet_format(text, font="wow"))
def print_xhelvi_white(text): print(Fore.WHITE + figlet_format(text, font="xhelvi"))
def print_yie_ar_red(text): print(Fore.RED + figlet_format(text, font="yie-ar"))
def print_alphabet_red(text): print(Fore.RED + figlet_format(text, font="alphabet"))
def print_avatar_green(text): print(Fore.GREEN + figlet_format(text, font="avatar"))
def print_b1ff_style_yellow(text): print(Fore.YELLOW + figlet_format(text, font="b1ff"))
def print_barbwire_outline_blue(text): print(Fore.BLUE + figlet_format(text, font="barbwire"))
def print_barricade_block_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="barricade"))
def print_bazbil_wide_cyan(text): print(Fore.CYAN + figlet_format(text, font="baz_bil"))
def print_beer_style_white(text): print(Fore.WHITE + figlet_format(text, font="beer_pwk"))
def print_benjamin_classic_red(text): print(Fore.RED + figlet_format(text, font="benjamin"))
def print_bigchief_chunky_green(text): print(Fore.GREEN + figlet_format(text, font="bigchief"))
def print_bloody_drip_yellow(text): print(Fore.YELLOW + figlet_format(text, font="bloody"))
def print_bolger_smooth_blue(text): print(Fore.BLUE + figlet_format(text, font="bolger"))
def print_brainfuck_hacker_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="brainfuck"))
def print_bright_crisp_cyan(text): print(Fore.CYAN + figlet_format(text, font="bright"))
def print_bulbhead_quirky_white(text): print(Fore.WHITE + figlet_format(text, font="bulbhead"))
def print_calligraphy_style_red(text): print(Fore.RED + figlet_format(text, font="caligraphy"))
def print_cardinal_widecaps_green(text): print(Fore.GREEN + figlet_format(text, font="cardinal"))
def print_chiseled_sharp_yellow(text): print(Fore.YELLOW + figlet_format(text, font="chiseled"))
def print_clean6x10_blue(text): print(Fore.BLUE + figlet_format(text, font="clb6x10"))
def print_clean8x10_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="clb8x10"))
def print_coins_dotty_cyan(text): print(Fore.CYAN + figlet_format(text, font="coins"))
def print_computer_terminal_white(text): print(Fore.WHITE + figlet_format(text, font="computer"))
def print_contessa_curvy_red(text): print(Fore.RED + figlet_format(text, font="contessa"))
def print_courier_classic_green(text): print(Fore.GREEN + figlet_format(text, font="courier"))
def print_cricket_bouncy_yellow(text): print(Fore.YELLOW + figlet_format(text, font="cricket"))
def print_cursive_loopy_blue(text): print(Fore.BLUE + figlet_format(text, font="cursive"))
def print_dancing_handwritten_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="dancingfont"))
def print_delta_cornerbox_cyan(text): print(Fore.CYAN + figlet_format(text, font="deltacorner"))
def print_diablo_demonic_white(text): print(Fore.WHITE + figlet_format(text, font="diablo"))
def print_dotty_pixelated_red(text): print(Fore.RED + figlet_format(text, font="dotty"))
def print_eco_minimal_green(text): print(Fore.GREEN + figlet_format(text, font="eco"))
def print_eldorado_western_yellow(text): print(Fore.YELLOW + figlet_format(text, font="eldorado"))
def print_epic_huge_blue(text): print(Fore.BLUE + figlet_format(text, font="epic"))
def print_fangsong_serif_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="fangsong"))
def print_fire_font_hot_cyan(text): print(Fore.CYAN + figlet_format(text, font="fire_font-k"))
def print_fluffy_soft_white(text): print(Fore.WHITE + figlet_format(text, font="fluffy"))
def print_fraktur_gothic_red(text): print(Fore.RED + figlet_format(text, font="fraktur"))
def print_fun_face_playful_green(text): print(Fore.GREEN + figlet_format(text, font="fun_face"))
def print_funfaces_quirky_yellow(text): print(Fore.YELLOW + figlet_format(text, font="funfaces"))
def print_fuzzy_floofy_blue(text): print(Fore.BLUE + figlet_format(text, font="fuzzy"))
def print_graffiti_street_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="graffiti"))
def print_grand_pr_fancy_cyan(text): print(Fore.CYAN + figlet_format(text, font="grand_pr"))
def print_gravity_slanted_white(text): print(Fore.WHITE + figlet_format(text, font="gravity"))
def print_greenbeans_funky_red(text): print(Fore.RED + figlet_format(text, font="green_beans"))
def print_guppy_fishy_green(text): print(Fore.GREEN + figlet_format(text, font="guppy"))
def print_hades_ancient_yellow(text): print(Fore.YELLOW + figlet_format(text, font="hades"))
def print_helv_neat_blue(text): print(Fore.BLUE + figlet_format(text, font="helv"))
def print_hollywood_glam_magenta(text): print(Fore.MAGENTA + figlet_format(text, font="hollywood"))
def print_houseof_cards_cyan(text): print(Fore.CYAN + figlet_format(text, font="house_of"))
def print_invita_script_white(text): print(Fore.WHITE + figlet_format(text, font="invita"))
def print_italic_smooth_red(text): print(Fore.RED + figlet_format(text, font="italic"))


def print_red(statement):
    print(f"{Fore.RED}{statement}{Style.RESET_ALL}")

# Function to print in green
def print_green(statement):
    print(f"{Fore.GREEN}{statement}{Style.RESET_ALL}")

# Function to print in blue
def print_blue(statement):
    print(f"{Fore.BLUE}{statement}{Style.RESET_ALL}")

# Function to print in yellow
def print_yellow(statement):
    print(f"{Fore.YELLOW}{statement}{Style.RESET_ALL}")

# Function to print in magenta
def print_magenta(statement):
    print(f"{Fore.MAGENTA}{statement}{Style.RESET_ALL}")

# Function to print in cyan
def print_cyan(statement):
    print(f"{Fore.CYAN}{statement}{Style.RESET_ALL}")

# Function to print in white
def print_white(statement):
    print(f"{Fore.WHITE}{statement}{Style.RESET_ALL}")

# Function to print in light red
def print_light_red(statement):
    print(f"{Fore.LIGHTRED_EX}{statement}{Style.RESET_ALL}")

# Function to print in light green
def print_light_green(statement):
    print(f"{Fore.LIGHTGREEN_EX}{statement}{Style.RESET_ALL}")

# Function to print in light blue
def print_light_blue(statement):
    print(f"{Fore.LIGHTBLUE_EX}{statement}{Style.RESET_ALL}")

def blindSearch():
    global x, y
    # Get tile ways
    time,_=gameinfo()
    path_home,time_to_exit_home,added_costs_home=A_star(True,time)
    time_to_home=convert_cost_to_time(added_costs_home)
    time_to_home=time_to_home/17+18
    if time<=time_to_home:
        return False
    lidarProMax()
    available_ways = []


    primary_angles = ["180", "360", "270", "90"]  
    sub_primary_angles = ["225", "135", "315", "45"]
    index_list = primary_angles + sub_primary_angles

    lst_tileways = tileways(x, y)
    rounded_angle = abs_yaw()
    max_difference = 360
    min_point = 0

    for i in [-1, 0, 1]:  # x
        for j in [-1, 0, 1]:  # y
            if i == 0 and j == 0:
                continue
            dirdin = mapdin2dirdin(i, j)
            tilewaysLST = tileways(x + i, y + j)
            tilewaysLST_check = tileways(x, y)
            tilewaysdin = tilewaysdin_alternative(i, j)
            if tilewaysLST[tilewaysdin] and tilewaysLST_check[dirdin]:
                available_ways.append(index_list[dirdin])

    def prioritize_ways(ways, priority_list):
        return [way for way in priority_list if way in ways]


    primary_available = prioritize_ways(available_ways, primary_angles)
    sub_primary_available = prioritize_ways(available_ways, sub_primary_angles)

    prioritized_ways = primary_available + sub_primary_available

    for i in prioritized_ways:
        error = abs(int(i) - int(rounded_angle))
        if error > 180:
            error = 360 - error
        if error < max_difference:
            max_difference = error
            min_point = i

    map_periodizer = deg2mapdin(min_point)

    if (
        not (r2_map[x + map_periodizer[0]][y + map_periodizer[1]]["type"] in [2, -2])
    ) and r2_map[x + map_periodizer[0]][y + map_periodizer[1]]["status"] == 0:
        logging.info(f"We want to go to: {x + map_periodizer[0], y + map_periodizer[1]}")
        forward(getLocation((x + map_periodizer[0], y + map_periodizer[1])))
        setLocation()
        return True
    else:
        if len(prioritized_ways) > 1:
            prioritized_ways.remove(min_point)
            for i in prioritized_ways:
                error = abs(int(i) - int(rounded_angle))
                if error > 180:
                    error = 360 - error
                if error < max_difference:
                    max_difference = error
                    min_point = i
            map_periodizer = deg2mapdin(min_point)
        if (
            not (r2_map[x + map_periodizer[0]][y + map_periodizer[1]]["type"] in [2, -2])
        ) and r2_map[x + map_periodizer[0]][y + map_periodizer[1]]["status"] == 0:
            logging.info(f"We want to go to: {x + map_periodizer[0], y + map_periodizer[1]}")
            forward(getLocation((x + map_periodizer[0], y + map_periodizer[1])))
            setLocation()
            return True
        else:
            logging.warning("We are out of blind search")
            return False


def lop_on_target(target):
    global LOPonTraget
    we_had_lop_before=False
    for i in LOPonTraget:
        if i[0]==target:
            r2_map[i[0]][i[1]]["status"]==-2
            
            we_had_lop_before=True
            break
    if we_had_lop_before==False:
        LOPonTraget.append(target)

    
def reset_avoid():
    global avoid_type,avoid_status
    

    avoid_status=[-1,-2,-3]
    avoid_type=[-1,-2,2]
def path_clearance():
    global avoid_type,avoid_status,r2_map

    # avoid_status=[-1]
    # avoid_type=[-1,2]
    for i in range(x_min, x_max + 1):
            for j in range(y_min, y_max + 1):
                if r2_map[i][j]["status"] == -2:
                    r2_map[i][j]["status"] = 1
                    # r2_map[i][j]["type"] = 1
                    # r2_map[i][j]["walls"] = np.zeros((5, 5), dtype='int32')
def print_colored_list(lst):
    
    print("[", end="")
    print(", ".join(lst), end="")
    print("]")
def print_Astar_map(a_star_map,final_pos):
    print(a_star_map[x_min:x_max+1,y_min:y_max+1])
    return
    printablelist=[]
    printList=[]
    printList.append(f"{Fore.YELLOW}X:   {Style.RESET_ALL}")
    runable_x=[]
    runable_y=[]
    for i in range(x_min,x_max+1):
        runable_x.append(i)
    for i in range(y_min,y_max+1):
        runable_y.append(i)
    runable_y.reverse()
    runable_x.reverse()


    for j in range(x_min,x_max+1):
        printList.append(f"{Fore.YELLOW}{j}{Style.RESET_ALL}")
    print_colored_list(printList)




    for j in range(y_min,y_max+1):
        printList=[]
        printList.append(f"{Fore.YELLOW}Y: {j}{Style.RESET_ALL}")


        for i in range(x_min,x_max+1):
            if i ==x and j ==y:
                printList.append(f"{Fore.GREEN}'X{Style.RESET_ALL}")
            elif i==final_pos[0] and j==final_pos[1]:
                printList.append(f"{Fore.GREEN}'F{Style.RESET_ALL}")
            elif int(a_star_map[i][j] )==0:
                printList.append(f"{Fore.BLUE}00{Style.RESET_ALL}")
            elif int(a_star_map[i][j] )<10:
                printList.append(f"{Fore.RED}_{int(a_star_map[i][j] )}{Style.RESET_ALL}")

            else:
                printList.append(f"{Fore.RED}{int(a_star_map[i][j] )}{Style.RESET_ALL}")
        printablelist.append(printList)

    for i in printablelist:
        print_colored_list(i)
def check_for_targets():
    global avoid_type,avoid_status,r2_map

    # avoid_status=[-1]
    # avoid_type=[-1,2]
    for i in range(x_min, x_max + 1):
            for j in range(y_min, y_max + 1):
                if r2_map[i][j]["status"] == 0:
                    return False
    return True



def  mapdin2dirdin(i,j):
    if [i,j]==[-1,-1]:
        return 4
    if [i,j]==[1,1]:
        return 7
    if [i,j]==[-1,1]:
        return 6
    if [i,j]==[1,-1]:
        return 5
    if [i,j]==[0,-1]:
        return 0
    if [i,j]==[-1,0]:
        return 2
    if [i,j]==[0,1]:
        return 1
    if [i,j]==[1,0]:
        return 3
def tilewaysdin_alternative(i,j):
    if [i,j]==[-1,-1]:
        return 7
    if [i,j]==[-1,1]:
        return 5
    if [i,j]==[1,-1]:
        return 6
    if [i,j]==[1,1]:
        return 4
    if [i,j]==[0,-1]:
        return 1
    if [i,j]==[-1,0]:
        return 3
    if [i,j]==[0,1]:
        return 0
    if [i,j]==[1,0]:
        return 2


import time


def calculateCosts(a_star_to_home=False):
    global avoid_type,avoid_status

    # time_start = time.time()
    astar_flag = True
    astar_flag_count = 0
    a_star_map = np.zeros((mapping_size, mapping_size), dtype=np.int16)
    final_pos = [0, 0]

    # Initialize starting point
    a_star_map[x][y] = 1

    directions = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),          (0, 1),
        (1, -1), (1, 0), (1, 1)
    ]

    index_list = ["180", "360", "270", "90", "225", "135", "315", "45"]

    for iteration in range(400):
        if not astar_flag:
            if astar_flag_count > 150:
                # time_elapsed = (time.time() - time_start) * 1000
                # print_yellow(f"The calculate costs algo took {time_elapsed:.2f} ms")
                return [0, 0], a_star_map
            astar_flag_count += 1

        astar_flag = False

        # Iterate over map bounds
        for x_astar in range(x_min, x_max + 1):
            for y_astar in range(y_min, y_max + 1):
                if a_star_map[x_astar, y_astar] != iteration + 1:
                    continue

                # Check completion conditions
                if (not a_star_to_home and r2_map[x_astar][y_astar]["status"] == 0) or \
                   (a_star_to_home and [x_astar ,y_astar] == [100, 100]):
                    final_pos = [x_astar, y_astar]
                    # print_Astar_map(a_star_map=a_star_map, final_pos=final_pos)
                    # time_elapsed = (time.time() - time_start) * 1000
                    # print_yellow(f"The calculate costs algo took {time_elapsed:.2f} ms")
                    return final_pos, a_star_map

                # Evaluate potential moves
                for (i, j), direction in zip(directions, index_list):
                    next_x, next_y = x_astar + i, y_astar + j

                    # Boundary check for the next position
                    if not (0 <= next_x < mapping_size and 0 <= next_y < mapping_size):
                        continue

                    dirdin = mapdin2dirdin(i, j)
                    tilewaysLST = tileways(next_x, next_y)
                    tilewaysLST_check = tileways(x_astar, y_astar)
                    tilewaysdin = tilewaysdin_alternative(i, j)

                    if not (tilewaysLST[tilewaysdin] or tilewaysLST_check[dirdin]):
                        continue

                    if a_star_map[next_x, next_y] == 0:
                        tile_type = r2_map[next_x][next_y]["type"]
                        cost = iteration + 2 + (3 if tile_type == 3 else 0)

                        if tile_type == 2:
                            cost = 0

                        if (not(r2_map[next_x][next_y]["status"] in avoid_status) and
                            tile_type not in avoid_type):
                            if [next_x, next_y] in multyply_cost:
                                cost=100
                            a_star_map[next_x, next_y] = cost
                            astar_flag = True

    return final_pos, a_star_map

giridam_targets=[]
giridam_spot=[]
def giridam_spoter(target):
    global giridam_spot,r2_map
    if giridam_spot.count(target)>7:
        info(f"removed couse we got stck in it too much {target}")
        r2_map[target[0]][target[1]]["status"]=-2
        r2_map[target[0]][target[1]]['type']=1
        return
    else:
        giridam_spot.append(target)
        info(f"unavalble spot noted for the {giridam_spot.count(target)}th time: {target}")

        return
def giridm_on_target(target):
    global giridam_targets,r2_map
    if giridam_targets.count(target)>3:
        info(f"removed couse we got stck on the way to it {target}")
        
        r2_map[target[0]][target[1]]["status"]=-2
        r2_map[target[0]][target[1]]['type']=1
        return
    else:
        giridam_targets.append(target)
        info(f"unavalble target noted for the {giridam_targets.count(target)} time : {target}")

        return
def get_kiasha_ways(final_pos):

    available_ways = []
    index_list = ["180", "360", "270", "90", "225", "135", "315", "45"]

    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if i == 0 and j == 0:
                continue

            dirdin = mapdin2dirdin(i, j)
            tilewaysLST = tileways(final_pos[0] + i, final_pos[1] + j)
            tilewaysLST_check = tileways(final_pos[0], final_pos[1])
            tilewaysdin = tilewaysdin_alternative(i, j)

            if tilewaysLST[tilewaysdin] and tilewaysLST_check[dirdin]:
                din=deg2mapdin(index_list[dirdin])
                if r2_map[final_pos[0]+din[0]][final_pos[1]+din[1]]["type"]!=2 and r2_map[final_pos[0]+din[0]][final_pos[1]+din[1]]["status"] not in [-1,-2]:
                    available_ways.append(index_list[dirdin])

    return available_ways



def get_blind_ways(possible_ways):
    #possible_ways = ["270", "180", "360"]
    lidarProMax()
    # print("salamgoli:",possible_ways )
    lst_mohem = [(180, 0, -1), (135, 1, -1), (90, 1, 0), (45, 1, 1),(360, 0, 1), (315, -1, 1), (270, -1, 0), (225, -1, -1)]

    lst_zero_memory = []
    for item in possible_ways:
        for item2, dx, dy in lst_mohem:
            # d_list=deg2mapdin(int(item2))
            # dx,dy=d_list[0],d_list[1]
            if int(item) == item2:
                info(f"debug2:{item}")
                info(f"type:{type(item)}")

                if r2_map[x + dx][y + dy]["status"] == 0:
                    if r2_map[x + dx][y + dy]["type"] != 2:

                        info(f"debug3:{item2} ")
                        lst_zero_memory.append(item2)
                    else:
                         info("moshkel 1")
                else:
                    if item == "270":
                      info(f'ghand: x= {x + dx} y= {y + dy} res= {r2_map[x + dx][y + dy]["status"]}')
                      info("moshkel 2")
    return lst_zero_memory
    
def best_target(lst_possible):
    mini = ":1000"
    current_angle = abs_yaw()
    lst_delta_dirs = []
    lst_mini = []
    for target in lst_possible: 
        delta_dir = target - current_angle
        if delta_dir >= 180:
            delta_dir = delta_dir - 360
        elif delta_dir <= -180:
            delta_dir = delta_dir + 360
            print(delta_dir)
        lst_mini.append(str(target)+":"+str(abs(delta_dir)))
    for mini2 in lst_mini:
        if abs(int(mini2.split(":")[1])) < abs(int(mini.split(":")[1])):
            mini = mini2
    print("mini",mini)
    return mini


def navigation():
    lst_mohem = [(180, 0, -1), (135, 1, -1), (90, 1, 0), (45, 1, 1),(360, 0, 1), (315, -1, 1), (270, -1, 0), (225, -1, -1)]
    time_to_exit=False
    time,_=gameinfo()
    path_home,time_to_exit_home,added_costs_home=A_star(True,time)  
    time_to_home=convert_cost_to_time(added_costs_home)
    time_to_home=time_to_home/17+18

    if time<=time_to_home:
        return False
    lst_zero_memory = get_blind_ways(get_kiasha_ways((x,y)))

    if len(lst_zero_memory) == 0:
        return False

    else:

        b_target = best_target(lst_zero_memory)
        for dirr, dx, dy in lst_mohem:
            if dirr == int(b_target.split(":")[0]):
                x_t = x + dx
                y_t = y + dy
                return forward(getLocation((x_t, y_t)))

def A_star(a_star_to_home=False,time_game=0):
    
    go(0, 0)
    global noPathLOP_count, r2_map, x, y

    if a_star_to_home and (x, y) == (100, 100):
        return [100, 100], 0, 0

    time_to_exit = False
    added_costs = 0
    final_pos, a_star_map = calculateCosts(a_star_to_home)


    if final_pos == [0, 0]:
        path_clearance()
        go(0.5,0.5)

        final_pos,a_star_map=calculateCosts(a_star_to_home)

        if final_pos==[0,0] :
            final_pos,a_star_map=calculateCosts()
            if final_pos==[0,0]:
                go(0.5,0.5)




                logging.info("no more targets going home")
                print_green("no more targets going home")
                if x==100 and y==100:
                    print_red("we are quitting")
                    exitProcess() 
                r2_map[100][100]["status"]=0
                
                final_pos,a_star_map=calculateCosts()
                print(f"final_pos after finalpos {final_pos}")

                time_to_exit=True
        
    else:
        r2_map[100][100]["status"]=1

    path , added_costs = build_path(final_pos, a_star_map, added_costs, time_game, a_star_to_home)
    
    if path is None:
        return -1, 0, 0
    if not a_star_to_home:
        Ainfo(f"target: {final_pos}")
        Ainfo(f"starting point: {x} , {y}")
        Ainfo(f"path: {path}")

    logging.info(f"The final path is {path}")
    return path, time_to_exit, added_costs






def handle_no_path_to_target(a_star_to_home, time, final_pos):
    global noPathLOP_count,r2_map

    go(0, 0)

    if a_star_to_home and time > 70:
        return -1, 0, 0

    if noPathLOP_count > 2:

        if final_pos != [100, 100]:
            
            print_red(f"Removing the target: {final_pos}")
            r2_map[final_pos[0]][final_pos[1]]["status"] = 1
        return -1, 0, 0

    noPathLOP_count += 1
    logging.info("Resetting path" if noPathLOP_count > 1 else "No path loop")

    if noPathLOP_count > 1:
        path_clearance()
        return -1, 0, 0

    lackofprog()
    lop_on_target(final_pos)
    return -1, 0, 0


def build_path(final_pos, a_star_map, added_costs, time, a_star_to_home):
    path = [final_pos]
    target = final_pos
    a_star_map[final_pos[0]][final_pos[1]] = 0
    while_counter=0
    added_costs=0
    while True:
        if while_counter>10:
            break
        while_counter+=1
        available_ways = get_available_ways(final_pos)
        min_cost_pos, min_cost = get_min_cost_position(final_pos, available_ways, a_star_map)
        
        if not min_cost_pos or final_pos==[] or final_pos==[0,0]:
            return handle_no_path_to_target(a_star_to_home, time, target),0

        try:
            added_costs += min_cost
            a_star_map[min_cost_pos[0]][min_cost_pos[1]] = 0
            path.append(min_cost_pos)
            while_counter=0
            final_pos = min_cost_pos
        except Exception as e:
            logging.error(f"Error: {e}")
            return -1,0

        if (final_pos[0], final_pos[1]) == (x, y):
            path.remove([x, y])
            return path,added_costs


def get_available_ways(final_pos):

    available_ways = []
    index_list = ["180", "360", "270", "90", "225", "135", "315", "45"]

    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if i == 0 and j == 0:
                continue

            dirdin = mapdin2dirdin(i, j)
            tilewaysLST = tileways(final_pos[0] + i, final_pos[1] + j)
            tilewaysLST_check = tileways(final_pos[0], final_pos[1])
            tilewaysdin = tilewaysdin_alternative(i, j)

            if tilewaysLST[tilewaysdin] or tilewaysLST_check[dirdin]:
                available_ways.append(index_list[dirdin])

    return available_ways


def get_min_cost_position(final_pos, available_ways, a_star_map):

    min_cost_pos = None
    min_cost = float('inf')

    for direction in available_ways:
        map_periodizer = deg2mapdin(direction)
        x_pos, y_pos = final_pos[0] + map_periodizer[0], final_pos[1] + map_periodizer[1]
        current_cost = a_star_map[x_pos][y_pos]

        if 0 < current_cost < min_cost:
            min_cost_pos = [x_pos, y_pos]
            min_cost = current_cost
    
    return min_cost_pos, min_cost



                    
def follow_path(path,time_to_exit,move=True):
    global noPathLOP_count
    logging.info(f"the path is: {path}")
    
    if move:
        for i in reversed(path):
            noPathLOP_count=0
            reset_avoid()
            if i==[x,y]:
                continue
            time,_=gameinfo()
            path_home,time_to_exit_home,added_costs_home=A_star(True,time)
            time_to_home=convert_cost_to_time(added_costs_home)
            time_to_home=time_to_home/17+18
            if  time<=2:
                Ainfo("path failed time low")
                print_red("time too low exiting right now")
                exitProcess()
                return
            # print(f"time: {time} time to home {time_to_home}")
            logging.info(f"we are following the path on: {i}")
            if time<=time_to_home:
                Ainfo("path failed time low")

                return
            if type(i)==int:
                Ainfo("path failed")

                return
            return_forward= forward(getLocation(i))
            if return_forward==-1:
                Ainfo("path failed giridaaaaaam")

                giridm_on_target(path[0])
                return
            if return_forward==0:
                Ainfo("path failed black hole")

                return

        
    Ainfo("path successful")

def nav():
    global noPathLOP_count
    lidarProMax()
    check_targets()
    time_to_exit=False
    time,_=gameinfo()
    path_home,time_to_exit_home,added_costs_home=A_star(True,time)  
    time_to_home=convert_cost_to_time(added_costs_home)
    time_to_home=time_to_home/17+18

    if time<=time_to_home:
        logging.info("time is low going back home")
        print_green("time is low going back home")
        if path_home ==-1:
            path_home,time_to_exit_home,added_costs_home=A_star(True,time)
            
        for i in reversed(path_home):
            reset_avoid()
            noPathLOP_count=0
            time,_=gameinfo()
            
            if time<=2:
                print_red("time too low")
                exitProcess()
            if i==[x,y]:
                continue
            if type(i)==int:
                return
            if  forward(getLocation(i)) in [-1,0]:
                path_home,time_to_exit_home,added_costs=A_star(True,time)
                if path_home==-1:
                    path_home,time_to_exit_home,added_costs_home=A_star(True,time)

        exitProcess()
        
    if  not blindSearch() :
        path,time_to_exit,added_costs=A_star(time_game=time)

        if path==-1:
            return
        follow_path(path,time_to_exit)
def checkSurroundings(i, j):
    """Check if a room at position (i, j) should turn into 4."""
    has_four = False  
    room0count=0
    for p in [-1, 0, 1]:
        for q in [-1, 0, 1]:  
           
            if p == 0 and q == 0:
                continue

            x, y = i + p, j + q
            if x_min <= x < x_max+1 and y_min <= y < y_max+1:
                room_value = r2_map[x+p][y+q]["room"]

                if room_value == 4:
                    has_four = True
                if not (room_value in [4,0]):
                    return False
                if room_value==0:
                    room0count+=1
                if room0count>2:
                    return False

    return has_four


def checkroom0():

    """Iteratively update rooms until no more changes occur."""
    global r2_map
    changed_rooms = []
    exit_next=False
    
    while True:
        to_update = []
        for i in range(x_min, x_max+1):  # x
            for j in range(y_min, y_max+1):  # y
                if r2_map[i][j]["room"] == 0 :
                    if checkSurroundings(i, j):
                        r2_map[i][j]["room"] = 4
                        to_update.append((i, j))

        if (not to_update) and exit_next:
            return 
        if not to_update:  
            logging.info(f"Rooms that have been changed to 4: {changed_rooms}")
            exit_next=True
def detectTile(green,blue,red):
    tilenum=-1
    if red > 180 and red < 230 and blue > 80 and blue < 120 and green > 150 and green < 210 and blue < green < red:
        # print("swap")
        tileNum = 3
        return tileNum

    elif red > 80 and red < 140 and blue > 80 and blue < 140 and green > 80 and green < 140 and red  < blue< green:
        # print("check point")
        tileNum = 4
        return tileNum

    elif red > 200 and red < 280 and blue > 55 and blue < 90 and green > 55 and green < 90 and green == blue:
        # print("red")
        tileNum = 8
        return tileNum
    elif red > 110 and red < 170 and blue > 200 and blue < 250 and green > 20 and green < 80 and green < red < blue:
        # print("purple")
        tileNum = 7
        return tileNum
    elif red > 100 and red < 80 and blue > 200 and blue < 280 and green > 100 and green < 80 and green == red:
        # print("blue")
        tileNum = 6
        return tileNum
    elif 20 < red < 100 and blue > 25 and blue < 45 and green > 220 and green < 260 and red == blue:
        # print("green")
        tileNum = 9
        return tileNum
    elif red > 200 and red < 280 and green > 200 and green < 280 and blue > 100 and blue < 90 and blue < green < red:
        # print("orange")
        tileNum = 10
        return tileNum
    elif red > 200 and red < 280 and green > 220 and green < 300 and blue > 100 and blue < 90 and blue < green == red:
        # print("yellow")
        tileNum = 11
        return tileNum
    return tilenum
def refactor_path(path:list):
    # if 
    
    
    if type(path) !=list:
        return -1
    # print(path)

    output_path=[]
    for index,i in enumerate(path):
        last_path=path[index-1]
        if r2_map[i[0]][i[1]]["type"]>5 and( int(i[0])%2==1 and int(i[1])%2==1) and (r2_map[last_path[0]][last_path[1]]["type"]!=r2_map[i[0]][i[1]]["type"] or index==0):
            for n in [-1,0,1]:
                for m in [-1,0,1]:
                    if (i[0]+n)%2==0 and (int(i[1])+m)%2==0 and (r2_map[(int(i[0])+n)][(int(i[1])+m)]["type"]==r2_map[i[0]][i[1]]["type"]):
                        output_path.append([int(i[0])+n,int(i[1])+m])
                        output_path.append(i)


        else:
            output_path.append(i)
    return output_path
"""
tile types:
0: not visited
1: normal tile
2: trap (hole)
3: swamp
4: checkpoint
6: blue
7: purple
8: red
9: green
10: orange
11: yellow


status:
-1: not visited
-2: can not visit (giridam)
1: visited
0: seen and can visit
"""
def det_color(frame,num):
    for i in range(6):
        for j in range(6):
            green=frame[i,j,0]
            blue=frame[i,j,1]
            red=frame[i,j,2]
            print(f"clolors{num}: {green,blue,red}")
            tilenum=detectTile(green ,blue,red)
            if tilenum!=-1:
                return tilenum
    return -1
def tileColorDitaction():

    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    frame1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)

    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    frame2 = cv.cvtColor(image2, cv.COLOR_BGRA2BGR)
    # print(frame2.shape)

    frame2=frame2[34:40,29:35]
    frame1=frame1[34:40,29:35]

    color1=det_color(frame1,1)
    color2=det_color(frame2,2)
    if color1!=-1 :
        print(f"color1 {color1}")
    if color2!=-1 :
        print(f"color2 {color2}")
    cv.imshow("frame1",frame1)
    cv.waitKey(0)
    cv.imshow("frame2",frame2)
    cv.waitKey(0)
    # print(f"the frame one tile num {det_color(frame1)}")
    # print(f"the frame two tile num {det_color(frame2)}")

    # cv.imshow("frame2",frame2)
    # cv.waitKey(0)


# def routing(x_current, y_current):
#     print("Yooohooo 75% kar jam")
#     pass

def resize_pic(pic):
    new_w , new_h = 200, 200
    new_dim = (new_w, new_h)
    resized_pic = cv.resize(pic, new_dim, interpolation = cv.INTER_AREA)
    return resized_pic

def sortContour(approx):
    a = []
    b = []
    #Optimization
    # print("approx",approx)
    for i in range(4):
        a.append([approx[i,0,0],approx[i,0,1]])
    # print("AAAAAAAA",a)
    #Optimization
    for i in range(4):
        min = [100,100]
        for j in range(len(a)):
            if a[j][0]<min[0]:
                min = a[j]
                ir = j
        b.append(min)
        a.pop(ir)
    if(b[0][1]>b[1][1]):
        a = b[0]
        b[0]= b[1]
        b[1]= a
    if(b[2][1]>b[3][1]):
        a = b[2]
        b[2] = b[3]
        b[3]= a
    # print(np.array([[b[2]],[b[0]],[b[1]],[b[3]]]))
    return np.array([[b[2]],[b[0]],[b[1]],[b[3]]])

def im(img,side):
    global warped_img
    img_copy = img.copy()
    imgOrg= img
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_thesh = (90, 50, 20)
    upper_thesh = (100, 175, 175)

    thresh = cv.inRange(hsv, lower_thesh, upper_thesh)
    thresh = cv.bitwise_not(thresh)
    
    thresh_resize=resize_pic(thresh)
    contours , _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # cv.imshow(f"thresh {side}", thresh)
    # cv.waitKey(1)

    # print("jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj")

    if len(contours)>0:
        for contour in contours:

            epsilon = 0.1*cv.arcLength(contour,False)
            approx = cv.approxPolyDP(contour,epsilon,True)



            
            
            x_s,y_s,w,h =  cv.boundingRect(contour)
            cv.rectangle(contour, (x_s,y_s), (x_s+w, y_s+h), (0,0,255), 1)
            
            area = cv.contourArea(contour)
            contour_degree = atan(w/h)
            
            contour_degreee=math.degrees(contour_degree)

            
            if len(approx) == 4 and(( area>200 and x_s >1 and y_s>1 and x_s+w <=63 and y_s+h < 40)or(y_s>35 and y_s+h<10)):
                approx=sortContour(approx)
                approx2  = approx.reshape(4, 2).astype(np.float32)
                summ = approx.sum(axis=1)
                diff = np.diff(approx, axis=1)

                # top_l = approx[np.argmin(summ)]
                # top_r = approx[np.argmin(diff)]
                # bottom_l = approx[np.argmax(summ)]
                # bottom_r = approx[np.argmax(diff)]
                # box = np.array([top_l, top_r, bottom_l, bottom_r], dtype=np.float32)
                warp_array = np.array([[0, 0], [90, 0], [90,90], [0,90]], dtype=np.float32)
                
                boxed_img = cv.getPerspectiveTransform(approx2, warp_array)
                warped_img = cv.warpPerspective(img, boxed_img, (90,90))
                
                # cv.imshow("warp", warped_img)
                # cv.waitKey(1)
                print("area", area)
                
                cv.drawContours(imgOrg, contour, -1, (0,255,0), 2 )
                # cv.imshow("con",imgOrg)
            

                
                if not detect_victim(contour_degreee,contour,img_copy,side,warped_img):
                    detect_hazard(contour,img_copy,side,contour_degreee,warped_img)
                    pass

            
        # cv.imshow("contour",imgOrg)

        resized_contour=resize_pic(imgOrg)
        # cv.imshow(f"def


def detect_victim(contour_degreee,contour,image,side,warped_img):
    global patterns

    lower_b = np.array([0 , 0 , 0])
    upper_b = np.array([180 , 50 , 50])
    lower_w = np.array([0 , 0, 150])
    upper_w = np.array([180, 30, 255])
    cropped_cutout = warped_img
    hsv = cv.cvtColor(cropped_cutout, cv.COLOR_BGR2HSV)
    mask_b = cv.inRange(hsv, lower_b, upper_b)
    mask_w = cv.inRange(hsv, lower_w, upper_w)
    b_pixels = np.count_nonzero(mask_b)
    w_pixels = np.count_nonzero(mask_w)
    total = np.count_nonzero(cropped_cutout)
        
    if total ==0:
        return 0
        
    b_percentage = (b_pixels / total) * 100
    w_percentage = (w_pixels / total) * 100

    camIndex=0
    if side == "R":
        camIndex=1140
    else:
        camIndex=1448
    print("contour degree",contour_degreee)

    x_s, y_s, w, h = cv.boundingRect(contour)
    mid = [y_s+(h//2), x_s+(w//2)]
    cv.circle(image, mid,5, (0, 255 ,0),-1)
    # cv.imshow("mid", image)
    cropped_cutout = warped_img
    cv.waitKey(1)
    image_mid_point = [image.shape[0]//2, image.shape[1]//2]
    print(image_mid_point[1] - mid[1]," ", image_mid_point[0] - mid[0])
    print(f"camindex: {camIndex-(image_mid_point[1] - mid[1])}")
    # cv.imshow("c",cropped_cutout)
    _, binary = cv.threshold(cropped_cutout, 125, 255, cv.THRESH_BINARY_INV)

    h_split = 90 // 9
    w_split = 90 // 9
    matrix = np.zeros((9, 9), dtype=int)

    for i in range(9):
        for j in range(9):
            part = binary[i * h_split:(i + 1) * h_split, j * w_split:(j + 1) * w_split]  
            
            
            if np.sum(part) > 0:
                print(np.sum(part))
                matrix[i, j] = 1

    print(matrix)


    for letter, pattern in patterns.items():

        H = np.linalg.norm(patterns['H'] - matrix)
        S = np.linalg.norm(patterns['S'] - matrix)
        U = np.linalg.norm(patterns['U'] - matrix)
        UU = np.linalg.norm(patterns['uu'] - matrix)
        SS = np.linalg.norm(patterns['ss'] - matrix)
        SSS = np.linalg.norm(patterns['sss'] - matrix)
        HH = np.linalg.norm(patterns['hh'] - matrix)
        UUU = np.linalg.norm(patterns['uuu'] - matrix)
        UUUU = np.linalg.norm(patterns['uuuu'] - matrix)
        HHH = np.linalg.norm(patterns['hhh'] - matrix)
        HHHH = np.linalg.norm(patterns['hhhh'] - matrix)
        H6 = np.linalg.norm(patterns['h6'] - matrix)
        U5 = np.linalg.norm(patterns['u5'] - matrix)
        H7 = np.linalg.norm(patterns['h7'] - matrix)
        H8 = np.linalg.norm(patterns['h8'] - matrix)

        print(f"H:{min([h,HH])}, S:{min([S,SS])}, U:{min([U,UU,UUU])}")

        if H < 3 or HH<3 or HHH<3 or HHHH<3 or H6<4 or H7<3 or H8<3:
            print("H dtected")
            go(0,0)
            report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"h")
            
            return 1
        elif S < 3 or SS<3 or SSS<3: 
            print("S detected")
            go(0,0)
            report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"s")
            
            return 1
        elif U < 3 or UU<3 or UUU < 3 or UUUU<3 or U5<3:
            print("U detected")
            go(0,0)
            report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"u")
           
            return 1
        # elif b_percentage >= 2.5 and 20>w_percentage >= 1 :
        #         print("Corrosive_h")
        #         time_now = time.time()
        #         if time_now - last_p > 2:
        #             last_p,_=game()
        #             report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"c")
        
        
    return 0
        

    

def report(noting, deg,vic:str):
    vic = vic.capitalize()
    if r2_map[x][y]["HSU"]!=vic:
        report_victim(vic,*getgps())
        r2_map[x][y]["HSU"]=vic

def detect_hazard(contour,image,side,contour_degreee,warped_img):
        global last_p
        camIndex=0
        if side == "R":
            camIndex=1140
        else:
            camIndex=1448
        


        x_s, y_s, w, h = cv.boundingRect(contour)
        cropped_cutout = warped_img
        mid = [y_s+(h//2), x_s+(w//2)]
        cv.circle(image, mid,5, (0, 255 ,0),-1)
        # cv.imshow("mid", image)
        
        # cv.waitKey(1)
        image_mid_point = [image.shape[0]//2, image.shape[1]//2]

        # cv.imshow("c",cropped_cutout)
        hsv = cv.cvtColor(cropped_cutout, cv.COLOR_BGR2HSV)
        rgbo = cv.cvtColor(cropped_cutout, cv.COLOR_BGR2RGBA)
        # cv.imshow("rgbo", rgbo)

        resized_hsv = resize_pic(hsv)
        # cv.imshow("c",resized_hsv)

        lower_r1 = np.array([0, 120, 70])
        upper_r1 = np.array([10, 255, 255])
        lower_r2 = np.array([170, 120, 70]) 
        upper_r2 = np.array([180, 255, 255])
        lower_y = np.array([20, 120, 70])
        upper_y = np.array([40, 255, 255])
        lower_b = np.array([0 , 0 , 0])
        upper_b = np.array([180 , 50 , 50])
        lower_w = np.array([0 , 0, 150])
        upper_w = np.array([180, 30, 255])
        lower_blue = np.array([90, 140, 0])
        upper_blue = np.array([160, 255, 255])
            
        mask_r1 = cv.inRange(hsv, lower_r1, upper_r1)
        mask_r2 = cv.inRange(hsv, lower_r2, upper_r2)
        mask_r = cv.bitwise_or(mask_r1, mask_r2)
        mask_y = cv.inRange(hsv, lower_y, upper_y)
        mask_b = cv.inRange(hsv, lower_b, upper_b)
        mask_w = cv.inRange(hsv, lower_w, upper_w)
        mask_blue = cv.inRange(hsv, lower_blue, upper_blue)

        # cv.imshow("mask black",mask_b)
        # cv.imshow("mask white",mask_w)


        r_pixels = np.count_nonzero(mask_r)
        y_pixels = np.count_nonzero(mask_y)
        b_pixels = np.count_nonzero(mask_b)
        w_pixels = np.count_nonzero(mask_w)
        blue_pixels = np.count_nonzero(mask_blue)

        total = 90*90
        
        if total ==0:
            return 0
        
        blue_percentage = (blue_pixels/total)*100
        r_percentage = (r_pixels / total) * 100
        y_percentage = (y_pixels / total) * 100
        b_percentage = (b_pixels / total) * 100
        w_percentage = (w_pixels / total) * 100



        # nesbat = b_percentage/w_percentage
        # print("nesbat:",nesbat)
        print("red:", r_percentage)
        print("yellow:", y_percentage)
        print("black:", b_percentage)
        print("white:", w_percentage)
        print("blue:", blue_percentage)

        print(f"camindex: {camIndex-(image_mid_point[1] - mid[1])}")
        if r_percentage == 0:
            if b_percentage >= 5 and 60>w_percentage >= 1 :
                print("Corrosive_v")

                report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"c")
            elif 0<=b_percentage <2 and 93>w_percentage>88 :
                print("Poison")

                report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"p")
                                           
        else:
            if 10 < r_percentage and y_percentage<=1:
                print("flammable gas")
                # time_now = time.time()
               
                    # last_p,_=gameinfo()
                report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"f")
                
            elif 10<y_percentage :
                print("organic peroxide")


                report(0,camIndex-(image_mid_point[1] - mid[1])/64*60/360*512,"o")   




def vision():
    
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    frame1 = cv.cvtColor(image1, cv.COLOR_BGRA2BGR)

    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    frame2 = cv.cvtColor(image2, cv.COLOR_BGRA2BGR)
    framHAZ1 = np.copy(frame1)
    framHAZ2 = np.copy(frame2)
    im(frame1,"R")
    im(frame2,"L")
    
    
    return

        
def path_follow(locations):
    for loc in locations:
        forward(getLocation(loc))
    

go_to_home = 0    
pc = 0
def a_star():
    go(0,0)
    global go_to_home, pc
    a_data[:210, :210] = -1 
    
    print("ssssdddddeeeee",x,y)
    a_data[x][y] = 0


    direction_init = abs_yaw()
    break_kon = 0
    for cnt in range(0, 200):

        positions = np.argwhere(a_data == cnt)
        for x_a_star, y_a_star in positions:
            k = 0
            lst_raha = get_kiasha_ways((x_a_star,y_a_star))
            lst_mohem = [(180, 0, -1), (135, 1, -1), (90, 1, 0), (45, 1, 1),(360, 0, 1), (315, -1, 1), (270, -1, 0), (225, -1, -1)]
            print(f"XXXXX: {x_a_star}, YYYYYYY: {y_a_star}")
            if r2_map[x_a_star][y_a_star]["status"] == 0:
                break_kon = 1
                break

            for item in lst_raha:
                for angle, dx, dy in lst_mohem:
                    if int(item) == angle and a_data[x_a_star + dx][y_a_star + dy] == -1:
                        x_simul = x_a_star + dx
                        y_simul = y_a_star + dy
                        if r2_map[x_simul][y_simul]["type"] == 3:
                            k = 3
                        if [x_simul, y_simul] in multyply_cost:
                            k = 100 ## bishtar shavad
                        if r2_map[x_simul][y_simul]["status"] == -2 or r2_map[x_simul][y_simul]["type"] == 2 :
                            k = (-1 * cnt) - 2 

                        a_data[x_simul][y_simul] = cnt + 1 + k
                
            if break_kon == 1:
                break
        if break_kon == 1:
            break



    if break_kon == 0:
        pc += 1
        path_clearance()

        if pc >= 2:
            if x == 100 and y == 100:
                print("Algorithm work has been done!")
                exitProcess()
                return None
                
            else:
                print("GO TO HOMEEEEEEE ")  
                x_a_star = 100
                y_a_star = 100


    a = routing(x_a_star, y_a_star)
    print("salam kasra fazeli")
    print(a)

    # print(a[::-1])
    try:
        b = a[::-1]
        b.append((x_a_star,y_a_star))
    except:
        b= None 
    print(f"Destinaion is X: {x_a_star} Y:{y_a_star} \n")

    for i in range(95 - 3, 105 + 3):
        for j in range(95 - 3, 105 + 3):
            if a_data[j][i] == -1:
                print(" . ", end="")
            elif a_data[j][i] == 0:
                print(" = ", end="")
            else:
                print(f"{a_data[j][i]:3}", end="")
        print("\n")
    print("\n")


    return b

    # print_Astar_map(a_data, (x_a_star, y_a_star))

def routing(x_current, y_current):
    lst_paths = []
    cnt_routing = 0
    x_target = x_current
    y_target = y_current

    # mapData[x_current][y_current] = -1
    lst_values = np.zeros([8])

    while robot.step(timeStep) != -1:
        # print("972 972 972\n")
        cnt_routing += 1
        # if cnt_routing > a_data[x_current][y_current] + 20:
        #     print("Too routing gir kardi, alan dorostesh mikonam:) \n")
        #     if x_target == 100 and y_target == 100:
        #         finish()
        #     else:
        #         mapData[x_target][y_target] = 1
        # print(cnt_routing)


##### bayad dorost she


        if cnt_routing > a_data[x_current][y_current] + 450:
            print("Too routing gir kardi, alan dorostesh mikonam:) \n")
            if x_target == 100 and y_target == 100:
                lackofprog()
                return None

                # exitProcess()
            else:
                lackofprog()
                return None
                # mapData[x_target][y_target] = 1

        mini = 100
        mini_num = -1
        lst_values[0] = a_data[x_current][y_current - 1]
        lst_values[1] = a_data[x_current + 1][y_current - 1]
        lst_values[2] = a_data[x_current + 1][y_current]
        lst_values[3] = a_data[x_current + 1][y_current + 1]
        lst_values[4] = a_data[x_current][y_current + 1]
        lst_values[5] = a_data[x_current - 1][y_current + 1]
        lst_values[6] = a_data[x_current - 1][y_current]
        lst_values[7] = a_data[x_current - 1][y_current - 1]

        for i in range(0, 8):
            if lst_values[i] < mini and lst_values[i] != -1:
                mini = lst_values[i]
                mini_num = i

        # if mapData[x_map][y_map] == -1:
        #     break

        if x_current == x  and y_current == y:
            print("beheshti")
            break
        if mini_num == 0:
            # mapData[x_current][y_current - 2] = -1
            lst_paths.append((x_current,y_current-1))
            x_current = x_current
            y_current = y_current - 1

        if mini_num == 1:
            # mapData[x_current + 2][y_current - 2] = -1
            lst_paths.append((x_current+1,y_current-1))

            x_current = x_current + 1
            y_current = y_current - 1
        if mini_num == 2:
            # mapData[x_current + 2][y_current] = -1
            lst_paths.append((x_current+1,y_current))


            x_current = x_current + 1
            y_current = y_current

        if mini_num == 3:
            # mapData[x_current + 2][y_current + 2] = -1
            lst_paths.append((x_current+1,y_current+1))


            x_current = x_current + 1
            y_current = y_current + 1

        if mini_num == 4:
            # mapData[x_current][y_current + 2] = -1

            lst_paths.append((x_current,y_current+1))


            x_current = x_current
            y_current = y_current + 1
        if mini_num == 5:
            # mapData[x_current - 2][y_current + 2] = -1
            lst_paths.append((x_current-1,y_current+1))


            x_current = x_current - 1
            y_current = y_current + 1
        if mini_num == 6:
            # mapData[x_current - 2][y_current] = -1
            lst_paths.append((x_current-1,y_current))


            x_current = x_current - 1
            y_current = y_current
        if mini_num == 7:
            # mapData[x_current - 2][y_current - 2] = -1
            lst_paths.append((x_current-1,y_current-1))

            x_current = x_current - 1
            y_current = y_current - 1

    return lst_paths



def give_delta_xy(dirs):
    lst_dirs = dirs
    target_dir = best_target(lst_dirs)
    lst_xy = ["180:0:-1", "135:1:1", "90:1:0", "45:1:1", "360:0:1", "315:-1:1", "270:-1:0", "225:-1:-1"]
    for item in lst_xy:
        if target_dir == item.split(":")[0]:
            pass





# **********************main while**********************

def main_robot():
    global flag, cordinent
    while robot.step(timeStep) != -1:

        if flag == 0:

            cordinent = getgps()
            for i in range(10):

                lidarProMax()

            flag = 1
            output = lidar_output()
            if output[0] < 7 or output[3] < 7:
                rotation_for_degrees(90)
            logging.info("Setup Tasks Completed.")



        nav() 
        # navigation()
        # right_search()
        # escape()
        # backward(6)



    



if __name__ == "__main__":


    tilesGrid = GridBase()

    obstacleRemover = DeleteObstacles()

    # app_thread = Thread(target=run_pyqt, daemon=True)
    # app_thread.start()

    robot_thread = Thread(target=main_robot, daemon=True)
    robot_thread.start()

    robot_thread.join()
    # main_robot()