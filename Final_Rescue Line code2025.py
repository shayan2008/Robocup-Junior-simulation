from controller import Keyboard
from controller import Robot
import sys
from controller import InertialUnit
from controller import GPS
from controller import Lidar
from controller import Receiver, Emitter
import struct
import string
import numpy as np
from math import *

# CONSTANTS
RED_COLOR = "\033[31m"
GREEN_COLOR = "\033[32m"
BLUE_COLOR = "\033[34m"
GREY_COLOR = "\033[98m"
YELLOW_COLOR = "\033[0;33m"
CYAN_COLOR = "\033[0;36m"
RESET_COLOR = "\033[39m"

X_OFFSET = 100
Y_OFFSET = 100

ERRORCHECKWALL = 225

x_map = X_OFFSET
y_map = Y_OFFSET

NORTH = 0
NORTH_F = 1

NORTH_RIGHT = 22.5
NORTH_RIGHT_F = 23.5

NORTH_EAST = 45
NORTH_EAST_UP = 46
NORTH_EAST_RIGHT = 44

EAST_UP = 67.5
EAST_UP_F = 68.5

EAST = 90
EAST_F = 91

EAST_DOWN = 112.5
EAST_DOWN_F = 113.5

SOUTH_EAST = 135
SOUTH_EAST_RIGHT = 111.5
SOUTH_EAST_DOWN = 156.5

SOUTH_RIGHT = 157.5
SOUTH_RIGHT_F = 158.5

WEST_UP = -67.5
WEST_UP_F = -66.5

WEST_DOWN = -112.5
WEST_DOWN_F = -111.5

SOUTH = -180
SOUTH_F = -179

SOUTH_LEFT = -157.5
SOUTH_LEFT_F = -156.5

WEST = -90
WEST_F = -89

SOUTH_WEST = -135
SOUTH_WEST_DOWN = -134
SOUTH_WEST_LEFT = -111.5

NORTH_WEST = -45
NORTH_WEST_UP = -44
NORTH_WEST_LEFT = -46

NORTH_LEFT = -22.5
NORTH_LEFT_F = -21.5

NORTHD = 0.5
EASTD = 90.5
SOUTHD = -179.5
WESTD = -89.5

RED = 1
GREEN = 2
BLUE = 3
PURPLE = 4
GRAY = 5
BLACK = 6
WHITE = 7
KAKI = 8
YELLOW = 9
ORANGE = 10

room4_found = 0
BlackHoleFound = 0
victim_found_this_move = 0

remaining_time = 0
score = 0
check_room4 = 0



NORTHN = 0.5

NORTH_EASTN = 45.5

EASTN = 90.5

SOUTH_EASTN = 135.5

SOUTHN = -179.5

NORTH_WESTN = -44.5

WESTN = -89.5

SOUTH_WESTN = -134.5



ERRORCHECKWALL = 255


redSee = 0
greenSee = 0
orangeSee = 0

robot = Robot()
timeStep = 32

Camera1 = robot.getDevice("camera1")
Camera2 = robot.getDevice("camera2")

colour_sensor = robot.getDevice("colour_sensor")
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
gps = robot.getDevice("gps")
timestep = int(robot.getBasicTimeStep())

receiver.enable(timestep)

dis_sensor1 = robot.getDevice("DM")
dis_sensor2 = robot.getDevice("DR")
dis_sensor3 = robot.getDevice("DL")
dis_sensor4 = robot.getDevice("R45")
dis_sensor5 = robot.getDevice("L45")

dis_sensor1.enable(timeStep)
dis_sensor2.enable(timeStep)
dis_sensor3.enable(timeStep)
dis_sensor4.enable(timeStep)
dis_sensor5.enable(timeStep)

Camera1.enable(timestep)
Camera2.enable(timestep)

wheelL = robot.getDevice("wheelL motor")
wheelR = robot.getDevice("wheelR motor")

IMU = robot.getDevice("imu")

IMU.enable(timeStep)
gps = robot.getDevice("gps")

colour_sensor.enable(timestep)

gps.enable(timeStep)
wheelL.setVelocity(0)
wheelR.setVelocity(0)
wheelL.setPosition(float("inf"))
wheelR.setPosition(float("inf"))

mapData = np.zeros([250, 250])
mapData2 = np.zeros([250, 250])
a_data = np.zeros([210, 210])

reset_flag = 0

lid = robot.getDevice("lidar")
lid.enable(timeStep)
# Create a robot instance
keyboard = Keyboard()
# Enable keyboard input with a sampling period of 64 milliseconds
keyboard.enable(64)


def delay(num):
    robot.step(num)


def go(spl, spr):
    if spl > 100:
        spl = 100
    elif spl < -100:
        spl = -100
    if spr > 100:
        spr = 100
    elif spr < -100:
        spr = -100
        
    wheelL.setVelocity(spl * 6.28 / 100)
    wheelR.setVelocity(spr * 6.28 / 100)


def ReadLidar(input):
    if input == "F":
        return pixel_avearge(0)
    elif input == "FR":
        return pixel_avearge(32)
    elif input == "FL":
        return pixel_avearge(480)
    elif input == "R":
        return pixel_avearge(128)
    elif input == "RF":
        return pixel_avearge(96)
    elif input == "RB":
        return pixel_avearge(160)
    elif input == "B":
        return pixel_avearge(256)
    elif input == "BR":
        return pixel_avearge(224)
    elif input == "BL":
        return pixel_avearge(288)
    elif input == "L":
        return pixel_avearge(384)
    elif input == "LF":
        return pixel_avearge(416)
    elif input == "LB":
        return pixel_avearge(352)
    elif input == "R45":
        return pixel_avearge(64)
    elif input == "R135":
        return pixel_avearge(192)
    elif input == "L45":
        return pixel_avearge(448)
    elif input == "L135":
        return pixel_avearge(320)


def pixel_avearge(pixel_number):
    image = lid.getRangeImage()
    image_min = 1000
    # pixel_number = pixel_number - 3
    # for i in range(pixel_number, pixel_number + 7):
    pixel_number = pixel_number - 5  # 10
    for i in range(pixel_number, pixel_number + 11):  # 22
        if pixel_number < 0:
            if (image[pixel_number + 512 + 1024] * 1000) < image_min:
                image_min = image[pixel_number + 512 + 1024] * 1000

                # print(image[pixel_number + 512 + 1024] * 1000)
        elif pixel_number > 511:
            pixel_number = pixel_number - 512
            if (image[pixel_number - 512 + 1024] * 1000) < image_min:
                image_min = image[pixel_number - 512 + 1024] * 1000
        else:
            if (image[pixel_number + 1024] * 1000) < image_min:
                image_min = image[pixel_number + 1024] * 1000
        pixel_number = pixel_number + 1

    result = int(image_min)
    return result


def get_gps_x():
    g = gps.getValues()
    g[0] = g[0] * 1000
    return int(g[0])


def get_gps_y():
    g = gps.getValues()
    g[2] = g[2] * 1000
    return int(g[2])


def update_coordinate_map():
    # print(f"x_start = {GPS_X_START} y_start= {GPS_Y_START}")
    # print(f"gps_x= {get_gps_x()} gps_y={get_gps_y()}")
    global x_map, y_map
    x_map = floor(((get_gps_x() - GPS_X_START + 15) / 30.001)) + X_OFFSET
    y_map = floor(((get_gps_y() - GPS_Y_START + 15) / 30.001)) + Y_OFFSET
    # print(f"gps_x2= {x_map} gps_y2={y_map}")


def get_yaw():
    imu = IMU.getRollPitchYaw()
    return (imu[2] * 180 / pi) * -1


def dir_update():
    # yaw = get_yaw()
    # global dir
    # print(f"diiiiirection3333:{dir}")

    if 5 > get_yaw() > -5:
        return NORTH
    elif 50 > get_yaw() > 40:
        return NORTH_EAST
    elif 95 > get_yaw() > 85:
        return EAST
    elif 140 > get_yaw() > 130:
        return SOUTH_EAST
    elif -175 > get_yaw() or get_yaw() > 175:
        return SOUTH
    elif -130 > get_yaw() > -140:
        return SOUTH_WEST
    elif -85 > get_yaw() > -95:
        return WEST
    elif -40 > get_yaw() > -50:
        return NORTH_WEST

    else:
        # print(f"diiiiiiiiiiiiiiiiir={dir} \n")
        print(f"!!!!!!!!!!!! direction else dirUpdate !!!!!!!!!!!!!!!\n")
        # return dir

    # return dir_value


def color():
    global check_room4, redSee, orangeSee, greenSee
    output = 0
    image = colour_sensor.getImage()
    red = colour_sensor.imageGetRed(image, 1, 0, 0)
    green = colour_sensor.imageGetGreen(image, 1, 0, 0)
    blue = colour_sensor.imageGetBlue(image, 1, 0, 0)
    # print(f"R : {red} , G : {green} , B : {blue}")

    if (
        (red > 245 and red < 256)
        and (green > 57 and green < 69)
        and (blue > 57 and blue < 69)
    ):
        # 252,63,63
        # 237,48,48
        output = RED
        print("Red Found")

        if redSee == 0:
            if check_room4 == 0:
                check_room4 = 1
            else:
                check_room4 = 0
        redSee = 1
        # forward()

    elif (
        (red > 27 and red < 39)
        and (green > 243 and green < 256)
        and (blue > 27 and blue < 39)
    ):
        # 31,249,31
        # 25,230,25
        output = GREEN
        if greenSee == 0:
            if check_room4 == 0:
                check_room4 = 1
            else:
                check_room4 = 0
        
        greenSee = 1
        # forward()
    elif (
        (red > 204 and red < 216)
        and (green > 170 and green < 184)
        and (blue > 94 and blue < 108)
    ):
        # 210,176,102
        # 173,139,77
        output = KAKI
    elif (
        (red > 58 and red < 70)
        and (green > 58 and green < 70)
        and (blue > 245 and blue < 256)
    ):
        # 64,64,252
        # 48,48,237
        output = BLUE
    elif (
        (red > 138 and red < 150)
        and (green > 57 and green < 69)
        and (blue > 219 and blue < 231)
    ):
        # 144,63,225
        # 112,48,191
        output = PURPLE
    elif (
        (red > 245 and red < 256)
        and (green > 245 and green < 256)
        and (blue > 245 and blue < 256)
    ):
        # 248 ta 250
        # 42,47,64
        output = GRAY
    elif (
        (red > 246 and red < 256)
        and (green > 246 and green < 256)
        and (blue > 58 and blue < 70)
    ):
        # 252,252,64
        # 237,237,48
        output = YELLOW
    elif (
        (red > 246 and red < 256)
        and (green > 220 and green <= 232)
        and (blue > 57 and blue < 69)
    ):
        # 252,226,63
        # 237,191,48
        output = ORANGE
        if orangeSee == 0:
            if check_room4 == 0:
                check_room4 = 1
            else:
                check_room4 = 0
        orangeSee = 1
        
        print("Orange Found")
        # forward()

    elif (
        (red > 35 and red < 50)
        and (green > 35 and green < 50)
        and (blue > 35 and blue < 50)
    ):
        # 41,41,41
        output = BLACK
    else:
        output = WHITE
        redSee = 0
        greenSee = 0
        orangeSee = 0
    return output


def p_find(select):
    global  p_vic_is_found
    p_image = select.getImage()
    p_white_pixel_h = 0
    p_white_pixel_last = 0
    p_white_pixel = 0
    p_white_pixel_w = 0
    p_white_pixel_h_2 = 0
    p_white_pixel_h2_2 = 0
    p_breaK = 0
    p_breaK2 = 0 
    p_breaK_2 = 0
    p_breaK2_2  = 0
    count_w = 0
    count_w2 = 0
    p_add = 0
    p_there_is_some_thing = 0 
    p_w = 0
    p_w2 = 0
    v = 0
    found_vic = 0
    found_vic2 = 0
    wcheck = 0
    if select==Camera1:
        if dir==NORTH:
            if mapData[x_map-2,y_map] >= 30:
                p_add = 1
        if dir==SOUTH:
            if mapData[x_map,y_map-2] >= 30:
                p_add = 1
        if dir==WEST:
            if mapData[x_map,y_map+2] >= 30:
                p_add = 1
        if dir==EAST:
            if mapData[x_map+2,y_map] >= 30:
                p_add = 1
    if select==Camera2:
        if dir==NORTH:
            if mapData[x_map-2,y_map] >= 30:
                p_add = 1
        if dir==SOUTH:
            if mapData[x_map,y_map+2] >= 30:
                p_add = 1
        if dir==WEST:
            if mapData[x_map,y_map-2] >= 30:
                p_add = 1
        if dir==EAST:
            if mapData[x_map-2,y_map] >= 30:
                p_add = 1

    for x1 in range(64):
        rx = select.imageGetRed(p_image, 64 , x1 ,25)
        gx = select.imageGetGreen(p_image, 64 , x1,25)
        bx = select.imageGetBlue(p_image, 64 , x1,25)
        if rx in range(0xA1- 2  ,0xA1+20) and gx in range(0xA1-2 ,0xA1+ 20) and bx in range(0xA1 -2 ,0xA1+ 20):
            wcheck = x1
            found_vic += 1
    if found_vic > 3:
        rx3 = select.imageGetRed(p_image, 64 , wcheck ,24)
        gx3 = select.imageGetGreen(p_image, 64 , wcheck,24)
        bx3 = select.imageGetBlue(p_image, 64 , wcheck,24)
        if rx3 in range(0xA1- 2  ,0xA1+20) and gx3 in range(0xA1-2 ,0xA1+ 20) and bx3 in range(0xA1 -2 ,0xA1+ 20):
            found_vic2 = 1 
    if found_vic2 ==0 :
        if p_add == 0:
            for w4 in range(63 , 0 , -1):
                for h4 in range(40):
                    r4 = select.imageGetRed(p_image, 64 , w4 ,h4)
                    g4 = select.imageGetGreen(p_image, 64 , w4,h4)
                    b4 = select.imageGetBlue(p_image, 64 , w4,h4)
                    if p_w2 == 0 and r4 in range(195 , 255) and g4 in range(195 ,255) and b4 in range(195 , 255) :
                        p_white_pixel_last = w4  # first white pixel in top anl left
                        p_w2 = 1
                        break
            for w2 in range(64):
                for h2 in range(40):
                    r = select.imageGetRed(p_image, 64 , w2 ,h2)
                    g = select.imageGetGreen(p_image, 64 , w2,h2)
                    b = select.imageGetBlue(p_image, 64 , w2,h2)
                    if p_w == 0 and r in range(170 , 255) and g in range(170 ,255) and b in range(170 , 255) :
                        p_white_pixel = w2  # first white pixel in top anl left
                        p_w = 1
                        break
            v = p_white_pixel_last - p_white_pixel
            if p_white_pixel != 0 and p_white_pixel_last != 0:
                if v > 5:
                    for ww in range(15 , 55): # to find if there is something
                        rw = select.imageGetRed(p_image, 64 ,ww ,20)
                        gw = select.imageGetGreen(p_image, 64 ,ww ,20)
                        bw = select.imageGetBlue(p_image, 64 ,ww ,20)
                        # agar chizi bood
                        if not(rw in range(18-5 , 18+5) and gw in range(30-5 , 30+5) and bw in range(32-5 , 32+5) or rw in range(59-5 , 59+5) and gw in range(123-5 , 123+5) and bw in range(134-5 ,134+5)) and p_there_is_some_thing == 0:
                            p_there_is_some_thing = 1
                            break
                    if p_there_is_some_thing==1:
                        if v > 35:
                            count_w = 0
                            p_breaK_2 = 0
                            for w_2 in range(40):
                                for h_2 in range(15):
                                    r_2 = select.imageGetRed(p_image, 64 , w_2 ,h_2)
                                    g_2 = select.imageGetGreen(p_image, 64 , w_2,h_2)
                                    b_2 = select.imageGetBlue(p_image, 64 , w_2,h_2)
                                    if r_2 in range(172,255) and g_2 in range(172,255) and b_2 in range(172,255) and p_breaK_2 == 0:
                                        p_white_pixel_h_2 = w_2
                                        p_breaK_2 = 1
                                        for hw in range(40):
                                            r_2_1 = select.imageGetRed(p_image, 64 , p_white_pixel_h_2 ,hw)
                                            g_2_1 = select.imageGetGreen(p_image, 64 , p_white_pixel_h_2,hw)
                                            b_2_1 = select.imageGetBlue(p_image, 64 ,p_white_pixel_h_2 ,hw)
                                            if r_2_1 in range(172,255) and g_2_1 in range(172,255) and b_2_1 in range(172,255):
                                                count_w +=1
                                        break
                            # p_breaK_2 = 0
                            # for w2_2 in range(39 , 0 , -1):
                            #     for h2_2 in range(14 , 0 , -1):
                            #         r2_2 = select.imageGetRed(p_image, 64 , w2_2 ,h2_2)
                            #         g2_2 = select.imageGetGreen(p_image, 64 , w2_2,h2_2)
                            #         b2_2 = select.imageGetBlue(p_image, 64 , w2_2,h2_2)
                            #         if r2_2 in range(172,210) and g2_2 in range(172,210) and b2_2 in range(172,210) and p_breaK2_2 == 0:
                            #             p_white_pixel_h2_2 = h2_2
                            #             p_breaK_2 = 1
                            #             for hw_2 in range(40):
                            #                 r_2_2 = select.imageGetRed(p_image, 64 , p_white_pixel_h2_2 ,hw_2)
                            #                 g_2_2 = select.imageGetGreen(p_image, 64 , p_white_pixel_h2_2,hw_2)
                            #                 b_2_2 = select.imageGetBlue(p_image, 64 ,p_white_pixel_h2_2 ,hw_2)
                            #                 if r_2_2 in range(172,210) and g_2_2 in range(172,210) and b_2_2 in range(172,210):
                            #                     count_w2 +=1
                            #             break
                            if not(0<p_white_pixel_h_2 < 10 or p_white_pixel_h_2  > 31):
                                if select == Camera1:
                                    if ReadLidar("L") < 80:
                                        if count_w >15: # square
                                            p_victim(select)
                                        elif 0<count_w < 15: # lozi
                                            p_hazmat(select)
                                    elif ReadLidar("L") >= 80:
                                        if count_w >=10: # square
                                            p_victim(select)
                                        elif 0<count_w < 10: # lozi
                                            p_hazmat(select)
                                            print("-----------------------------------",count_w, p_white_pixel_h_2)
                                if select == Camera2:
                                    if ReadLidar("R") < 80:
                                        if count_w >= 15: # square
                                            p_victim(select)
                                        elif 0<count_w < 15: # lozi
                                            p_hazmat(select)
                                    elif ReadLidar("R") >= 80:
                                        if count_w >= 10: # square
                                            p_victim(select)
                                        elif 0<count_w < 10: # lozi
                                            p_hazmat(select)
                        elif v <= 35:
                            for hc in range(40):   # is picture complete or incomplete?
                                rc = select.imageGetRed(p_image, 64  ,0 ,hc)
                                gc = select.imageGetGreen(p_image, 64 , 0,hc)
                                bc = select.imageGetBlue(p_image, 64 , 0,hc)
                                rc2 = select.imageGetRed(p_image, 64 ,63 ,hc)
                                gc2 = select.imageGetGreen(p_image, 64 , 63,hc)
                                bc2 = select.imageGetBlue(p_image, 64 , 63,hc)
                                if rc in range(0 , 10) and gc in range(0 ,20) and bc in range(0 , 20) or rc2 in range(0 , 10) and gc2  in range(0 ,20) and bc2 in range(0 , 20) or rc in range(172,210) and gc in range(172,210) and bc in range(172,210) or rc2 in range(172,210) and gc2  in range(172,210) and bc2 in range(172,210):
                                    break
                            else:  # picture is complete. check if lozi or square?
                                count_w = 0
                                p_breaK_2 = 0
                                for w_2 in range(40):
                                    for h_2 in range(15):
                                        r_2 = select.imageGetRed(p_image, 64 , w_2 ,h_2)
                                        g_2 = select.imageGetGreen(p_image, 64 , w_2,h_2)
                                        b_2 = select.imageGetBlue(p_image, 64 , w_2,h_2)
                                        if r_2 in range(172,255) and g_2 in range(172,255) and b_2 in range(172,255) and p_breaK_2 == 0:
                                            p_white_pixel_h_2 = w_2
                                            p_breaK_2 = 1
                                            for hw in range(40):
                                                r_2_1 = select.imageGetRed(p_image, 64 , p_white_pixel_h_2 ,hw)
                                                g_2_1 = select.imageGetGreen(p_image, 64 , p_white_pixel_h_2,hw)
                                                b_2_1 = select.imageGetBlue(p_image, 64 ,p_white_pixel_h_2 ,hw)
                                                if r_2_1 in range(172,255) and g_2_1 in range(172,255) and b_2_1 in range(172,255):
                                                    count_w +=1
                                            break
                                if not(0<p_white_pixel_h_2 < 10 or p_white_pixel_h_2  > 31):
                                    if select == Camera1:
                                        if ReadLidar("L") < 80:
                                            if count_w >15: # square
                                                p_victim(select)
                                            elif 0<count_w < 15: # lozi
                                                p_hazmat(select)
                                        elif ReadLidar("L") >= 80:
                                            if count_w >=10: # square
                                                p_victim(select)
                                            elif 0<count_w < 10: # lozi
                                                p_hazmat(select)
                                    elif select == Camera2:
                                        if ReadLidar("R") < 80:
                                            if count_w >= 15: # square
                                                p_victim(select)
                                            elif 0<count_w < 15: # lozi
                                                p_hazmat(select)
                                        elif ReadLidar("R") >= 80:
                                            if count_w >= 10: # square
                                                p_victim(select)
                                            elif 0<count_w < 10: # lozi
                                                p_hazmat(select)
            elif p_white_pixel == 0 and p_white_pixel_last == 0:
                p_hazmat2(select)


def p_hazmat(select):
    global  p_vic_is_found
    p_image = select.getImage()
    p_black_pixel = 0
    p_m2 = 0
    p_m3 = 0
    p_d = 0
    p_h_w2 = 0
    p_h_w = 0
    p_h_white_pixel = 0
    p_h_white_pixel_last = 0 
    p_w_last  = 0 
    p_h_last = 0
    p_h_first = 0
    p_w_first  = 0 
    p_x_first= 0
    count_w = 0
    p_detect = 0
    p_bl = 0
    p_wh = 0    
    for w2 in range(64):
        for h2 in range(40):
            r = select.imageGetRed(p_image, 64 , w2 ,h2)
            g = select.imageGetGreen(p_image, 64 , w2,h2)
            b = select.imageGetBlue(p_image, 64 , w2,h2)
            if p_h_w == 0 and r in range(170 , 255) and g in range(160 ,255) and b in range(160, 255) :
                p_h_white_pixel = w2  # first white pixel in top anl left
                p_h_w = 1
                break
    for w3 in range(63 , 0 , -1):
        for h3 in range(40):
            r2 = select.imageGetRed(p_image, 64 , w3 ,h3)
            g2 = select.imageGetGreen(p_image, 64 , w3,h3)
            b2 = select.imageGetBlue(p_image, 64 , w3,h3)
            if p_h_w2 == 0 and r2 in range(170 , 255) and g2 in range(160 ,255) and b2 in range(160 , 255) :
                p_h_white_pixel_last = w3  # first white pixel in top anl left
                p_h_w2 = 1
                break
    p_h_middle = (p_h_white_pixel + p_h_white_pixel_last) // 2
    if p_vic_is_found==0:
        for h6 in range(17 , 35):
            r = select.imageGetRed(p_image, 64 , p_h_middle ,h6)
            g = select.imageGetGreen(p_image, 64 , p_h_middle,h6)
            b = select.imageGetBlue(p_image, 64 , p_h_middle,h6)
            if r in range(0 , 10) and g in range(0 ,20) and b in range(0 , 20):
                p_bl += 1
            if p_bl >1 :
                send_victim(select, "C", 33)
                break
    if p_vic_is_found==0:
        for w8 in range(20 , 40):
            r = select.imageGetRed(p_image, 64 , p_h_middle ,w8)
            g = select.imageGetGreen(p_image, 64 , p_h_middle,w8)
            b = select.imageGetBlue(p_image, 64 , p_h_middle,w8)
            if r in range(0xCF-5,0xCF+5) and g in range(0xCF-5,0xCF+5)  and b  in range(0xCF-5,0xCF+5):
                p_wh += 1 
            if p_wh >2:
                send_victim(select, "P", 34)
                break
    if p_vic_is_found==0:
        for hf in range (40):
            r = select.imageGetRed(p_image, 64 , p_h_middle ,hf)
            g = select.imageGetGreen(p_image, 64 ,  p_h_middle,hf)
            b = select.imageGetBlue(p_image, 64 , p_h_middle ,hf)
            if r in range(0xC5-50,0xC5+30) and g in range (40) and b  in range(0x4F-50,0x4F+40):
                    send_victim(select, "F", 35)
                    break
def p_hazmat2(select):
    global  p_vic_is_found
    p_image = select.getImage()
    o_p = 0
    for hc in range(40):   # is picture complete or incomplete?
        rc = select.imageGetRed(p_image, 64  ,0 ,hc)
        gc = select.imageGetGreen(p_image, 64 , 0,hc)
        bc = select.imageGetBlue(p_image, 64 , 0,hc)
        rc2 = select.imageGetRed(p_image, 64 ,63 ,hc)
        gc2 = select.imageGetGreen(p_image, 64 , 63,hc)
        bc2 = select.imageGetBlue(p_image, 64 , 63,hc)
        if rc in range(0xCF-50,0xCF+30) and gc in range(0xC1-50,0xC1+30) and bc in range(0,40) or rc2 in range(0xCF-50,0xCF+30) and gc2  in range(0xC1-50,0xC1+30) and bc2 in range(0 , 40):
            break
    else: 
        for h in range(30 , 40):
            for w in range(64):
                for h2 in range(10):
                    r = select.imageGetRed(p_image, 64 , w ,h)
                    g = select.imageGetGreen(p_image, 64 , w,h)
                    b = select.imageGetBlue(p_image, 64 ,w ,h)
                    r2 = select.imageGetRed(p_image, 64 , w ,h2)
                    g2 = select.imageGetGreen(p_image, 64 , w,h2)
                    b2 = select.imageGetBlue(p_image, 64 ,w ,h2)
                    if o_p == 0 and r in range(0xCF-50,0xCF+30) and g in range (0xC1-50,0xC1+10) and b  in range(0,40) and not(r2 in range(0xCF-50,0xCF+30) and g2 in range (0xC1-50,0xC1+10) and b2  in range(0,40)) :
                        send_victim(select, 'O', 36)
                        o_p = 1
                        break
                    break
            break

def p_victim(select):
    print("victim")
    global  p_vic_is_found
    p_image = select.getImage()
    p_black_pixel = 0
    p_black_pixel2 = 0
    p_white_pixel_h = 0
    p_white_pixel_h_last = 0
    d = 0
    p_w = 0
    p_w2 = 0
    p_wuh = 0
    first_base  = 0
    second_base = 0
    p_white_pixel = 0
    p_white_pixel_last = 0
    p_counter = 0
    p_w_last  = 0 
    p_h_last = 0
    p_h_first = 0
    p_w_first  = 0 
    p_wu = 0
    p_ww = 0
    p_x_first= 0
    p_middle = 0
    p_wr = 0
    p_wl = 0
    last_first_h = 0
    from_left = 0
    from_right = 0
    check = 0
    for wn in range(28, 36):
        rn = select.imageGetRed(p_image, 64 , wn ,20)
        gn = select.imageGetGreen(p_image, 64 , wn,20)
        bn = select.imageGetBlue(p_image, 64 , wn,20)
        if not( check == 0 or 
            rn in range(0 , 20) and 
            gn in range(0 ,20) and 
            bn in range(0 , 20) 
            or 
            rn in range(20 , 35) 
            and gn in range(20 , 35) 
            and bn in range(20 , 35)
            or 
            rn in range(39 , 55) 
            and gn in range(39 , 55) 
            and bn in range(39 , 55)
            or 
            rn in range(55 , 75) 
            and gn in range(55 , 75) 
            and bn in range(55 , 75)
            or 
            rn in range(75 , 105) 
            and gn in range(75 , 105) 
            and bn in range(75 , 105)
            or
            rn in range(105 , 135) 
            and gn in range(105 , 135) 
            and bn in range(105 , 135)
            or 
            rn  in range(135 , 155) 
            and gn in range(135 , 155) 
            and bn in range(135 , 155) 
            or 
            rn in range(200 , 255) 
            and gn in range(200 ,255) 
            and bn in range(200 , 255) ):
            check == 1
            break
    else:
        for w4 in range(63 , 0 , -1):
            for h4 in range(40):
                r4 = select.imageGetRed(p_image, 64 , w4 ,h4)
                g4 = select.imageGetGreen(p_image, 64 , w4,h4)
                b4 = select.imageGetBlue(p_image, 64 , w4,h4)
                if p_w2 == 0 and r4 in range(200 , 255) and g4 in range(200 ,255) and b4 in range(200 , 255) :
                    p_white_pixel_last = w4  # first white pixel in top anl left
                    p_w2 = 1
                    break
        # print("victim", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter)
        for w2 in range(64):
            for h2 in range(40):
                r = select.imageGetRed(p_image, 64 , w2 ,h2)
                g = select.imageGetGreen(p_image, 64 , w2,h2)
                b = select.imageGetBlue(p_image, 64 , w2,h2)
                if p_w == 0 and r in range(200 , 255) and g in range(200 ,255) and b in range(200 , 255) :
                    p_white_pixel = w2 
                    p_w = 1
                if (
                p_black_pixel == 0 and 
                r in range(0 , 20) and 
                g in range(0 ,20) and 
                b in range(0 , 20) 
                or 
                r in range(20 , 35) 
                and g in range(20 , 35) 
                and b in range(20 , 35)
                or 
                r in range(39 , 55) 
                and g in range(39 , 55) 
                and b in range(39 , 55)
                or 
                r in range(55 , 75) 
                and g in range(55 , 75) 
                and b in range(55 , 75)
                or 
                r in range(75 , 105) 
                and g in range(75 , 105) 
                and b in range(75 , 105)
                or
                r in range(105 , 135) 
                and g in range(105 , 135) 
                and b in range(105 , 135)
                or 
                r in range(135 , 155) 
                and g in range(135 , 155) 
                and b in range(135 , 155)):
                    p_w_last = w2 #first black pixel in top and left
                    p_black_pixel = 1
                    break
        for h in range(40):
            r2 = select.imageGetRed(p_image, 64 ,p_white_pixel ,h)
            g2 = select.imageGetGreen(p_image, 64 , p_white_pixel,h)
            b2 = select.imageGetBlue(p_image, 64 , p_white_pixel,h)
            if r2 in range(200 , 255) and g2 in range(200 ,255) and b2 in range(200 , 255) :
                first_base += 1
            
        for h in range(40):
            r2 = select.imageGetRed(p_image, 64 ,p_white_pixel_last ,h)
            g2 = select.imageGetGreen(p_image, 64 , p_white_pixel_last,h)
            b2 = select.imageGetBlue(p_image, 64 , p_white_pixel_last,h)
            if r2 in range(200 , 255) and g2 in range(200 ,255) and b2 in range(200 , 255) :
                second_base += 1

        height_trapezoid = abs(p_white_pixel_last - p_white_pixel)
        trapezoid_area = height_trapezoid* (second_base + first_base) // 2
        if trapezoid_area > 200:
            for w3 in range(63, 0  ,-1):
                for h3 in range(40):
                    r2 = select.imageGetRed(p_image, 64 , w3 ,h3)
                    g2 = select.imageGetGreen(p_image, 64 , w3,h3)
                    b2 = select.imageGetBlue(p_image, 64 , w3,h3)
                    if (
                    p_black_pixel2 == 0 and 
                    r2 in range(0 , 20) and 
                    g2 in range(0 ,20) and 
                    b2 in range(0 , 20) 
                    or 
                    r2 in range(20 , 35) 
                    and g2 in range(20 , 35) 
                    and b2 in range(20 , 35)
                    or 
                    r2 in range(39 , 55) 
                    and g2 in range(39 , 55) 
                    and b2 in range(39 , 55)
                    or 
                    r2 in range(55 , 75) 
                    and g2 in range(55 , 75) 
                    and b2 in range(55 , 75)
                    or 
                    r2 in range(75 , 105) 
                    and g2 in range(75 , 105) 
                    and b2 in range(75 , 105)
                    or 
                    r2 in range(105 , 135) 
                    and g2 in range(105 , 135) 
                    and b2 in range(105 , 135)
                    or 
                    r2 in range(135 , 155) 
                    and g2 in range(135 , 155) 
                    and b2 in range(135 , 155)):
                        p_w_first = w3 #first black pixel in top and left
                        p_black_piaxel2 = 1 
                        break
            for hu in range(40):
                for wu in range(64):
                    ru = select.imageGetRed(p_image, 64 , wu ,hu)
                    gu = select.imageGetGreen(p_image, 64 , wu,hu)
                    bu = select.imageGetBlue(p_image, 64 , wu,hu)
                    if p_wu == 0 and ru in range(200 , 255) and gu in range(200 ,255) and bu in range(200 , 255) :
                        p_white_pixel_h = hu # first white pixel in top anl left
                        p_wu = 1
                        break
            for huh in range(39  ,0 , -1):
                for wuh in range(64):
                    ruh = select.imageGetRed(p_image, 64 , wuh ,huh)
                    guh = select.imageGetGreen(p_image, 64 , wuh,huh)
                    buh = select.imageGetBlue(p_image, 64 , wuh,huh)
                    if p_wuh == 0 and ruh in range(200 , 255) and guh in range(200 ,255) and buh in range(200 , 255) :
                        p_white_pixel_h_last = huh # first white pixel in top anl left
                        p_wuh = 1
                        break
            vh = abs(p_white_pixel_h_last - p_white_pixel_h)

            for wr in range(63 , 0 , -1):
                for hr in range(40):
                    rr = select.imageGetRed(p_image, 64 , wr ,hr)
                    gr = select.imageGetGreen(p_image, 64 , wr,hr)
                    br = select.imageGetBlue(p_image, 64 , wr,hr)
                    if p_wr == 0 and rr in range(200 , 255) and gr in range(200 ,255) and br in range(200 , 255) :
                        from_right = hr
                        p_wr = 1
                        break
            for wl in range(64):
                for hl in range(40):
                    rl = select.imageGetRed(p_image, 64 , wl ,hl)
                    gl = select.imageGetGreen(p_image, 64 , wl,hl)
                    bl = select.imageGetBlue(p_image, 64 , wl,hl)
                    if p_wl == 0 and rl in range(200 , 255) and gl in range(200 ,255) and bl in range(200 , 255) :
                        from_left = hl
                        p_wl = 1
                        break
            if p_white_pixel_h == from_left:
                last_first_h = from_right
            elif p_white_pixel_h == from_right:
                last_first_h = from_left
            for wh in range(64):
                rh = select.imageGetRed(p_image, 64 , wh ,last_first_h)
                gh = select.imageGetGreen(p_image, 64 , wh,last_first_h)
                bh = select.imageGetBlue(p_image, 64 , wh,last_first_h)
                if rh in range(200 , 255) and gh in range(200 ,255) and bh in range(200 , 255) :
                    p_ww += 1
            p_middle = (p_w_first + p_w_last)// 2
            # print("m",middle)
            for h5 in range(40):
                r3 = select.imageGetRed(p_image, 64 , p_middle ,h5)
                g3 = select.imageGetGreen(p_image, 64 , p_middle,h5)
                b3 = select.imageGetBlue(p_image, 64 , p_middle,h5)
                if (
                r3 in range(0 , 20) and 
                g3 in range(0 ,20) and 
                b3 in range(0 , 20) 
                or 
                r3 in range(20 , 35) 
                and g3 in range(20 , 35) 
                and b3 in range(20 , 35)
                or 
                r3 in range(39 , 55) 
                and g3 in range(39 , 55) 
                and b3 in range(39 , 55)
                or 
                r3 in range(55 , 75) 
                and g3 in range(55 , 75) 
                and b3 in range(55 , 75)
                or 
                r3 in range(75 , 105) 
                and g3 in range(75 , 105) 
                and b3 in range(75 , 105)
                or 
                r3 in range(105 , 135) 
                and g3 in range(105 , 135) 
                and b3 in range(105 , 135)
                or 
                r3 in range(135 , 155) 
                and g3 in range(135 , 155) 
                and b3 in range(135 , 155)):
                    p_counter = p_counter + 1
                    p_x_first = h5
            d = abs(p_white_pixel_h - p_x_first)
            v = abs(p_white_pixel_last - p_white_pixel)
            print("v", v ,p_white_pixel_last, p_white_pixel )
            print("areaaaaaaaaaaaaaaaaaaaaaaaaa", trapezoid_area)
            if v > 5  :
                if 8 <p_counter<= 20 and p_vic_is_found==0 :
                    print("S detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter,  ReadLidar("R"), ReadLidar("L"))
                    send_victim(select, 'S', 30)

                if p_vic_is_found==0 and  3<p_counter <=8:
                    if vh >= 20:
                        if d >= 24:
                            print("U detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, p_white_pixel_h , p_x_first, vh)
                            send_victim(select, 'U', 32)
                        elif 7 < d < 24:
                            print("helllooooooooooo")
                            print("H detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, p_white_pixel_h ,  p_x_first,vh, p_white_pixel_h, p_white_pixel_h_last, select )
                            send_victim(select, 'H', 31)


                    elif 6<vh < 20:
                        print("S detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, last_first_h, vh , ReadLidar("R"), ReadLidar("L"))
                        send_victim(select, 'S', 30)


                if p_vic_is_found==0 and  0<p_counter <=3:
                    if d >= 24:
                        print("U detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, p_white_pixel_h , p_x_first)
                        send_victim(select, 'U', 32)


                    elif 17<= d < 23:
                        if vh > 25 :
                            print("hiiiiiiiiiiiii")
                            print("H detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, 'first',p_white_pixel_h ,  p_x_first, vh, 'last', p_white_pixel_h_last)
                            send_victim(select, 'H', 31)

                        elif 0< vh <= 25:
                            print("U detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, p_white_pixel_h , p_x_first, vh)
                            p_vic_is_found=1
                            send_victim(select, 'U', 32)

                            
                    elif 7 < d < 17:
                        print("byeeeeeeeeeeeeeeee")
                        print("H detected", "middle", p_middle ,"first",  p_w_first , "last", p_w_last,"s_count" , p_counter, "d", d, p_white_pixel_h ,  p_x_first)
                        send_victim(select, 'H', 31)
        print("middle", p_middle, "counter", p_counter, p_w_first, p_w_last, "d", d , "pwhite pixel_h", p_white_pixel_h,p_x_first, "vh", vh , p_white_pixel_h_last )
        # print(d)


def send_victim(select, victim, save_v):
    dir = dir_update()
    p_vic_is_found=1
    go(0,0)
    delay(2000)
    s_victimType = bytes(victim, "utf-8")
    s_position = gps.getValues()
    s_x = int(s_position[0] * 100)
    s_y = int(s_position[2] * 100)
    s_message = struct.pack("i i c", s_x, s_y, s_victimType) 
    emitter.send(s_message)
    # if select==Camera1:
    #     if dir==NORTH:
    #         mapData[x_map-2,y_map]=save_v
    #     if dir==SOUTH:
    #         mapData[x_map+2,y_map]=save_v
    #     if dir==WEST:
    #         mapData[x_map,y_map+2]=save_v
    #     if dir==EAST:
    #         mapData[x_map,y_map-2]=save_v
    # if select==Camera2:
    #     if dir==NORTH:
    #         mapData[x_map+2,y_map]=save_v
    #     if dir==SOUTH:
    #         mapData[x_map-2,y_map]=save_v
    #     if dir==WEST:
    #         mapData[x_map,y_map-2]=save_v
    #     if dir==EAST:
    #         mapData[x_map,y_map+2]=save_v 
    delay(1000)




def tilew(x3, y3):
    if mapData[x3, y3] == 0 or mapData[x3, y3] == 9:
        mapData[x3, y3] = 1



def maproom4():
    global check_room4
    if check_room4 == 1:
        for i in range(x_map - 2,x_map + 3):
            for j in range(y_map - 2, y_map + 3):
                mapData2[i][j] = 7


def SetWall(direction):
    dir = dir_update()
    directionResult = dir + direction
    if directionResult >= 180:
        directionResult = directionResult - 360
    elif directionResult < -180:
        directionResult = directionResult + 360

    if directionResult == NORTH_LEFT:
        tilew(x_map-1,y_map-2)
    if directionResult == NORTH:
        tilew(x_map,y_map-2)
    if directionResult == NORTH_RIGHT:
        tilew(x_map+1,y_map-2)
        
    if directionResult == NORTH_EAST:
        tilew(x_map+2,y_map-2)
    if directionResult == EAST_UP:
        tilew(x_map+2,y_map-1)
    if directionResult == EAST:
        tilew(x_map+2,y_map)
    if directionResult == EAST_DOWN:
        tilew(x_map+2,y_map+1)
    if directionResult == SOUTH_EAST:
        tilew(x_map+2,y_map+2)
        
    if directionResult == SOUTH_LEFT:
        tilew(x_map-1,y_map+2)
    if directionResult == SOUTH:
        tilew(x_map,y_map+2)
    if directionResult == SOUTH_RIGHT:
        tilew(x_map+1,y_map+2)
    
    if directionResult == NORTH_WEST:
        tilew(x_map-2,y_map-2)
    if directionResult == WEST_UP:
        tilew(x_map-2,y_map-1)
    if directionResult == WEST:
        tilew(x_map-2,y_map)
    if directionResult == WEST_DOWN:
        tilew(x_map-2,y_map+1)
    if directionResult == SOUTH_WEST:
        tilew(x_map-2,y_map+2)
        
    if directionResult == NORTHD:
        tilew(x_map,y_map-3)
    if directionResult == EASTD:
        tilew(x_map+3,y_map)
    if directionResult == SOUTHD:
        tilew(x_map,y_map+3)
    if directionResult == WESTD:
        tilew(x_map-3,y_map)
   
def SetMap():
    global x_map, y_map
    dir = dir_update()
    # x_map = int(((get_gps_x() - GPS_X_START + 15) / 30.001) + X_OFFSET)
    # y_map = int(((get_gps_y() - GPS_Y_START + 15) / 30.001) + Y_OFFSET)
    

    mapData[x_map, y_map] = 10




    
    if dir == NORTH or dir == EAST or dir == WEST or dir == SOUTH:
        if ReadLidar("F") <= 65:
              SetWall(NORTH)

        if ReadLidar("R") <= 65:
            SetWall(EAST)

        if ReadLidar("B") <= 65:
            SetWall(SOUTH)     

        if ReadLidar("L") <= 65:
              SetWall(WEST)

        #if 60 <= ReadLidar("F") <= 90:
        #      SetWall(NORTHD)
        #  
        #if 60 <= ReadLidar("R") <= 90:
        #    SetWall(EASTD)
        #
        #if 60 <= ReadLidar("B") <= 90:
        #    SetWall(SOUTHD)     
        #
        #if 60 <= ReadLidar("L") <= 90:
        #      SetWall(WESTD)



        if  60 <= ReadLidar("L45") <= (60/cos(pi/4)):
            SetWall(NORTH_WEST)

        if 50 <= ReadLidar("FL") <= (60/cos(pi/8)+8):
            SetWall(NORTH_LEFT)
    
        if 50 <= ReadLidar("LF") <= (60/cos(pi/8)+8):
            SetWall(WEST_UP)



        if  60 <= ReadLidar("R45") <= (60/cos(pi/4)):
            SetWall(NORTH_EAST)

        if 50 <= ReadLidar("RF") <= (60/cos(pi/8)+8):
            SetWall(EAST_UP)
    
        if 50 <= ReadLidar("FR") <= ((60/cos(pi/8))+8):
            SetWall(NORTH_RIGHT)



        if  60 <= ReadLidar("R135") <= (60/cos(pi/4)):
               SetWall(SOUTH_EAST)

        if 50 <= ReadLidar("RB") <= (60/cos(pi/8)+8):
            SetWall(EAST_DOWN)

        if 50 <= ReadLidar("BR") <= (60/cos(pi/8)+8):
            SetWall(SOUTH_RIGHT)



        if  60 <= ReadLidar("L135") <= (60/cos(pi/4)):
            SetWall(SOUTH_WEST)

        if 50 <= ReadLidar("LB") <= (60/cos(pi/8)+8):
            SetWall(WEST_DOWN)

        if 50 <= ReadLidar("BL") <= (60/cos(pi/8)+8):
            SetWall(SOUTH_LEFT)
    elif dir == NORTH_EAST or dir == SOUTH_EAST or dir == NORTH_WEST or dir == SOUTH_WEST:
        if 60 <= ReadLidar("F") <= (60/cos(pi/4)):
              SetWall(NORTH)

        if 60 <= ReadLidar("R") <= (60/cos(pi/4)):
            SetWall(EAST)

        if 60 <= ReadLidar("B") <= (60/cos(pi/4)):
            SetWall(SOUTH)     

        if 60 <= ReadLidar("L") <= (60/cos(pi/4)):
              SetWall(WEST)

        #if (60/cos(pi/4)) <= ReadLidar("F") <= 90:
        #      SetWall(NORTHD)
        #  
        #if (60/cos(pi/4)) <= ReadLidar("R") <= 90:
        #    SetWall(EASTD)
        #
        #if (60/cos(pi/4)) <= ReadLidar("B") <= 90:
        #    SetWall(SOUTHD)     
        #
        #if (60/cos(pi/4)) <= ReadLidar("L") <= 90:
        #      SetWall(WESTD)



        if ReadLidar("L45") <= 65:
            SetWall(NORTH_WEST)

        if 50 <= ReadLidar("FL") <= (60/cos(pi/8)+8):
            SetWall(NORTH_LEFT)
    
        if 50 <= ReadLidar("LF") <= (60/cos(pi/8)+8):
            SetWall(WEST_UP)



        if ReadLidar("R45") <= 65:
            SetWall(NORTH_EAST)

        if 50 <= ReadLidar("RF") <= (60/cos(pi/8)+8):
            SetWall(EAST_UP)
    
        if 50 <= ReadLidar("FR") <= ((60/cos(pi/8))+8):
            SetWall(NORTH_RIGHT)



        if ReadLidar("R135") <= 65:
               SetWall(SOUTH_EAST)

        if 50 <= ReadLidar("RB") <= (60/cos(pi/8)+8):
            SetWall(EAST_DOWN)

        if 50 <= ReadLidar("BR") <= (60/cos(pi/8)+8):
            SetWall(SOUTH_RIGHT)



        if ReadLidar("L135") <= 65:
            SetWall(SOUTH_WEST)

        if 50 <= ReadLidar("LB") <= (60/cos(pi/8)+8):
            SetWall(WEST_DOWN)

        if 50 <= ReadLidar("BL") <= (60/cos(pi/8)+8):
            SetWall(SOUTH_LEFT)
 

    block_repeated_tile(dir)

def printo():
    global x_map, y_map, xmax, ymax, xmin, ymin

    for y2 in range(ymin - 2, ymax + 3):
        print("\n")
        for x2 in range(xmin - 2, xmax + 3):
            if x2 == x_map and y2 == y_map:
                print(RED_COLOR, "=", end="  ")
            elif mapData[x2, y2] == 11:
                print(GREY_COLOR, "x", end="  ")
            elif mapData[x2, y2] == 0:
                print(GREY_COLOR, ".", end="  ")
            else:
                print(RESET_COLOR, "%2d" % int(mapData[x2, y2]), end=" ")
    print("\n")
    print("===========================================================================")
    print("\n")


def Tile_set(x5, y5, color):
    mapData[x5 - 1, y5 - 1] = color
    mapData[x5 - 1, y5 + 1] = color
    mapData[x5 + 1, y5 - 1] = color
    mapData[x5 + 1, y5 + 1] = color
    # dir = dir_update()
    # if dir == NORTH:
    #     mapData[x5-1,y5-3]=color
    #     mapData[x5-1,y5-5]=color
    #     mapData[x5+1,y5-3]=color
    #     mapData[x5+1,y5-5]=color
    # if dir == EAST:
    #     mapData[x5+3,y5-1]=color
    #     mapData[x5+5,y5-1]=color
    #     mapData[x5+3,y5+1]=color
    #     mapData[x5+5,y5+1]=color
    # if dir == WEST:
    #     mapData[x5-3,y5-1]=color
    #     mapData[x5-5,y5-1]=color
    #     mapData[x5-3,y5+1]=color
    #     mapData[x5-5,y5+1]=color
    # if dir == SOUTH:
    #     mapData[x5-1,y5+3]=color
    #     mapData[x5-1,y5+5]=color
    #     mapData[x5+1,y5+3]=color
    #     mapData[x5+1,y5+5]=color

def Tile_setB(x6,y6,color):
    dir = dir_update()
    if (dir == NORTH) or (dir == NORTH_EAST):
        mapData[x6-1,y6-3]=color
        mapData[x6-1,y6-4]=color
        mapData[x6-1,y6-5]=color
        mapData[x6,y6-3]=color
        mapData[x6,y6-4]=color
        mapData[x6,y6-5]=color
        mapData[x6+1,y6-3]=color
        mapData[x6+1,y6-4]=color
        mapData[x6+1,y6-5]=color
        if (mapData[x6-2,y6-2]==0):
            mapData[x6-2,y6-2]=9
        if (mapData[x6+2,y6-2]==0):
            mapData[x6+2,y6-2]=9
        if (mapData[x6-2,y6+2]==0):
            mapData[x6-2,y6+2]=9
        if (mapData[x6+2,y6+2]==0):
            mapData[x6+2,y6+2]=9
    if (dir == EAST) or (dir == SOUTH_EAST):
        mapData[x6+3,y6-1]=color
        mapData[x6+4,y6-1]=color
        mapData[x6+5,y6-1]=color
        mapData[x6+3,y6]=color
        mapData[x6+4,y6]=color
        mapData[x6+5,y6]=color
        mapData[x6+3,y6+1]=color
        mapData[x6+4,y6+1]=color
        mapData[x6+5,y6+1]=color
        if (mapData[x6-2,y6-2]==0):
            mapData[x6-2,y6-2]=9
        if (mapData[x6+2,y6-2]==0):
            mapData[x6+2,y6-2]=9
        if (mapData[x6-2,y6+2]==0):
            mapData[x6-2,y6+2]=9
        if (mapData[x6+2,y6+2]==0):
            mapData[x6+2,y6+2]=9
    if (dir == WEST) or (dir == NORTH_WEST):
        mapData[x6-3,y6-1]=color
        mapData[x6-4,y6-1]=color
        mapData[x6-5,y6-1]=color
        mapData[x6-3,y6]=color
        mapData[x6-4,y6]=color
        mapData[x6-5,y6]=color
        mapData[x6-3,y6+1]=color
        mapData[x6-4,y6+1]=color
        mapData[x6-5,y6+1]=color
        if (mapData[x6-2,y6-2]==0):
            mapData[x6-2,y6-2]=9
        if (mapData[x6+2,y6-2]==0):
            mapData[x6+2,y6-2]=9
        if (mapData[x6-2,y6+2]==0):
            mapData[x6-2,y6+2]=9
        if (mapData[x6+2,y6+2]==0):
            mapData[x6+2,y6+2]=9
    if (dir == SOUTH) or (dir == SOUTH_WEST):
        mapData[x6-1,y6+3]=color
        mapData[x6-1,y6+4]=color
        mapData[x6-1,y6+5]=color
        mapData[x6,y6+3]=color
        mapData[x6,y6+4]=color
        mapData[x6,y6+5]=color
        mapData[x6+1,y6+3]=color
        mapData[x6+1,y6+4]=color
        mapData[x6+1,y6+5]=color
        if (mapData[x6-2,y6-2]==0):
            mapData[x6-2,y6-2]=9
        if (mapData[x6+2,y6-2]==0):
            mapData[x6+2,y6-2]=9
        if (mapData[x6-2,y6+2]==0):
            mapData[x6-2,y6+2]=9
        if (mapData[x6+2,y6+2]==0):
            mapData[x6+2,y6+2]=9
 

def Tile_setBR(x8,y8,color):
    dir = dir_update()
    if (dir == NORTH) or (dir == NORTH_EAST):
        mapData[x8+1,y8-3] = color
        mapData[x8+1,y8-4] = color
        mapData[x8+1,y8-5] = color
        mapData[x8+2,y8-3] = color
        mapData[x8+2,y8-4] = color
        mapData[x8+2,y8-5] = color
        mapData[x8+3,y8-3] = color
        mapData[x8+3,y8-4] = color
        mapData[x8+3,y8-5] = color
        if (mapData[x8-2,y8-2]==0):
            mapData[x8-2,y8-2]=9
        if (mapData[x8+2,y8-2]==0):
            mapData[x8+2,y8-2]=9
        if (mapData[x8-2,y8+2]==0):
            mapData[x8-2,y8+2]=9
        if (mapData[x8+2,y8+2]==0):
            mapData[x8+2,y8+2]=9
    if (dir == EAST) or (dir == SOUTH_EAST):
        mapData[x8+3,y8+1] = color
        mapData[x8+4,y8+1] = color
        mapData[x8+5,y8+1] = color
        mapData[x8+3,y8+2] = color
        mapData[x8+4,y8+2] = color
        mapData[x8+5,y8+2] = color
        mapData[x8+3,y8+3] = color
        mapData[x8+4,y8+3] = color
        mapData[x8+5,y8+3] = color
        if (mapData[x8-2,y8-2]==0):
            mapData[x8-2,y8-2]=9
        if (mapData[x8+2,y8-2]==0):
            mapData[x8+2,y8-2]=9
        if (mapData[x8-2,y8+2]==0):
            mapData[x8-2,y8+2]=9
        if (mapData[x8+2,y8+2]==0):
            mapData[x8+2,y8+2]=9
    if (dir == WEST) or (dir == NORTH_WEST):
        mapData[x8-3,y8-1] = color
        mapData[x8-4,y8-1] = color
        mapData[x8-5,y8-1] = color
        mapData[x8-3,y8-2] = color
        mapData[x8-4,y8-2] = color
        mapData[x8-5,y8-2] = color
        mapData[x8-3,y8-3] = color
        mapData[x8-4,y8-3] = color
        mapData[x8-5,y8-3] = color
        if (mapData[x8-2,y8-2]==0):
            mapData[x8-2,y8-2]=9
        if (mapData[x8+2,y8-2]==0):
            mapData[x8+2,y8-2]=9
        if (mapData[x8-2,y8+2]==0):
            mapData[x8-2,y8+2]=9
        if (mapData[x8+2,y8+2]==0):
            mapData[x8+2,y8+2]=9
    if (dir == SOUTH) or (dir == SOUTH_WEST):
        mapData[x8-1,y8+3] = color
        mapData[x8-1,y8+4] = color
        mapData[x8-1,y8+5] = color
        mapData[x8-2,y8+3] = color
        mapData[x8-2,y8+4] = color
        mapData[x8-2,y8+5] = color
        mapData[x8-3,y8+3] = color
        mapData[x8-3,y8+4] = color
        mapData[x8-3,y8+5] = color
        if (mapData[x8-2,y8-2]==0):
            mapData[x8-2,y8-2]=9
        if (mapData[x8+2,y8-2]==0):
            mapData[x8+2,y8-2]=9
        if (mapData[x8-2,y8+2]==0):
            mapData[x8-2,y8+2]=9
        if (mapData[x8+2,y8+2]==0):
            mapData[x8+2,y8+2]=9
     


def Tile_setBL(x9,y9,color):
    dir = dir_update()
    if (dir == NORTH) or (dir == NORTH_WEST):
        mapData[x9-1,y9-3] = color
        mapData[x9-1,y9-4] = color
        mapData[x9-1,y9-5] = color
        mapData[x9-2,y9-3] = color
        mapData[x9-2,y9-4] = color
        mapData[x9-2,y9-5] = color
        mapData[x9-3,y9-3] = color
        mapData[x9-3,y9-4] = color
        mapData[x9-3,y9-5] = color
        if (mapData[x9-2,y9-2]==0):
            mapData[x9-2,y9-2]=9
        if (mapData[x9+2,y9-2]==0):
            mapData[x9+2,y9-2]=9
        if (mapData[x9-2,y9+2]==0):
            mapData[x9-2,y9+2]=9
        if (mapData[x9+2,y9+2]==0):
            mapData[x9+2,y9+2]=9
    if (dir == EAST) or (dir == NORTH_EAST):
        mapData[x9+3,y9-1] = color
        mapData[x9+4,y9-1] = color
        mapData[x9+5,y9-1] = color
        mapData[x9+3,y9-2] = color
        mapData[x9+4,y9-2] = color
        mapData[x9+5,y9-2] = color
        mapData[x9+3,y9-3] = color
        mapData[x9+4,y9-3] = color
        mapData[x9+5,y9-3] = color
        if (mapData[x9-2,y9-2]==0):
            mapData[x9-2,y9-2]=9
        if (mapData[x9+2,y9-2]==0):
            mapData[x9+2,y9-2]=9
        if (mapData[x9-2,y9+2]==0):
            mapData[x9-2,y9+2]=9
        if (mapData[x9+2,y9+2]==0):
            mapData[x9+2,y9+2]=9
    if (dir == WEST) or (dir == SOUTH_WEST):
        mapData[x9-3,y9+1] = color
        mapData[x9-4,y9+1] = color
        mapData[x9-5,y9+1] = color
        mapData[x9-3,y9+2] = color
        mapData[x9-4,y9+2] = color
        mapData[x9-5,y9+2] = color
        mapData[x9-3,y9+3] = color
        mapData[x9-4,y9+3] = color
        mapData[x9-5,y9+3] = color
        if (mapData[x9-2,y9-2]==0):
            mapData[x9-2,y9-2]=9
        if (mapData[x9+2,y9-2]==0):
            mapData[x9+2,y9-2]=9
        if (mapData[x9-2,y9+2]==0):
            mapData[x9-2,y9+2]=9
        if (mapData[x9+2,y9+2]==0):
            mapData[x9+2,y9+2]=9
    if (dir == SOUTH) or (dir == SOUTH_EAST):
        mapData[x9+1,y9+3] = color
        mapData[x9+1,y9+4] = color
        mapData[x9+1,y9+5] = color
        mapData[x9+2,y9+3] = color
        mapData[x9+2,y9+4] = color
        mapData[x9+2,y9+5] = color
        mapData[x9+3,y9+3] = color
        mapData[x9+3,y9+4] = color
        mapData[x9+3,y9+5] = color
        if (mapData[x9-2,y9-2]==0):
            mapData[x9-2,y9-2]=9
        if (mapData[x9+2,y9-2]==0):
            mapData[x9+2,y9-2]=9
        if (mapData[x9-2,y9+2]==0):
            mapData[x9-2,y9+2]=9
        if (mapData[x9+2,y9+2]==0):
            mapData[x9+2,y9+2]=9


def Tile_setB2(x10,y10,color):
    dir = dir_update()
    if (dir == NORTH) or (dir == NORTH_WEST):
        mapData[x10-1,y10-3]=color
        mapData[x10-1,y10-4]=color
        mapData[x10-1,y10-5]=color
        mapData[x10,y10-3]=color
        mapData[x10,y10-4]=color
        mapData[x10,y10-5]=color
        mapData[x10+1,y10-3]=color
        mapData[x10+1,y10-4]=color
        mapData[x10+1,y10-5]=color
        if (mapData[x10-2,y10-2]==0):
            mapData[x10-2,y10-2]=9
        if (mapData[x10+2,y10-2]==0):
            mapData[x10+2,y10-2]=9
        if (mapData[x10-2,y10+2]==0):
            mapData[x10-2,y10+2]=9
        if (mapData[x10+2,y10+2]==0):
            mapData[x10+2,y10+2]=9
    if (dir == EAST) or (dir == NORTH_EAST):
        mapData[x10+3,y10-1]=color
        mapData[x10+4,y10-1]=color
        mapData[x10+5,y10-1]=color
        mapData[x10+3,y10]=color
        mapData[x10+4,y10]=color
        mapData[x10+5,y10]=color
        mapData[x10+3,y10+1]=color
        mapData[x10+4,y10+1]=color
        mapData[x10+5,y10+1]=color
        if (mapData[x10-2,y10-2]==0):
            mapData[x10-2,y10-2]=9
        if (mapData[x10+2,y10-2]==0):
            mapData[x10+2,y10-2]=9
        if (mapData[x10-2,y10+2]==0):
            mapData[x10-2,y10+2]=9
        if (mapData[x10+2,y10+2]==0):
            mapData[x10+2,y10+2]=9
    if (dir == WEST) or (dir == SOUTH_WEST):
        mapData[x10-3,y10-1]=color
        mapData[x10-4,y10-1]=color
        mapData[x10-5,y10-1]=color
        mapData[x10-3,y10]=color
        mapData[x10-4,y10]=color
        mapData[x10-5,y10]=color
        mapData[x10-3,y10+1]=color
        mapData[x10-4,y10+1]=color
        mapData[x10-5,y10+1]=color
        if (mapData[x10-2,y10-2]==0):
            mapData[x10-2,y10-2]=9
        if (mapData[x10+2,y10-2]==0):
            mapData[x10+2,y10-2]=9
        if (mapData[x10-2,y10+2]==0):
            mapData[x10-2,y10+2]=9
        if (mapData[x10+2,y10+2]==0):
            mapData[x10+2,y10+2]=9
    if (dir == SOUTH) or (dir == SOUTH_EAST):
        mapData[x10-1,y10+3]=color
        mapData[x10-1,y10+4]=color
        mapData[x10-1,y10+5]=color
        mapData[x10,y10+3]=color
        mapData[x10,y10+4]=color
        mapData[x10,y10+5]=color
        mapData[x10+1,y10+3]=color
        mapData[x10+1,y10+4]=color
        mapData[x10+1,y10+5]=color
        if (mapData[x10-2,y10-2]==0):
            mapData[x10-2,y10-2]=9
        if (mapData[x10+2,y10-2]==0):
            mapData[x10+2,y10-2]=9
        if (mapData[x10-2,y10+2]==0):
            mapData[x10-2,y10+2]=9
        if (mapData[x10+2,y10+2]==0):
            mapData[x10+2,y10+2]=9

def setblackhole():
    global BlackHoleFound
    dir = dir_update()
    dif_x = abs(x_map - 100) 
    dif_y = abs(y_map - 100)
    DM = 1000*dis_sensor1.getValue()
    DR = 1000*dis_sensor2.getValue()
    DL = 1000*dis_sensor3.getValue()
    DR45 = 1000*dis_sensor4.getValue()
    DL45 = 1000*dis_sensor5.getValue()


    DM_100 = False
    DR_100 = False
    DL_100 = False
    DR45_100 = False
    DL45_100 = False
    
    DM_200 = False
    DR_200 = False
    DL_200 = False
    DR45_200 = False
    DL45_200 = False
    
    DM_300 = False
    DR_300 = False
    DL_300 = False
    DR45_300 = False
    DL45_300 = False
    
    DM_400 = False
    DR_400 = False
    DL_400 = False
    DR45_400 = False
    DL45_400 = False
    
    DM_500 = False
    DR_500 = False
    DL_500 = False
    DR45_500 = False
    DL45_500 = False
    
    DM_600 = False
    DR_600 = False
    DL_600 = False
    DR45_600 = False
    DL45_600 = False
    
    DM_700 = False
    DR_700 = False
    DL_700 = False
    DR45_700 = False
    DL45_700 = False
    
    if DM > 699:
        DM_700 = True
    if DM > 599:
        DM_600 = True
    if DM > 499:
        DM_500 = True
    if DM > 399:
        DM_400 = True
    if DM > 299:
        DM_300 = True
    if DM > 199:
        DM_200 = True
    if DM > 99:
        DM_100 = True
    
    
    
    if DR > 699:
        DR_700 = True
    if DR > 599:
        DR_600 = True
    if DR > 499:
        DR_500 = True
    if DR > 399:
        DR_400 = True
    if DR > 299:
        DR_300 = True
    if DR > 199:
        DR_200 = True
    if DR > 99:
        DR_100 = True
        
    if DL > 699:
        DL_700 = True
    if DL > 599:
        DL_600 = True
    if DL > 499:
        DL_500 = True
    if DL > 399:
        DL_400 = True
    if DL > 299:
        DL_300 = True
    if DL > 199:
        DL_200 = True
    if DL > 99:
        DL_100 = True
        
    if DR45 > 699:
        DR45_700 = True
    if DR45 > 599:
        DR45_600 = True
    if DR45 > 499:
        DR45_500 = True
    if DR45 > 399:
        DR45_400 = True
    if DR45 > 299:
        DR45_300 = True
    if DR45 > 199:
        DR45_200 = True
    if DR45 > 99:
        DR45_100 = True
        
    if DL45 > 699:
        DL45_500 = True 
    if DL45 > 599:
        DL45_600 = True   
    if DL45 > 499:
        DL45_500 = True
    if DL45 > 399:
        DL45_400 = True
    if DL45 > 299:
        DL45_300 = True
    if DL45 > 199:
        DL45_200 = True
    if DL45 > 99:
        DL45_100 = True
        
    #__________________________________________________90darage__________________________________________________       
    
    if DM_600 == True and DR_500 == True and DL_500 == True and DR45_300 == True and DL45_300 == True:
        if dif_x %4 == 0:
            if dif_y %4 == 0:
                if dir == NORTH or dir == SOUTH or dir == EAST or dir == WEST:
                    Tile_setB(x_map, y_map, 2)
                    BlackHoleFound = 1
                    
    if DM_100 == True and DR_600 == True and DL_100 == True and DR45_300 == True and DL45_100 == True:
        if dir == NORTH:
            if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:
                    Tile_setBR(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == EAST:
            if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    Tile_setBR(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == WEST:
            if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    Tile_setBR(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == SOUTH:
            if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:
                    Tile_setBR(x_map, y_map, 2)
                    BlackHoleFound = 1
                    
    if DM_100 == True and DR_100 == True and DL_600 == True and DR45_100 == True and DL45_300 == True:
        if dir == NORTH:
            if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:
                    Tile_setBL(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == EAST:
            if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    Tile_setBL(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == WEST:
            if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    Tile_setBL(x_map, y_map, 2)
                    BlackHoleFound = 1
        if dir == SOUTH:
            if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:
                    Tile_setBL(x_map, y_map, 2)
                    BlackHoleFound = 1
            
    #__________________________________________________45darage__________________________________________________
    
    if DM_400 == True and DR_100 == True and DL_100 == True and DR45_100 == True and DL45_100 == True:
        if dif_x %4 == 0:
            if dif_y %4 == 0:
                if dir == NORTH_EAST:
                    mapData[x_map+3,y_map-3] = 2
                    mapData[x_map+3,y_map-4] = 2
                    mapData[x_map+3,y_map-5] = 2
                    mapData[x_map+4,y_map-3] = 2
                    mapData[x_map+4,y_map-4] = 2
                    mapData[x_map+4,y_map-5] = 2
                    mapData[x_map+5,y_map-3] = 2
                    mapData[x_map+5,y_map-4] = 2
                    mapData[x_map+5,y_map-5] = 2
                    if (mapData[x_map-2,y_map-2]==0):
                        mapData[x_map-2,y_map-2]=9
                    if (mapData[x_map+2,y_map-2]==0):
                        mapData[x_map+2,y_map-2]=9
                    if (mapData[x_map-2,y_map+2]==0):
                        mapData[x_map-2,y_map+2]=9
                    if (mapData[x_map+2,y_map+2]==0):
                        mapData[x_map+2, y_map+2]=9
                    BlackHoleFound = 1
                if dir == SOUTH_EAST:
                    mapData[x_map+3,y_map+3] = 2
                    mapData[x_map+4,y_map+3] = 2
                    mapData[x_map+5,y_map+3] = 2
                    mapData[x_map+3,y_map+4] = 2
                    mapData[x_map+4,y_map+4] = 2
                    mapData[x_map+5,y_map+4] = 2
                    mapData[x_map+3,y_map+5] = 2
                    mapData[x_map+4,y_map+5] = 2
                    mapData[x_map+5,y_map+5] = 2
                    if (mapData[x_map-2,y_map-2]==0):
                        mapData[x_map-2,y_map-2]=9
                    if (mapData[x_map+2,y_map-2]==0):
                        mapData[x_map+2,y_map-2]=9
                    if (mapData[x_map-2,y_map+2]==0):
                        mapData[x_map-2,y_map+2]=9
                    if (mapData[x_map+2,y_map+2]==0):
                        mapData[x_map+2,y_map+2]=9
                    BlackHoleFound = 1
                if dir == NORTH_WEST:
                    mapData[x_map-3,y_map-3] = 2
                    mapData[x_map-4,y_map-3] = 2
                    mapData[x_map-5,y_map-3] = 2
                    mapData[x_map-3,y_map-4] = 2
                    mapData[x_map-4,y_map-4] = 2
                    mapData[x_map-5,y_map-4] = 2
                    mapData[x_map-3,y_map-5] = 2
                    mapData[x_map-4,y_map-5] = 2
                    mapData[x_map-5,y_map-5] = 2
                    if (mapData[x_map-2,y_map-2]==0):
                        mapData[x_map-2,y_map-2]=9
                    if (mapData[x_map+2,y_map-2]==0):
                        mapData[x_map+2,y_map-2]=9
                    if (mapData[x_map-2,y_map+2]==0):
                        mapData[x_map-2,y_map+2]=9
                    if (mapData[x_map+2,y_map+2]==0):
                        mapData[x_map+2,y_map+2]=9
                    BlackHoleFound = 1
                if dir == SOUTH_WEST:
                    mapData[x_map-3,y_map+3] = 2
                    mapData[x_map-3,y_map+4] = 2
                    mapData[x_map-3,y_map+5] = 2
                    mapData[x_map-4,y_map+3] = 2
                    mapData[x_map-4,y_map+4] = 2
                    mapData[x_map-4,y_map+5] = 2
                    mapData[x_map-5,y_map+3] = 2
                    mapData[x_map-5,y_map+4] = 2
                    mapData[x_map-5,y_map+5] = 2
                    if (mapData[x_map-2,y_map-2]==0):
                        mapData[x_map-2,y_map-2]=9
                    if (mapData[x_map+2,y_map-2]==0):
                        mapData[x_map+2,y_map-2]=9
                    if (mapData[x_map-2,y_map+2]==0):
                        mapData[x_map-2,y_map+2]=9
                    if (mapData[x_map+2,y_map+2]==0):
                        mapData[x_map+2,y_map+2]=9
                    BlackHoleFound = 1  
    
    
    
    if (dir == NORTH_EAST) or (dir == SOUTH_EAST) or (dir == NORTH_WEST) or (dir == SOUTH_WEST):
        if dif_x %4 == 0:
            if dif_y %4 == 0:
                if DM_100 == True and DR_400 == True and DL_100 == True and DR45_300 == True and DL45_100 == True:
                    Tile_setB2(x_map, y_map, 2)
                    BlackHoleFound = 1
                if DM_100 == True and DR_100 == True and DL_400 == True and DR45_100 == True and DL45_300 == True:
                    Tile_setB(x_map, y_map, 2)
                    BlackHoleFound = 1  
                    
        if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:               
                    if DM_400 == True and DR_700 == True and DL_100 == True and DR45_300 == True and DL45_100 == True:
                        Tile_setBL(x_map, y_map, 2)
                        BlackHoleFound = 1
        if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    if DM_400 == True and DR_700 == True and DL_100 == True and DR45_300 == True and DL45_100 == True:
                        Tile_setBL(x_map, y_map, 2)
                        BlackHoleFound = 1
        if (dif_x+2) %4 == 0:
                if dif_y %4 == 0:    
                    if DM_500 == True and DR_100 == True and DL_400 == True and DR45_100 == True and DL45_300 == True:
                        Tile_setBR(x_map, y_map, 2)
                        BlackHoleFound = 1
        if dif_x %4 == 0:
                if (dif_y+2) %4 == 0:
                    if DM_500 == True and DR_100 == True and DL_400 == True and DR45_100 == True and DL45_300 == True:
                        Tile_setBR(x_map, y_map, 2)
                        BlackHoleFound = 1
    



def Tile_setS(x7, y7, color):
    mapData[x7 - 1, y7 - 1] = color
    mapData[x7 - 1, y7] = color
    mapData[x7 - 1, y7 + 1] = color
    mapData[x7, y7 - 1] = color
    mapData[x7, y7] = color
    mapData[x7, y7 + 1] = color
    mapData[x7 + 1, y7 - 1] = color
    mapData[x7 + 1, y7] = color
    mapData[x7 + 1, y7 + 1] = color


def setcolor():
    global room4_found
    dir = dir_update()
    x_dif = abs(x_map - 100)
    y_dif = abs(y_map - 100)
    if (x_dif % 4) == 0:
        if (y_dif % 4) == 0:
            # if dir == NORTH or dir == EAST or dir == WEST or dir == SOUTH:
            if color() == RED:
                Tile_set(x_map, y_map, 82)
                room4_found = 1
            elif color() == GREEN:
                Tile_set(x_map, y_map, 71)
                room4_found = 1
            elif color() == BLUE:
                Tile_set(x_map, y_map, 66)
            elif color() == PURPLE:
                Tile_set(x_map, y_map, 80)
            elif color() == YELLOW:
                Tile_set(x_map, y_map, 89)
            elif color() == ORANGE:
                Tile_set(x_map, y_map, 79)
                room4_found = 1
            elif color() == GRAY:
                Tile_set(x_map, y_map, 4)
            elif color() == KAKI:
                Tile_set(x_map, y_map, 3)

def PrintMap():
    for yn in range(ymin - 7, ymax + 7):
        for xn in range(xmin - 7, xmax + 7):
            if xn == x_map and yn == y_map:
                print(YELLOW_COLOR, " = ", end="")  # yellow
            elif xn == 100 and yn == 100:
                print(CYAN_COLOR, " - ", end="")  # purple
            elif mapData[xn][yn] == 0:
                print(CYAN_COLOR, " . ", end="")  # blue
            elif mapData[xn][yn] == 10:
                print(GREEN_COLOR, " 9 ", end="")  # green
            elif mapData[xn][yn] == 1:
                print(RED_COLOR, " 1 ", end="")  # red
            elif mapData[xn][yn] == -1:
                print(" = ", end="")
            else:
                print(f"{int(mapData[xn][yn])}", end="  ")
        print("\n")
    print("\n")
    print(RESET_COLOR, "\n")


def turn(degree):  # 45, 90, 135, -45, -90, -135, 180
    # global dir
    print("TURN STARRRRRTTTTTEEEEEDDDDDD")
    global reset_flag
    global p_vic_is_found
    # dir = dir_update()
    yaw = get_yaw()
    des_yaw = degree + yaw


    if des_yaw > 180:
        des_yaw = des_yaw - 360
    elif des_yaw < -180:
        des_yaw = des_yaw + 360
    e_yaw = des_yaw - yaw
    if e_yaw > 180:
        e_yaw = e_yaw - 360

    elif e_yaw < -180:
        e_yaw = e_yaw + 360
    p_vic_is_found = 0
    while abs(e_yaw) > 0.1 and robot.step(timeStep) != -1:
        ##lop_called()
        if reset_flag == 1:
            break


        if ReadLidar("L") <= 90 and p_vic_is_found == 0:
            p_find(Camera1)  # left
        if ReadLidar("R") <= 90 and p_vic_is_found == 0:
            p_find(Camera2)  # right

        yaw = get_yaw()
        e_yaw = des_yaw - yaw

        if e_yaw > 180:
            e_yaw = e_yaw - 360

        elif e_yaw < -180:
            e_yaw = e_yaw + 360

        k_yaw = 4
        v = k_yaw * e_yaw

        go(v, -v)

    go(0, 0)
    robot.step(timeStep)


def move_back(des_x, des_y):
    #    update_coordinate()
    # global dir
    global p_vic_is_found

    K_DIS = 8


    dir = dir_update()

    #    des_x = gps_x_current + (des_x - 100) * 6
    gps_x_current = get_gps_x()
    gps_y_current = get_gps_y()

    delta_x = des_x - gps_x_current
    delta_y = des_y - gps_y_current

    e_distance = sqrt((delta_x) ** 2 + (delta_y) ** 2)  # faseleye fisaghooresi

    p_vic_is_found = 0
    while abs(e_distance) > 3 and robot.step(timeStep) != -1:

        #   update_coordinate()
        #   get_xy_map()

        gps_x_current = get_gps_x()
        gps_y_current = get_gps_y()
        delta_x = des_x - gps_x_current
        delta_y = des_y - gps_y_current

        e_distance = sqrt((delta_x) ** 2 + (delta_y) ** 2)

        v_distance = e_distance * K_DIS

        dest_angle = atan(delta_x / (delta_y + 0.000001))
        dest_angle = dest_angle * 180 / pi  # tabdil radian be darajeh



        if delta_x < 0 and delta_y > 0:
            dest_angle = -180 - dest_angle
        elif delta_x <= 0 and delta_y >= 0:
            dest_angle = -180 - dest_angle
        elif delta_x > 0 and delta_y >= 0:
            dest_angle = 180 - dest_angle
        elif delta_x > 0 and delta_y < 0:
            dest_angle = dest_angle + 90
        elif delta_x < 0 and delta_y < 0:
            dest_angle = dest_angle - 90
        elif delta_x == 0 and delta_y < 0:
            dest_angle = dest_angle

        e_angle = dest_angle - get_yaw()

        if e_angle > 180:
            e_angle = e_angle - 360
        elif e_angle < -180:
            e_angle = e_angle + 360

        k_angle = 2
        v_angle = e_angle * k_angle


        # print("delta_x = ", delta_x, end="")
        # print("delta_y: ", delta_y, end="")
        # print("e_distance: ", e_distance, end="")
        # print("v_distance: ", v_distance)

        # print("dest_angle: ", dest_angle, end="")
        # print("error_angle: ", e_angle, end="")
        # print("v_angle: ", v_angle, end="")
        # print("v_dist + v_angle= ", v_distance + v_angle, end="")
        # print("v_dist - v_angle= ", v_distance - v_angle)

        go(-v_distance - v_angle, -v_distance + v_angle)

        # go(v_distance* (1 + k_move), v_distance * (1 - k_move))

        # print("v=", v_distance)
    # print(f"diiiiirection:{dir}")
    go(0, 0)
    robot.step(timeStep)
    print("ROBOT STOPPPPPPPPEEDDDDDD!  ")
    # print(f"e_x={delta_x}, e_y={delta_y}, e_distance={e_distance} , yaw: {get_yaw()}")
    correct_angle(dir)


def forward():
    #    update_coordinate()
    # global dir
    global reset_flag
    global p_vic_is_found
    flag1 = 0
    K_DIS = 12 # 8
    print(f"FORWARD STARTED: x= {x_map} y= {y_map}\n")

    gps_init_x = get_gps_x()
    gps_init_y = get_gps_y()
    dir = dir_update()

    if dir == NORTH:
        # des_x = get_gps_x()
        # des_y = (-2 * 30) + get_gps_y() # /2 * 6 => har do ta khooneh 6cm hast

        des_x_map = x_map
        des_y_map = y_map - 2

    elif dir == EAST:
        # des_x = (2 * 30) + get_gps_x()
        # des_y = get_gps_y() # /2 * 6 => har do ta khooneh 6cm hast

        des_x_map = x_map + 2
        des_y_map = y_map

    elif dir == SOUTH:
        # des_x = get_gps_x()
        # des_y = (2 * 30) + get_gps_y() # /2 * 6 => har do ta khooneh 6cm hast

        des_x_map = x_map
        des_y_map = y_map + 2

    elif dir == WEST:
        # des_x = (-2 * 30) + get_gps_x()
        # des_y = get_gps_y() # /2 * 6 => har do ta khooneh 6cm hast
        des_x_map = x_map - 2
        des_y_map = y_map

    elif dir == NORTH_EAST:

        des_x_map = x_map + 2
        des_y_map = y_map - 2

    elif dir == NORTH_WEST:

        des_x_map = x_map - 2
        des_y_map = y_map - 2

    elif dir == SOUTH_EAST:

        des_x_map = x_map + 2
        des_y_map = y_map + 2

    elif dir == SOUTH_WEST:

        des_x_map = x_map - 2
        des_y_map = y_map + 2

    #    des_x = gps_x_current + (des_x - 100) * 6
    gps_x_current = get_gps_x()
    gps_y_current = get_gps_y()

    des_x = ((des_x_map - 100) * 30) + GPS_X_START
    des_y = (
        (des_y_map - 100) * 30
    ) + GPS_Y_START  # /2 * 6 => har do ta khooneh 6cm hast

    delta_x = des_x - gps_x_current
    delta_y = des_y - gps_y_current
    # if dir == NORTH or dir == SOUTH:
    #     e_x = 0
    # elif dir == WEST or dir == EAST:
    #     e_y = 0

    e_distance = sqrt((delta_x) ** 2 + (delta_y) ** 2)  # faseleye fisaghooresi
    # e_angle = atan(6/e_x)

    p_vic_is_found = 0
    while abs(e_distance) > 3 and robot.step(timeStep) != -1:
        if reset_flag == 1:
            break

        if ReadLidar("L") <= 90 and p_vic_is_found == 0:
            p_find(Camera1)  # left
        if ReadLidar("R") <= 90 and p_vic_is_found == 0:
            p_find(Camera2)




        #   update_coordinate()
        #   get_xy_map()
        gps_x_current = get_gps_x()
        gps_y_current = get_gps_y()
        delta_x = des_x - gps_x_current
        delta_y = des_y - gps_y_current

        e_distance = sqrt((delta_x) ** 2 + (delta_y) ** 2)

        v_distance = e_distance * K_DIS

        dest_angle = atan(delta_x / (delta_y + 0.000001))
        dest_angle = dest_angle * 180 / pi  # tabdil radian be darajeh

        if delta_x < 0 and delta_y > 0:
            dest_angle = -180 - dest_angle
        elif delta_x <= 0 and delta_y >= 0:
            dest_angle = -180 - dest_angle
        elif delta_x > 0 and delta_y >= 0:
            dest_angle = 180 - dest_angle
        elif delta_x > 0 and delta_y < 0:
            dest_angle = dest_angle + 90
        elif delta_x < 0 and delta_y < 0:
            dest_angle = dest_angle - 90
        elif delta_x == 0 and delta_y < 0:
            dest_angle = dest_angle

        e_angle = dest_angle - get_yaw()

        if e_angle > 180:
            e_angle = e_angle - 360
        elif e_angle < -180:
            e_angle = e_angle + 360

        k_angle = 2
        v_angle = e_angle * k_angle

        if int(ReadLidar("F")) < 40:
            flag1 = 1
            go(0, 0)
            break

        go(v_distance + v_angle, v_distance - v_angle)


    if flag1 == 1:
        move_back(gps_init_x, gps_init_y)
        SetWall(NORTH)
    go(0, 0)
    robot.step(timeStep)
    print("ROBOT STOPPPPPPPPEEDDDDDD!  ")
    gps_x_current = get_gps_x()
    gps_y_current = get_gps_y()
    delta_x = des_x - gps_x_current
    delta_y = des_y - gps_y_current
    e_distance = sqrt((delta_x) ** 2 + (delta_y) ** 2)
    correct_angle(dir)


def correct_angle(dir2):
    error = dir2 - get_yaw()
    turn(error)


def getMemory(dirMemory):



    if dirMemory == NORTH:
        return mapData[x_map, y_map - 2]

    elif dirMemory == NORTH_F:
        return mapData[x_map][y_map - 3]

    elif dirMemory == NORTH_LEFT:
        return mapData[x_map - 1, y_map - 2]

    elif dirMemory == NORTH_LEFT_F:
        return mapData[x_map - 1, y_map - 3]

    elif dirMemory == NORTH_RIGHT:
        return mapData[x_map + 1, y_map - 2]

    elif dirMemory == NORTH_RIGHT_F:
        return mapData[x_map + 1][y_map - 3]

    elif dirMemory == NORTH_EAST:
        return mapData[x_map + 2, y_map - 2]

    elif dirMemory == NORTH_EAST_UP:
        return mapData[x_map + 2, y_map - 3]

    elif dirMemory == NORTH_EAST_RIGHT:
        return mapData[x_map + 3, y_map - 2]

    elif dirMemory == EAST_UP:
        return mapData[x_map + 2, y_map - 1]

    elif dirMemory == EAST_UP_F:
        return mapData[x_map + 3, y_map - 1]

    elif dirMemory == EAST:
        return mapData[x_map + 2, y_map]

    elif dirMemory == EAST_F:
        return mapData[x_map + 3][y_map]

    elif dirMemory == EAST_DOWN:
        return mapData[x_map + 2, y_map + 1]

    elif dirMemory == EAST_DOWN_F:
        return mapData[x_map + 3, y_map + 1]

    elif dirMemory == SOUTH_EAST:
        return mapData[x_map + 2, y_map + 2]

    elif dirMemory == SOUTH_EAST_RIGHT:
        return mapData[x_map + 3, y_map + 2]

    elif dirMemory == SOUTH_EAST_DOWN:
        return mapData[x_map + 2, y_map + 3]

    elif dirMemory == SOUTH_LEFT:
        return mapData[x_map - 1, y_map + 2]

    elif dirMemory == SOUTH_LEFT_F:
        return mapData[x_map - 1, y_map + 3]

    elif dirMemory == SOUTH:
        return mapData[x_map, y_map + 2]

    elif dirMemory == SOUTH_F:
        return mapData[x_map][y_map + 3]

    elif dirMemory == SOUTH_RIGHT:
        return mapData[x_map + 1, y_map + 2]

    elif dirMemory == SOUTH_RIGHT_F:
        return mapData[x_map + 1, y_map + 3]

    elif dirMemory == NORTH_WEST:
        return mapData[x_map - 2, y_map - 2]

    elif dirMemory == NORTH_WEST_UP:
        return mapData[x_map - 2, y_map - 3]

    elif dirMemory == NORTH_WEST_LEFT:
        return mapData[x_map - 3, y_map - 2]

    elif dirMemory == WEST_UP:
        return mapData[x_map - 2, y_map - 1]

    elif dirMemory == WEST_UP_F:
        return mapData[x_map - 3, y_map - 1]

    elif dirMemory == WEST:
        return mapData[x_map - 2, y_map]

    elif dirMemory == WEST_F:
        return mapData[x_map - 3, y_map]

    elif dirMemory == WEST_DOWN:
        return mapData[x_map - 2, y_map + 1]

    elif dirMemory == WEST_DOWN_F:
        return mapData[x_map - 3, y_map + 1]

    elif dirMemory == SOUTH_WEST:
        return mapData[x_map - 2, y_map + 2]

    elif dirMemory == SOUTH_WEST_DOWN:
        return mapData[x_map - 2, y_map + 3]

    elif dirMemory == SOUTH_WEST_LEFT:
        return mapData[x_map - 3, y_map + 2]


def readMap():

    print("Analyzing the data to findout what the best path is!\n")
    SetMap()
    cntt = 0
    lst_paths = []

    # print(
    #     f"""
    #           WEST={getMemory(WEST)}
    #           WEST_UP={getMemory(WEST_UP)}
    #           WEST_DOWN={getMemory(WEST_DOWN)}
    #           WEST_F={getMemory(WEST_F)}
    #           WEST_UP_F= {getMemory(WEST_UP_F)}
    #           WEST_DOWN_F= {getMemory(WEST_DOWN_F)}

    #     """
    # )

    if (
        (getMemory(NORTH) < 1)
        and (getMemory(NORTH_RIGHT) > 2 or getMemory(NORTH_RIGHT) < 1)
        and (getMemory(NORTH_LEFT) > 2 or getMemory(NORTH_LEFT) < 1)
        and (getMemory(NORTH_F) > 2 or getMemory(NORTH_F) < 1)
        and (getMemory(NORTH_RIGHT_F) > 2 or getMemory(NORTH_RIGHT_F) < 1)
        and (getMemory(NORTH_LEFT_F) > 2 or getMemory(NORTH_LEFT_F) < 1)
    ):
        print("GOOOOOO FOOOORRWARDDDDD\n")
        cntt += 1
        lst_paths.append(NORTH)
        # return NORTH
    if (
        (getMemory(WEST) < 1)
        and (getMemory(WEST_UP) > 2 or getMemory(WEST_UP) < 1)
        and (getMemory(WEST_DOWN) > 2 or getMemory(WEST_DOWN) < 1)
        and (getMemory(WEST_F) > 2 or getMemory(WEST_F) < 1)
        and (getMemory(WEST_UP_F) > 2 or getMemory(WEST_UP_F) < 1)
        and (getMemory(WEST_DOWN_F) > 2 or getMemory(WEST_DOWN_F) < 1)
    ):
        print("TURN LEEEEEFTTTTTTTTTTT\n")
        cntt += 1
        lst_paths.append(WEST)
        # return WEST
    if (
        (getMemory(SOUTH) < 1)
        and (getMemory(SOUTH_LEFT) > 2 or getMemory(SOUTH_LEFT) < 1)
        and (getMemory(SOUTH_RIGHT) > 2 or getMemory(SOUTH_RIGHT) < 1)
        and (getMemory(SOUTH_F) > 2 or getMemory(SOUTH_F) < 1)
        and (getMemory(SOUTH_LEFT_F) > 2 or getMemory(SOUTH_LEFT_F) < 1)
        and (getMemory(SOUTH_RIGHT_F) > 2 or getMemory(SOUTH_RIGHT_F) < 1)
    ):
        print("TURN BACKKKKKKKKKK\n")
        lst_paths.append(SOUTH)
        cntt += 1
        # return SOUTH
    if (
        (getMemory(EAST) < 1)
        and (getMemory(EAST_F) > 2 or getMemory(EAST_F) < 1)
        and (getMemory(EAST_UP) > 2 or getMemory(EAST_UP) < 1)
        and (getMemory(EAST_DOWN) > 2 or getMemory(EAST_DOWN) < 1)
        and (getMemory(EAST_UP_F) > 2 or getMemory(EAST_UP_F) < 1)
        and (getMemory(EAST_DOWN_F) > 2 or getMemory(EAST_DOWN_F) < 1)
    ):
        print("TURN RIGHHHHHHHHTTTTTT\n")
        cntt += 1
        lst_paths.append(EAST)
        # return EAST
    if (
        (getMemory(NORTH_EAST) < 1)
        and (getMemory(NORTH_RIGHT) > 2 or getMemory(NORTH_RIGHT) < 1)
        and (getMemory(EAST_UP) > 2 or getMemory(EAST_UP) < 1)
        and (getMemory(NORTH_EAST_UP) > 2 or getMemory(NORTH_EAST_UP) < 1)
        and (getMemory(NORTH_EAST_RIGHT) > 2 or getMemory(NORTH_EAST_RIGHT) < 1)
    ):
        print("TURN RIGHTTTTT 454545454545454545\n")
        cntt += 1
        lst_paths.append(NORTH_EAST)
        # return NORTH_EAST
    if (
        (getMemory(NORTH_WEST) < 1)
        and (getMemory(WEST_UP) > 2 or getMemory(WEST_UP) < 1)
        and (getMemory(NORTH_WEST_LEFT) > 2 or getMemory(NORTH_WEST_LEFT) < 1)
        and (getMemory(NORTH_WEST_UP) > 2 or getMemory(NORTH_WEST_UP) < 1)
        and (getMemory(NORTH_LEFT) > 2 or getMemory(NORTH_LEFT) < 1)
    ):
        print("TURN LEFTTTTTT 454545454545454545\n")
        cntt += 1
        # return NORTH_WEST
        lst_paths.append(NORTH_WEST)

    if (
        (getMemory(SOUTH_EAST) < 1)
        and (getMemory(EAST_DOWN) > 2 or getMemory(EAST_DOWN) < 1)
        and (getMemory(SOUTH_RIGHT) > 2 or getMemory(SOUTH_RIGHT) < 1)
        and (getMemory(SOUTH_EAST_RIGHT) > 2 or getMemory(SOUTH_EAST_RIGHT) < 1)
        and (getMemory(SOUTH_EAST_DOWN) > 2 or getMemory(SOUTH_EAST_DOWN) < 1)
    ):
        print("TURN RIGHTTTTTT 135135135\n")
        cntt += 1
        # return SOUTH_EAST
        lst_paths.append(SOUTH_EAST)

    if (
        (getMemory(SOUTH_WEST) < 1)
        and (getMemory(SOUTH_WEST_DOWN) > 2 or getMemory(SOUTH_WEST_DOWN) < 1)
        and (getMemory(SOUTH_WEST_LEFT) > 2 or getMemory(SOUTH_WEST_LEFT) < 1)
        and (getMemory(WEST_DOWN) > 2 or getMemory(WEST_DOWN) < 1)
        and (getMemory(SOUTH_LEFT) > 2 or getMemory(SOUTH_LEFT) < 1)
    ):
        print("TURN LEFTTTTTT 135135135 \n")
        cntt += 1
        # return SOUTH_WEST
        lst_paths.append(SOUTH_WEST)

    if cntt == 0:
        print("An unknown error caused !\n")
        return "a_star"

    return lst_paths


def block_repeated_tile(dir_robot):
    # dir_block_r = dir_robot + 90
    # dir_block_l = dir_robot - 90
    # memory_R = getMemory(dir_block_r)
    # memory_L = getMemory(dir_block_l)
    # if memory_R == 0:
    #     memory_R += 1
    # elif memory_L == 0:
    #     memory_L = 10

    if dir_robot == NORTH or dir_robot == SOUTH:
        if mapData[x_map + 4][y_map] == 10 and mapData[x_map + 2][y_map] == 0:
            mapData[x_map + 2][y_map] = 10

        if mapData[x_map - 4][y_map] == 10 and mapData[x_map - 2][y_map] == 0:
            mapData[x_map - 2][y_map] = 10

    elif dir_robot == EAST or dir_robot == WEST:

        if mapData[x_map][y_map + 4] == 10 and mapData[x_map][y_map + 2] == 0:
            mapData[x_map][y_map + 2] = 10

        if mapData[x_map][y_map - 4] == 10 and mapData[x_map][y_map - 2] == 0:
            mapData[x_map][y_map - 2] = 10


def choosePath(lst_paths):
    dir = dir_update()
    delta_dirs = []
    mini = 1000
    for item in lst_paths:
        dir2 = item - dir
        if dir2 > 180:
            dir2 = dir2 - 360
        elif dir2 < -180:
            dir2 = dir2 + 360

        delta_dirs.append(dir2)

    for item2 in delta_dirs:
        if abs(item2) < abs(mini):
            mini = item2
    print("mini: ", mini)
    return mini


def print_a_star():
    for i in range(ymin - 3, ymax + 3):
        for j in range(xmin - 3, xmax + 3):
            if a_data[j][i] == -1:
                print(" . ", end="")
            elif a_data[j][i] == 0:
                print(" = ", end="")
            else:
                print(f"{a_data[j][i]:3}", end="")
        print("\n")
    print("\n")

go_to_home = 0
def a_star(mode):
    global go_to_home, remaining_time
    if mode == 1:
        print("salam dooste aziz!A* started!\n")
    for i in range(0, 200):
        for j in range(0, 200):
            a_data[i][j] = -1

    a_data[x_map][y_map] = 0
    print(f"x_map={x_map}y_map={y_map}")
    break_kon = 0
    for cnt in range(0, 200):
        for y_a_star in range(0, 200, 2):
            for x_a_star in range(0, 200, 2):
                if a_data[x_a_star][y_a_star] == cnt:

                    # print(f"X={x_a_star}Y={y_a_star}CNT = {cnt}")
                    wallUp = 0
                    wallRight = 0
                    wallLeft = 0
                    wallBack = 0
                    wallUr = 0
                    wallUl = 0
                    wallBr = 0
                    wallBl = 0

                    if (
                        mapData[x_a_star + 1][y_a_star - 2] == 1
                        or mapData[x_a_star - 1][y_a_star - 2] == 1
                        or mapData[x_a_star][y_a_star - 3] == 1
                        or mapData[x_a_star][y_a_star - 2] == 1
                        or a_data[x_a_star][y_a_star - 2] != -1
                        or mapData[x_a_star + 1][y_a_star - 2] == 2
                        or mapData[x_a_star - 1][y_a_star - 2] == 2
                        or mapData[x_a_star][y_a_star - 3] == 2
                        or mapData[x_a_star][y_a_star - 2] == 2
                        or mapData[x_a_star - 1][y_a_star - 1] == 2
                        or mapData[x_a_star + 1][y_a_star - 1] == 2
                        or mapData[x_a_star - 1][y_a_star - 3] == 2
                        or mapData[x_a_star + 1][y_a_star - 3] == 2
                    ):
                        wallUp = 1
                    if (
                        mapData[x_a_star + 2][y_a_star + 1] == 1
                        or mapData[x_a_star + 2][y_a_star - 1] == 1
                        or mapData[x_a_star + 3][y_a_star] == 1
                        or mapData[x_a_star + 2][y_a_star] == 1
                        or a_data[x_a_star + 2][y_a_star] != -1
                        or mapData[x_a_star + 2][y_a_star + 1] == 2
                        or mapData[x_a_star + 2][y_a_star - 1] == 2
                        or mapData[x_a_star + 3][y_a_star] == 2
                        or mapData[x_a_star + 2][y_a_star] == 2
                        or mapData[x_a_star + 1][y_a_star - 1] == 2
                        or mapData[x_a_star + 1][y_a_star + 1] == 2
                        or mapData[x_a_star + 3][y_a_star - 1] == 2
                        or mapData[x_a_star + 3][y_a_star + 1] == 2
                    ):
                        wallRight = 1

                    if (
                        mapData[x_a_star - 2][y_a_star + 1] == 1
                        or mapData[x_a_star - 2][y_a_star - 1] == 1
                        or mapData[x_a_star - 3][y_a_star] == 1
                        or mapData[x_a_star - 2][y_a_star] == 1
                        or a_data[x_a_star - 2][y_a_star] != -1
                        or mapData[x_a_star - 2][y_a_star + 1] == 2
                        or mapData[x_a_star - 2][y_a_star - 1] == 2
                        or mapData[x_a_star - 3][y_a_star] == 2
                        or mapData[x_a_star - 2][y_a_star] == 2
                        or mapData[x_a_star - 1][y_a_star - 1] == 2
                        or mapData[x_a_star - 1][y_a_star + 1] == 2
                        or mapData[x_a_star - 3][y_a_star - 1] == 2
                        or mapData[x_a_star - 3][y_a_star + 1] == 2
                    ):
                        wallLeft = 1

                    if (
                        mapData[x_a_star - 1][y_a_star + 2] == 1
                        or mapData[x_a_star + 1][y_a_star + 2] == 1
                        or mapData[x_a_star][y_a_star + 3] == 1
                        or mapData[x_a_star][y_a_star + 2] == 1
                        or a_data[x_a_star][y_a_star + 2] != -1
                        or mapData[x_a_star - 1][y_a_star + 2] == 2
                        or mapData[x_a_star + 1][y_a_star + 2] == 2
                        or mapData[x_a_star][y_a_star + 3] == 2
                        or mapData[x_a_star][y_a_star + 2] == 2
                        or mapData[x_a_star + 1][y_a_star + 1] == 2
                        or mapData[x_a_star - 1][y_a_star + 1] == 2
                        or mapData[x_a_star - 1][y_a_star + 3] == 2
                        or mapData[x_a_star + 1][y_a_star + 3] == 2
                    ):
                        wallBack = 1
                    if (
                        mapData[x_a_star + 1][y_a_star - 2] == 1
                        or mapData[x_a_star + 2][y_a_star - 1] == 1
                        or mapData[x_a_star + 2][y_a_star - 3] == 1
                        or mapData[x_a_star + 3][y_a_star - 2] == 1
                        or mapData[x_a_star + 2][y_a_star - 2] == 1
                        or a_data[x_a_star + 2][y_a_star - 2] != -1
                        or mapData[x_a_star + 1][y_a_star - 2] == 2
                        or mapData[x_a_star + 2][y_a_star - 1] == 2
                        or mapData[x_a_star + 2][y_a_star - 3] == 2
                        or mapData[x_a_star + 3][y_a_star - 2] == 2
                        or mapData[x_a_star + 2][y_a_star - 2] == 2
                        or mapData[x_a_star + 3][y_a_star - 1] == 2
                        or mapData[x_a_star + 3][y_a_star - 3] == 2
                        or mapData[x_a_star + 1][y_a_star - 3] == 2
                        or mapData[x_a_star + 1][y_a_star - 1] == 2
                    ):
                        wallUr = 1
                    if (
                        mapData[x_a_star - 1][y_a_star - 2] == 1
                        or mapData[x_a_star - 2][y_a_star - 1] == 1
                        or mapData[x_a_star - 3][y_a_star - 2] == 1
                        or mapData[x_a_star - 2][y_a_star - 3] == 1
                        or mapData[x_a_star - 2][y_a_star - 2] == 1
                        or a_data[x_a_star - 2][y_a_star - 2] != -1
                        or mapData[x_a_star - 1][y_a_star - 2] == 2
                        or mapData[x_a_star - 2][y_a_star - 1] == 2
                        or mapData[x_a_star - 3][y_a_star - 2] == 2
                        or mapData[x_a_star - 2][y_a_star - 3] == 2
                        or mapData[x_a_star - 2][y_a_star - 2] == 2
                        or mapData[x_a_star - 3][y_a_star - 1] == 2
                        or mapData[x_a_star - 3][y_a_star - 3] == 2
                        or mapData[x_a_star - 1][y_a_star - 3] == 2
                        or mapData[x_a_star - 1][y_a_star - 1] == 2
                    ):
                        wallUl = 1
                    if (
                        mapData[x_a_star + 2][y_a_star + 1] == 1
                        or mapData[x_a_star + 1][y_a_star + 2] == 1
                        or mapData[x_a_star + 3][y_a_star + 2] == 1
                        or mapData[x_a_star + 2][y_a_star + 3] == 1
                        or mapData[x_a_star + 2][y_a_star + 2] == 1
                        or a_data[x_a_star + 2][y_a_star + 2] != -1
                        or mapData[x_a_star + 2][y_a_star + 1] == 2
                        or mapData[x_a_star + 1][y_a_star + 2] == 2
                        or mapData[x_a_star + 3][y_a_star + 2] == 2
                        or mapData[x_a_star + 2][y_a_star + 3] == 2
                        or mapData[x_a_star + 2][y_a_star + 2] == 2
                        or mapData[x_a_star + 3][y_a_star + 1] == 2
                        or mapData[x_a_star + 3][y_a_star + 3] == 2
                        or mapData[x_a_star + 1][y_a_star + 3] == 2
                        or mapData[x_a_star + 1][y_a_star + 1] == 2
                    ):
                        wallBr = 1
                    if (
                        mapData[x_a_star - 1][y_a_star + 2] == 1
                        or mapData[x_a_star - 2][y_a_star + 1] == 1
                        or mapData[x_a_star - 3][y_a_star + 2] == 1
                        or mapData[x_a_star - 2][y_a_star + 3] == 1
                        or mapData[x_a_star - 2][y_a_star + 2] == 1
                        or a_data[x_a_star - 2][y_a_star + 2] != -1
                        or mapData[x_a_star - 1][y_a_star + 2] == 2
                        or mapData[x_a_star - 2][y_a_star + 1] == 2
                        or mapData[x_a_star - 3][y_a_star + 2] == 2
                        or mapData[x_a_star - 2][y_a_star + 3] == 2
                        or mapData[x_a_star - 2][y_a_star + 2] == 2
                        or mapData[x_a_star - 3][y_a_star + 1] == 2
                        or mapData[x_a_star - 3][y_a_star + 3] == 2
                        or mapData[x_a_star - 1][y_a_star + 3] == 2
                        or mapData[x_a_star - 1][y_a_star + 1] == 2
                    ):
                        wallBl = 1


                    if mode == 1:
                        if (
                            mapData[x_a_star][y_a_star] == 0
                            or mapData[x_a_star][y_a_star] == -1
                        ):
                            break_kon = 1
                            break
                    if mode == 0:
                        if a_data[X_OFFSET][Y_OFFSET] != -1:
                            break_kon = 1
                            break

                    if wallUp == 0:

                        a_data[x_a_star][y_a_star - 2] = cnt + 1
                        if mapData[x_a_star][y_a_star - 2] == 3:
                            a_data[x_a_star][y_a_star - 2] = cnt + 2

                    if wallUr == 0:

                        a_data[x_a_star + 2][y_a_star - 2] = cnt + 1
                        if mapData[x_a_star + 2][y_a_star - 2] == 3:
                            a_data[x_a_star + 2][y_a_star - 2] = cnt + 2

                    if wallLeft == 0:
                        a_data[x_a_star - 2][y_a_star] = cnt + 1
                        if mapData[x_a_star - 2][y_a_star] == 3:
                            a_data[x_a_star - 2][y_a_star] = cnt + 2

                    if wallUl == 0:
                        a_data[x_a_star - 2][y_a_star - 2] = cnt + 1
                        if mapData[x_a_star - 2][y_a_star - 2] == 3:
                            a_data[x_a_star - 2][y_a_star - 2] = cnt + 2

                    if wallRight == 0:
                        a_data[x_a_star + 2][y_a_star] = cnt + 1
                        if mapData[x_a_star + 2][y_a_star] == 3:
                            a_data[x_a_star + 2][y_a_star] = cnt + 2

                    if wallBr == 0:
                        a_data[x_a_star + 2][y_a_star + 2] = cnt + 1
                        if mapData[x_a_star + 2][y_a_star + 2] == 3:
                            a_data[x_a_star + 2][y_a_star + 2] = cnt + 2

                    if wallBack == 0:
                        a_data[x_a_star][y_a_star + 2] = cnt + 1
                        if mapData[x_a_star][y_a_star + 2] == 3:
                            a_data[x_a_star][y_a_star + 2] = cnt + 2

                    if wallBl == 0:
                        a_data[x_a_star - 2][y_a_star + 2] = cnt + 1
                        if mapData[x_a_star - 2][y_a_star + 2] == 3:
                            a_data[x_a_star - 2][y_a_star + 2] = cnt + 2

            if break_kon == 1:
                break
        if break_kon == 1:
            cnt = 203
            break
    
    if mode == 0:
        print("FASELE to HOME: ", a_data[X_OFFSET][Y_OFFSET])
        # if (a_data[X_OFFSET][Y_OFFSET] * 2 > get_time() - 30) and (get_time() != 0):
        get_time()
        # if (remaining_time < 465) and (remaining_time != 0):
        if (a_data[X_OFFSET][Y_OFFSET] * 2 > remaining_time - 30) and (remaining_time!= 0):
            print("You should go back home!\n")
            go_to_home = 1
            print("ymin, ymax, xmin, xmax: ",ymin, ymax, xmin, xmax)
            for i in range(ymin, ymax + 10, 2):
                for j in range(xmin, xmax + 10, 2):
                    if mapData[j][i] == 0 or mapData[j][i] == -1:
                        mapData[j][i] = 10
                        

    
    if break_kon == 0 or go_to_home == 1:
        if x_map == 100 and y_map == 100:
            print("Algorithm work has been done!")
            finish()
        else:
            print("GO TO HOMEEEEEEE ")
            x_a_star = 100
            y_a_star = 100
            mapData[x_a_star][y_a_star] = 0
    if mode == 1 or (mode == 0 and go_to_home == 1):
        routing(x_a_star, y_a_star)
        print_a_star()
        print(f"Destinaion is X: {x_a_star} Y:{y_a_star} \n")


def routing(x_current, y_current):
    mapData[x_current][y_current] = -1
    lst_values = np.zeros([8])

    while robot.step(timeStep) != -1:
        # print("972 972 972\n")
        mini = 100
        mini_num = -1
        lst_values[0] = a_data[x_current][y_current - 2]
        lst_values[1] = a_data[x_current + 2][y_current - 2]
        lst_values[2] = a_data[x_current + 2][y_current]
        lst_values[3] = a_data[x_current + 2][y_current + 2]
        lst_values[4] = a_data[x_current][y_current + 2]
        lst_values[5] = a_data[x_current - 2][y_current + 2]
        lst_values[6] = a_data[x_current - 2][y_current]
        lst_values[7] = a_data[x_current - 2][y_current - 2]

        for i in range(0, 8):
            if lst_values[i] < mini and lst_values[i] != -1:
                mini = lst_values[i]
                mini_num = i

        if mapData[x_map][y_map] == -1:
            break

        if mini_num == 0:
            mapData[x_current][y_current - 2] = -1

            x_current = x_current
            y_current = y_current - 2

        if mini_num == 1:
            mapData[x_current + 2][y_current - 2] = -1
            x_current = x_current + 2
            y_current = y_current - 2
        if mini_num == 2:
            mapData[x_current + 2][y_current] = -1
            x_current = x_current + 2
            y_current = y_current

        if mini_num == 3:
            mapData[x_current + 2][y_current + 2] = -1
            x_current = x_current + 2
            y_current = y_current + 2

        if mini_num == 4:
            mapData[x_current][y_current + 2] = -1
            x_current = x_current
            y_current = y_current + 2
        if mini_num == 5:
            mapData[x_current - 2][y_current + 2] = -1
            x_current = x_current - 2
            y_current = y_current + 2
        if mini_num == 6:
            mapData[x_current - 2][y_current] = -1
            x_current = x_current - 2
            y_current = y_current
        if mini_num == 7:
            mapData[x_current - 2][y_current - 2] = -1
            x_current = x_current - 2
            y_current = y_current - 2


def lop_called():
    global reset_flag
    # If receiver queue is not empty
    length = receiver.getQueueLength()
    print("LOP CALLED= length: ", length)
    if length > 0:
        receivedData = receiver.getBytes()
        print("recieved Data:  type:", receivedData, type(receivedData))
        # tup = struct.unpack("c", receivedData)  # Parse data into character
        tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int


        print("length: ", len(tup))
        if tup[0].decode("utf-8") == "L":  # 'L' means lack of progress occurred
            print("Detected Lack of Progress!")
            go(0, 0)
            delay(1000)
            update_coordinate_map()
            # print("Bonjure Madame")
            print("getYAWWWWW", get_yaw())
            if abs(get_yaw()) > 1:
                turn(-get_yaw())
            # print("Bonjure Mousiure")
            reset_flag = 1
        receiver.nextPacket()  # Discard the current data packet



# if receiver.getQueueLength() > 0: # If receiver queue is not empty
#     receivedData = receiver.getBytes()
#     tup = struct.unpack('c', receivedData) # Parse data into character
#     if tup[0].decode("utf-8") == 'L': # 'L' means lack of progress occurred
#         print("Detected Lack of Progress!") 
#     receiver.nextPacket() # Discard the current data packet
        

remaining_time = 0
def get_time():
    global remaining_time
    # print('get time started1')
    # receiver.nextPacket()
    message = struct.pack("c", "G".encode())  # message = 'G' for game information
    emitter.send(message)  # send message
    # print('get time started2')
    length = receiver.getQueueLength()
    print("TIME = length: ", length)
    # print("reciever.get=> ", receiver.getQueueLength())
    if length > 0:  # If receiver queue is not empty
        receivedData = receiver.getBytes()
        tup = struct.unpack('c f i i', receivedData) 
        print(f"0= {tup[0]}      1= {tup[1]}     2= {tup[2]}")
        if tup[0].decode("utf-8") == "G":
            # print('get time started3')
            print(f"Remaining time: {tup[2]} TYPE: {type(tup[2])}")

            remaining_time = tup[2]
            receiver.nextPacket()  # Discard the current data packet


def get_score():
    global score
    # print('get time started1')
    # receiver.nextPacket()
    message = struct.pack("c", "G".encode())  # message = 'G' for game information
    emitter.send(message)  # send message
    # print('get time started2')

    # print("reciever.get=> ", receiver.getQueueLength())
    if receiver.getQueueLength() > 0:  # If receiver queue is not empty
        receivedData = receiver.getBytes()
        tup = struct.unpack('c f i i', receivedData) 

        if tup[0].decode("utf-8") == "G":
            # print('get time started3')
            print(f"SCORE: {tup[1]} TYPE: {type(tup[1])}")
            score = tup[1]
            receiver.nextPacket()  # Discard the current data packet


def send_supervisor():
    global room4_found
    subMatrix = np.zeros([((xmax) - (xmin) + 1), ((ymax) - (ymin) + 1)], dtype=str)
    subMatrix2 = np.zeros([((ymax) - (ymin) + 1), ((xmax) - (xmin) + 1)], dtype=str)
    for y3 in range(ymin, ymax + 1):
        for x3 in range(xmin, xmax + 1):
            if (y3%4 == 0) and (x3%4 == 0) and mapData[x3,y3]==2:
                mapData[x3-1,y3]=0
                mapData[x3,y3-1]=0
                mapData[x3,y3]=0
                mapData[x3,y3+1]=0
                mapData[x3+1,y3]=0
            if (y3%4 == 0) and (x3%4 == 0) and mapData[x3,y3]==3:
                mapData[x3-1,y3]=0
                mapData[x3,y3-1]=0
                mapData[x3,y3]=0
                mapData[x3,y3+1]=0
                mapData[x3+1,y3]=0
    for y2 in range(ymin, ymax + 1):
        for x2 in range(xmin, xmax + 1):
            if mapData[x2, y2] == 0:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
            elif mapData[x2, y2] == 10:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
            elif mapData[x2, y2] == 1:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "1"
            elif mapData[x2, y2] == 2:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "2"
            elif mapData[x2, y2] == 3:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "3"
            elif mapData[x2, y2] == 4:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "4"
            elif mapData[x2, y2] == 5:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "5"
            elif mapData[x2, y2] == 82:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "r"
            elif mapData[x2, y2] == 71:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "g"
            elif mapData[x2, y2] == 66:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "b"
            elif mapData[x2, y2] == 80:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "p"
            elif mapData[x2, y2] == 89:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "y"
            elif mapData[x2, y2] == 79:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "o"
            elif mapData[x2, y2] == 30:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "S"
            elif mapData[x2, y2] == 31:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "H"
            elif mapData[x2, y2] == 32:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "U"
            elif mapData[x2, y2] == 33:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "C"
            elif mapData[x2, y2] == 34:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "P"
            elif mapData[x2, y2] == 35:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "F"
            elif mapData[x2, y2] == 36:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
                subMatrix[x2 - (xmin), y2 - (ymin)] = "O"
            else:
                subMatrix[x2 - (xmin), y2 - (ymin)] = "0"
    if room4_found == 1:            
        for yy in range(ymin, ymax + 1):
            for xx in range(xmin, xmax + 1):
                if mapData2[xx,yy] == 7:
                    subMatrix[xx - (xmin), yy - (ymin)] = "*"

    # for yi in range(0, ymax - ymin + 1):
    #     for xi in range(0, xmax - xmin + 1):
    #         print("%2d" % int(subMatrix[xi, yi]), end=" ")
    #     print("\n")

    # for x2 in range(xmin , xmax+1 ):
    #     subMatrix[x2-(xmin),0]='1'
    # for x2 in range(xmin , xmax+1 ):
    #     subMatrix[x2-(xmin),(ymax)-(ymin)]='1'
    # for y2 in range(ymin , ymax+1 ):
    #     subMatrix[0,y2-(ymin)]='1'
    # for y2 in range(ymin , ymax+1 ):
    #     subMatrix[(xmax)-(xmin),y
    #
    # 2-(ymin)]='1'

    for y2 in range(ymin, ymax + 1):
        for x2 in range(xmin, xmax + 1):
            subMatrix2[y2 - (ymin), x2 - (xmin)] = subMatrix[x2 - (xmin), y2 - (ymin)]



    # for y2 in range(0, ((ymax)-(ymin)+1)):
    #     print("\n")
    #     for x2 in range(0, ((xmax)-(xmin)+1)):
    #         print((subMatrix[x2, y2]), end=" ")

    s = subMatrix2.shape
    s_bytes = struct.pack("2i", *s)

    flatMap = ",".join(subMatrix2.flatten())
    sub_bytes = flatMap.encode("utf-8")
    a_bytes = s_bytes + sub_bytes
    emitter.send(a_bytes)
    map_evaluate_request = struct.pack("c", b"M")
    emitter.send(map_evaluate_request)
    exit_mes = struct.pack("c", b"E")
    emitter.send(exit_mes)


def finish():
    global xmin
    global ymin
    global xmax
    global ymax
    for ix in range(0, 249):
        for iy in range(0, 249):
            if mapData[ix, iy] == 1:
                if ix < xmin:
                    xmin = ix
                if iy < ymin:
                    ymin = iy
                if ix > xmax:
                    xmax = ix
                if iy > ymax:
                    ymax = iy
    # print(f"\n xmax = {xmax} ymax = {ymax} xmin = {xmin} ymin = {ymin} ")
    score = np.zeros([xmax - xmin + 1, ymax - ymin + 1])
    for xi in range(xmin, xmax + 1):
        for yi in range(ymin, ymax + 1):
            score[xi - xmin][yi - ymin] = mapData[xi][yi]
    for yi in range(0, ymax - ymin + 1):
        for xi in range(0, xmax - xmin + 1):
            print("%2d" % int(score[xi, yi]), end=" ")
        print("\n")

    print("\n")
    mapData[x_map + 1, y_map + 1] = 5
    mapData[x_map + 1, y_map - 1] = 5
    mapData[x_map - 1, y_map + 1] = 5
    mapData[x_map - 1, y_map - 1] = 5
    send_supervisor()


flag = 0
# RIGHT_ARROW_KEY = 316

go(0, 0)
delay(100)
GPS_X_START = get_gps_x()
GPS_Y_START = get_gps_y()
# GPS_X_START_B = get_gps_x()
# GPS_Y_START_B = get_gps_x() + 60


x_map = X_OFFSET
y_map = Y_OFFSET
xmin = 100
ymin = 100
xmax = 100
ymax = 100

temp = get_yaw()
turn(-temp)

while robot.step(timeStep) != -1:

    BlackHoleFound = 0
    reset_flag = 0
    update_coordinate_map()
    # lop_called()
    SetMap()
    maproom4()
    setblackhole()
    setcolor()
    readMap()
    a_star(0) # calculates what the distance to the starting tile is.
    get_time()
    # get_score()

    print("CHECKROOM4: ", check_room4)
    # print(" REMAINING TIME=> ", remaining_time)
    # print(" get score TIME=> ", score)


    if x_map < xmin:
        xmin = x_map
    if x_map > xmax:
        xmax = x_map
    if y_map < ymin:
        ymin = y_map
    if y_map > ymax:
        ymax = y_map

    PrintMap()
    ##lop_called()
    if readMap() != "a_star":
        # print("lst_paths",readMap())
        # print("yek")
        if reset_flag == 0:
            dir_before = dir_update()
            turn((choosePath(readMap())))
            dir_after = dir_update()
        if reset_flag == 0:
            setblackhole()
        if reset_flag == 0:
            if BlackHoleFound == 0:
                if abs(dir_after - dir_before) == 180:
                    redSee = 0
                    greenSee = 0
                    orangeSee = 0
                    color()
                print("colour", color())
                color()
                forward()
        # move_back(GPS_X_START_B, GPS_Y_START_B)
    else:
        if reset_flag == 0:
            a_star(1)

    # key = keyboard.getKey()
    # if key == keyboard.UP:
    #     print(f"gpsx={get_gps_x()}, gpsy={get_gps_y()} \n")
    #     forward()
    # elif key == keyboard.RIGHT:
    #     turn(NORTH_EAST)
    # elif key == keyboard.DOWN:
    #     turn(SOUTH)
    # elif key == keyboard.LEFT:
    #     turn(NORTH_WEST)
    # # print(f"yaw111:{get_yaw()}")


keyboard.disable()
