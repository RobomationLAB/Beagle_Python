# -*- coding: utf-8 -*-

# made by HOO
import sys
import os

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)


import serial
import binascii
import re
import queue
import threading
import time
from threading import Thread
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.animation as animation
from PyQt5 import uic
from bitstring import Bits
import struct
# import keyboard
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
import serial.tools.list_ports  # For listing available serial ports
import math

# form_class = uic.loadUiType("untitled12.ui")[0]
# form_class = uic.loadUiType("untitled11.ui")[0]
# form_class = uic.loadUiType("beagle_lidar_final.ui")[0]

form = resource_path("beagle_lidar_final.ui")
form_class = uic.loadUiType(form)[0]
# form_class = uic.loadUiType("Lidar3.ui")[0]
__platform__ = sys.platform


########### Automatically detect serial ports ###########

global COM
COM = "COM"
find_port = True
dev = serial.tools.list_ports.comports()

ports=[]

for d in dev:

    # ports.append((d.device, d.serial_number))
    ports.append(d.device)

# print('\nDetected serial ports:')

# for d in ports:
#
#     print("Port:" + str(d[0]) + "\tSerial Number:" + str(d[1]))
# ports.sort()
print("ports : " ,ports)
for i in range(len(ports)):
    try:
        ser = serial.Serial(
                        port=ports[i],
                        # baudrate=115200,
                        baudrate=460800,
                        # 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                    )
        print(ports[i])
        my_list = []
        for d in range(100):
            if ser.readable():
                # if Serial().ser.readable():

                # print("readable ..  get data 동작중...")
                rxData = ser.read(1)  # Get a value by 1 byte
                # print(rxData)
                Data = binascii.b2a_hex(rxData).decode('ascii')
                # print(Data)
                hexStr = ' 0x'.join(re.findall('(.{2})', Data))
                # print(hexStr)
                hexStr = '0x' + hexStr
                # print(hexStr)

                data = hexStr.split(" ")
                my_list.append(data)

        print(my_list)
        print(my_list[0][0])
        print(type(my_list))
        for a in range(len(my_list)):
            if my_list[a][0] == "0x52":
                if my_list[a+1][0] == "0x4f":
                    print("Find!!!!")
                    COM = ports[i]
                    find_port = False
                    ser.close()
                    break
        if find_port == False:
            break
    except:
        print("Cannot connect to {}".format(ports[i]))



print("COM = ",COM)
######################

hexStr = []
list_data_str = []
q_box = queue.Queue()
n = 0
d = 0
imu_index = 0
gyro = 1
acc = 1
mag = 1
keyboard_val = 0
up = 0
down = 0
right = 0
left = 0

robo_message_ID_lst = [0]
robo_data_length_lst = [0]
robo_index_lst = [0, 0]
robo_index_lst_val = 0
robo_time_stamp1_lst = [0]
robo_time_stamp2_lst = []
robo_encoder_left1_lst = [0]
robo_encoder_right1_lst = [0]
robo_temperature_lst = [0]
robo_rssi_lst = [0]
robo_battery_voltage_lst = [0]
robo_battery_state_lst = [0]
robo_motor_state_lst = [0]
robo_sound_state_lst = [0]
robo_servo1_speed_lst = [0]
robo_servo1_angle_lst = [0]
robo_servo2_speed_lst = [0]
robo_servo2_angle_lst = [0]
robo_servo3_speed_lst = [0]
robo_servo3_angle_lst = [0]

imu_message_ID_lst = [0]
imu_data_length_lst = [0]
imu_index_lst = [0]
imu_index_lst_val = 0
imu_time_stamp1_lst = [0]
imu_gyro_lst = [0]
imu_gyro_range_lst = [0]
imu_acc_lst = [0]
imu_acc_range_lst = [0]
imu_mag_odr_lst = [0]
imu_mag_rep_xy_lst = [0]
imu_mag_rep_z_lst = [0]
imu_data_id1_lst = [0]
imu_gyrox_n1_lst = [0]
imu_gyroy_n1_lst = [0]
imu_gyroz_n1_lst = [0]
imu_data_id2_lst = [0]
imu_accx_n1_lst = [0]
imu_accy_n1_lst = [0]
imu_accz_n1_lst = [0]
imu_acc_temp_n_lst = [0]
imu_data_id3_lst = [0]
imu_compassx_n1_lst = [0]
imu_compassy_n1_lst = [0]
imu_compassz_n1_lst = [0]
data_id_and_data_lst = [0]

imu_gyrox_len = [0, 0]

gyro_ind_lst = [0]
acc_ind_lst = [0]
mag_ind_lst = [0]

lidar_n = 0  ##star, stop index
angle = 0
wait_flag = 0

Pos_flag = 0    # Indicates the quadrant position

DataPos0 = 0    # Angle between quadrant 1 data
DataPos1 = 0    # Angle between quadrant 2 data
DataPos2 = 0    # Angle between quadrant 3 data
DataPos3 = 0    # Angle between quadrant 4 data

start_lidar = 1



ToFPosDataQ0 = []   # quadrant 1 distance list
ToFPosDataQ1 = []   # quadrant 2 distance list
ToFPosDataQ2 = []   # quadrant 3 distance list
ToFPosDataQ3 = []   # quadrant 4 distance list

thread = 1

########### Draw Gyro, Acc, and Mag graphs ###########
class MyMplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=3, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)


        self.axes = fig.add_subplot(111, xlim=(0, 100), ylim=(-33000, 33000))
        self.axes.set_xticks([0,20,40,60,80,100])
        self.axes.set_yticks([-33000,-20000,-10000,0,10000,20000,33000])
        self.axes.set_yticklabels([-33000, -20000,-10000 ,0, 10000,20000, 33000],fontsize=7)
        self.axes.set_xticklabels([0, 20, 40, 60, 80, 100],fontsize=7)


        # self.axes.facecolor("m")
        # fig.suptitle("Gyro")
        # self.axes.set_ylabel("Gravity")
        # self.axes.set_xlabel("Sec (X-Red/Y-green/Z-blue)")
        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig)

        self.setParent(parent)

    def compute_initial_figure(self):
        pass


class MyMplCanvas2(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig2 = Figure(figsize=(width, height), dpi=dpi)


        # self.axes2 = fig2.add_subplot(111, xlim=(0, 50), ylim=(-33000, 33000))
        self.axes2 = fig2.add_subplot(111, xlim=(0, 100), ylim=(-2048, 2048))

        self.axes2.set_yticks([-2048,-1000,0,1000,2048])


        # self.axes3 = fig.add_subplot(111, xlim=(0, 50), ylim=(-2048, 2048))
        # fig2.suptitle("Accelerometer", fontsize = 25)
        # self.axes2.set_ylabel("deg/s(Raw)")
        # self.axes2.set_xlabel("Sec (X-Red/Y-green/Z-blue)")
        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig2)

        self.setParent(parent)

    def compute_initial_figure(self):
        pass


class MyMplCanvas3(FigureCanvas):
    def __init__(self, parent=None, width=1, height=1, dpi=100):
        fig3 = Figure(figsize=(width, height), dpi=dpi)

        # self.axes2 = fig2.add_subplot(111, xlim=(0, 50), ylim=(-33000, 33000))
        # self.axes3 = fig3.add_subplot(111, xlim=(0, 100), ylim=(-128, 128))

        self.axes3 = fig3.add_subplot(111, xlim=(0, 100), ylim=(-150, 150))




        # self.axes3.set_xlabel("dddddd")
        # fig3.set_facecolor("m")  # Background color
        # fig3.suptitle("Magnetometer")
        # self.axes3.set_ylabel("u/T(Raw)")
        # self.axes3.set_xlabel("Sec (X-Red/Y-green/Z-blue)")
        # self.axes3.set_xtick(10)


        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig3)

        self.setParent(parent)

    def compute_initial_figure(self):
        pass

########### Main ###########
class WindowClass(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        ##################################################################
        self.setWindowTitle("BEAGLE 0.2")
        self.canvas_gyro = MyMplCanvas(self, width=10, height=3, dpi=100)
        # self.canvas_gyro.setStyleSheet( "border-style : solid;" "border-width : 0.5px;")
        self.graph_verticalLayout.addWidget(self.canvas_gyro)

        self.x = np.linspace(0, 100, 101)

        self.y = np.ones(101)

        self.line_g1, = self.canvas_gyro.axes.plot(self.x, self.y, animated=True, lw=1, color='red')
        self.line_g2, = self.canvas_gyro.axes.plot(self.x, self.y, animated=True, lw=1, color='blue')
        self.line_g3, = self.canvas_gyro.axes.plot(self.x, self.y, animated=True, lw=1, color='green')

        self.ani_gyro = animation.FuncAnimation(self.canvas_gyro.figure, self.update_line_gyro, blit=True, interval=2)

        ##################################################################
        # acc graph
        self.canvas_acc = MyMplCanvas2(self, width=10, height=3, dpi=100)
        self.graph_verticalLayout_2.addWidget(self.canvas_acc)
        self.x_acc = np.linspace(0, 100, 101)

        self.y_acc = np.ones(101)

        self.line_a1, = self.canvas_acc.axes2.plot(self.x_acc, self.y_acc, animated=True, lw=1, color='red')
        self.line_a2, = self.canvas_acc.axes2.plot(self.x_acc, self.y_acc, animated=True, lw=1, color='blue')
        self.line_a3, = self.canvas_acc.axes2.plot(self.x_acc, self.y_acc, animated=True, lw=1, color='green')


        self.ani_acc = animation.FuncAnimation(self.canvas_acc.figure, self.update_line_acc, blit=True, interval=2)
        ##################################################################
        # mag graph
        self.canvas_mag = MyMplCanvas3(self, width=10, height=3, dpi=100)
        self.graph_verticalLayout_3.addWidget(self.canvas_mag)
        self.x_mag = np.linspace(0, 100, 101)
        # self.x_acc = np.arange(20)
        self.y_mag = np.ones(101)

        self.line_m1, = self.canvas_mag.axes3.plot(self.x_mag, self.y_mag, animated=True, lw=1, color='red')
        self.line_m2, = self.canvas_mag.axes3.plot(self.x_mag, self.y_mag, animated=True, lw=1, color='blue')
        self.line_m3, = self.canvas_mag.axes3.plot(self.x_mag, self.y_mag, animated=True, lw=1, color='green')
        # self.line2, = self.canvas.axes.plot(self.x2, self.y2, animated=True, lw=2,color='green')

        self.ani_mag = animation.FuncAnimation(self.canvas_mag.figure, self.update_line_mag, blit=True, interval=2)

        ##################################################################

        ############################## Default setting #####################################
        self.Tx_right = 0x00
        self.Tx_left = 0x00
        self.Tx_distance_range = 0x00
        self.Tx_distance_id = 0x00
        self.Tx_sound_clip = 0x00
        self.Tx_buzzer = 0x00
        self.Tx_buzzer_low = 0x00
        self.Tx_buzzer_middle = 0x00
        self.Tx_buzzer_high = 0x00
        self.Tx_musical = 0x00
        self.Tx_servo1_speed = 0x00
        self.Tx_servo2_speed = 0x00
        self.Tx_servo3_speed = 0x00
        self.Tx_servo1_angle = 0x5a
        self.Tx_servo2_angle = 0x5a
        self.Tx_servo3_angle = 0x5a
        self.Tx_motor_mode = 0x80
        self.Tx_distance_range = 0x00
        self.pps = 0
        self.keyboard_slider_val = 0x00

        # self.mag_rep_xy = 0x00
        # self.mag_rep_z = 0x00
        self.acc_index = 0x00
        self.mag_index = 0x00
        self.gyro_index = 0x00
        self.gyro_odr = 0x00
        self.gyro_range = 0x00
        self.acc_odr = 0x00
        self.acc_range = 0x00
        self.mag_odr = 0x00
        self.mag_rep_xy = 0X23
        self.mag_rep_z = 0X82

        self.gyro_odr_real = 0x00
        self.gyro_range_real = 0x00
        self.acc_odr_real = 0x00
        self.acc_range_real = 0x00
        self.mag_odr_real = 0x20
        # self.mag_rep_xy = 0x00
        # self.mag_rep_z = 0x00

        self.continous = 1
        self.distance = 0

        self.mag_rep_xy_real = 0x23
        self.mag_rep_z_real = 0x82
        self.mag_odr_real = 0x20
        self.Tx_buzzer_high_real = 0x00
        self.Tx_buzzer_middle_real = 0x00
        self.Tx_buzzer_low_real= 0x00
        self.Tx_musical_real = 0x00
        self.Tx_sound_clip_real = 0x00
        self.Tx_right_real = 0x00
        self.Tx_left_real = 0x00
        self.pps_val = 0x00
        self.toggle = 0
        self.msb = 0
        self.motor = 0
        self.soundclip = 0
        self.musical = 0
        self.buzzer = 0


        ###########################


        # exit_action = QAction("E&xit", self)
        # exit_action.setShortcut(QKeySequence(Qt.CTRL + Qt.Key_Q))
        # exit_action.setStatusTip("Exit application window")
        # exit_action.triggered.connect(self.my_exit)
        # filemenu.addAction(exit_action)
        ##################### GUI setup and image file upload ########################
        ############################################### Motor #############################################################
        self.Beagle_title.setStyleSheet( "border-image: url(Beagle.png);" )
        self.motor_title.setStyleSheet("color : white;" "background-color : black;")
        self.motor_background.setStyleSheet("background-color : #f4f4f4;")
        self.motor_operating.setStyleSheet( "border-image: url(Stop_01.png);" )
        #### System status
        self.system_title.setStyleSheet("color : white;" "background-color : black;" )
        self.Move_btn.setStyleSheet("border-image: url(Move.png);")
        self.Stop_btn.setStyleSheet("border-image: url(Stop.png);")
        self.distance_mode_txt.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.right_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.left_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.reset_btn.setStyleSheet("border-image: url(Reset.png);")
        # self.distance_range_slider.setStyleSheet("color : white;" "background-color :#dbdbdb;" )
        self.distance_range_slider.setStyleSheet("color : black;" "background-color :#dbdbdb;")
        self.right_slider.setStyleSheet("color : black;" "background-color :#dbdbdb;")
        self.left_slider.setStyleSheet("color : black;" "background-color :#dbdbdb;")

        # self.label.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        # self.label.setStyleSheet("background-color : #e1e1e1")

        ############################################### sound #############################################################
        self.sound_title.setStyleSheet("color : white;" "background-color : black;")
        self.sound_btn.setStyleSheet("border-image: url(Play2x.png);")
        self.sound_stop_btn.setStyleSheet("border-image: url(Stop.png);")
        self.musical_btn.setStyleSheet("border-image: url(Play2x.png);")
        self.sound_background.setStyleSheet("background-color : #f4f4f4;")
        self.reset_btn_2.setStyleSheet("border-image: url(Reset2x.png);")
        # self.musical_stop_btn.setStyleSheet("background-color : #d7dddc;" "border-color : black;" "border-style : solid;" "border-width : 2px;")
        self.buzzer_btn.setStyleSheet("border-image: url(Play2x.png);")
        self.musical_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.buzzer_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")


        ############################################### system status #############################################################
        self.Message_ID.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Data_length.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Index.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Time_stamp.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Encoder_left.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Encoder_right.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Temperature.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Rssi.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Battery_voltage.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Battery_state.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Motor_state.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Sound_state.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo1_speed.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo1_angle.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo2_speed.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo2_angle.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo3_speed.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")
        self.Servo3_angle.setStyleSheet("background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-left-width : 2px;")

        self.Message_ID_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Data_length_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Index_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Time_stamp_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Encoder_left_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Encoder_right_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Temperature_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Rssi_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Battery_voltage_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Battery_state_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Motor_state_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Sound_state_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo1_speed_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo1_angle_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo2_speed_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo2_angle_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo3_speed_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Servo3_angle_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")

        ############################################### Servo #############################################################
        self.servo_background.setStyleSheet("background-color : #f4f4f4;")
        self.Servo_title.setStyleSheet("color : white;" "background-color : black;")
        self.ser1_speed_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.ser1_angle_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                     "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.ser2_speed_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                          "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.ser2_angle_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                          "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.ser3_speed_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                          "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.ser3_angle_val.setStyleSheet("background-color : white;" "border-color : gray;"
                                          "border-style : solid;" "border-width : 0.5px;" "border-right : None;" "border-bottom : None;" "border-top-width : 1px;")
        self.reset_btn_3.setStyleSheet("border-image: url(Reset2x.png);")
        # self.line.setStyleSheet("color : red;" "border-color : red;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")

        ############################################### IMU Data #############################################################
        self.imu_title.setStyleSheet("color : white;" "background-color : black;")
        self.Message_ID2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Data_length2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Index2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.Time_stamp2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.gyro2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.gyro_range2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.acc2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.acc_range2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.mag_odr2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.mag_rep_xy2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.mag_rep_z2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.data_id2_1.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.gyrox2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.gyroy2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.gyroz2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.data_id2_2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.accx2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.accy2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.accz2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.acc_temp2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.data_id2_3.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.compassx2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.compassy2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.compassz2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")


        self.Message_ID2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.Data_length2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.Index2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.Time_stamp2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.gyro2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.gyro_range2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;" )
        self.acc2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.acc_range2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.mag_odr2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.mag_rep_xy2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.mag_rep_z2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.data_id2_1_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.gyrox2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.gyroy2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.gyroz2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.data_id2_2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.accx2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.accy2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.accz2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.acc_temp2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.data_id2_3_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.compassx2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.compassy2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.compassz2_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")


        # self.Data_length2_txt.setStyleSheet(
        #     "border-style : solid;" "border-width : 0.1px;" "border-left : None;" "border-right-width : 2px;")
        # self.Index2_txt.setStyleSheet(
        #     "border-style : solid;" "border-width : 0.5px;" "border-left : None;" "border-right-width : 2px;")
        # self.Time_stamp2_txt.setStyleSheet(
        #     "border-style : solid;" "border-width : 0.1px;" "border-left : None;" "border-right-width : 2px;")
        # self.gyro2_txt.setStyleSheet(
        #     "border-style : solid;" "border-width : 0.5px;" "border-left : None;" "border-right-width : 2px;")
        ############################################### IMU Controller #############################################################
        self.imu_controller.setStyleSheet("color : white;" "background-color : black;")
        self.reset_btn_4.setStyleSheet("border-image: url(Reset2x.png);")
        self.imu_controller_background.setStyleSheet("background-color : #f4f4f4;")
        self.pushButton_2.setStyleSheet("border-image: url(Apply.png);")
        self.pushButton_3.setStyleSheet("border-image: url(Apply.png);")
        self.mag_btn.setStyleSheet("border-image: url(Apply.png);")
        ############################################### NANO LiDAR ####################################################
        self.nanolidar.setStyleSheet("border-image: url(Nano Lidar2x.png);")
        self.Lidar_background.setStyleSheet("background-color : #f4f4f4;")
        self.lidar_background.setStyleSheet("background-color : white;")
        self.run_state_txt.setStyleSheet("border-image: url(Stop_022x.png);")
        self.start_btn.setStyleSheet("border-image: url(Start2x.png);")
        self.stop_btn.setStyleSheet("border-image: url(Stop2x.png);")
        self.lidar_system_title.setStyleSheet("color : white;" "background-color : black;")

        self.label_27.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"   "border-right : None" )
        self.label_28.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"    "border-right : None"  )
        self.label_24.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"   "border-right : None" )
        self.label_25.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"   "border-right : None" )
        self.label_26.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"   "border-right : None")
        self.label_23.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"   "border-right : None")

        self.index_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"  "border-left : None;" )
        self.systemstate_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"  "border-left : None;" )
        self.synchronous_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"  "border-left : None;" )
        self.angle_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"  "border-left : None;" )
        self.distance_length_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;"  "border-left : None;" )
        self.output_txt.setStyleSheet(
            "background-color : white;" "border-color : black;" "border-style : solid;" "border-width : 0.5px;" "border-bottom-width : 1px;" "border-left : None;" )

        ############################################### Gyro, Acc, Mag graph GUI #############################################################
        self.gyro_graph_txt.setStyleSheet("color : white;" "background-color : black;")
        self.acc_graph_txt.setStyleSheet("color : white;" "background-color : black;")
        self.mag_graph_txt.setStyleSheet("color : white;" "background-color : black;")

        self.groupBox_6.setStyleSheet( "border-style : solid;" "border-width : 0.5px;") # imu graph
        self.groupBox_9.setStyleSheet("border-style : solid;" "border-width : 0.5px;")  # acc graph
        self.groupBox_10.setStyleSheet("border-style : solid;" "border-width : 0.5px;")  # mag graph



        # self.gridLayout.setStyleSheet("border-style : solid;" "border-width : 2px;" "border-color : #FA8072;" "border-radius : 3px;")
        # self.groupBox.setStyleSheet("background-color : #EBE5E4;" "border-style : solid;" "border-width : 0.5px;")



        # self.groupBox_2.setStyleSheet("background-color : #E4EBEA;" "border-style : solid;" "border-width : 0.5px;")
        # self.groupBox_3.setStyleSheet("background-color : #E4EBEA;" "border-style : solid;" "border-width : 1px;")
        # self.groupBox_4.setStyleSheet("background-color : #E4EBEA;" "border-style : solid;" "border-width : 0.5px;")
        # self.groupBox_5.setStyleSheet("background-color : #E4EBEA;" "border-style : solid;" "border-width : 0.5px;")
        # self.groupBox_7.setStyleSheet("background-color : #E4EBEA;" "border-style : solid;" "border-width : 0.5px;")



        # self.reset_btn.setStyleSheet("background-color : #e1e1e1;" )




        # self.pushButton_2.setStyleSheet("background-color : #d7dddc;")
        # self.pushButton_2.setStyleSheet("background-color : #d7dddc;" "border-color : black;""border-style : solid;" "border-width : 2px;")
        # self.pushButton_3.setStyleSheet("background-color : #d7dddc;" "border-color : black;" "border-style : solid;" "border-width : 2px;")
        # self.mag_btn.setStyleSheet("background-color : #d7dddc;" "border-color : black;" "border-style : solid;" "border-width : 2px;")

        # self.buzzer_stop_btn.setStyleSheet("background-color : #d7dddc;" "border-color : black;" "border-style : solid;" "border-width : 2px;")



        # self.Move_btn.setStyleSheet('robomation.png')
        # self.Move_btn.setPixmap(QPixmap(pixmap).scaled(100,20))
        # self.Move_btn.setIcon(QtGui.QIcon("robomation.png").scaledToWidth(100,20))
        # self.Move_btn.setPixmap()

        # self.lidar_graph.adjustSize()



        # self.lidar_graph.enableAutoRange(axis="x")
        # self.lidar_graph.setMouseEnabled(x=True, y=True)


        # self.run_state_txt.setText("STOP")
        # self.run_state_txt.setFont(QtGui.QFont("Yu Gothic UI Semibold", 20))
        # self.run_state_txt.setStyleSheet("Color : red")

        ########## LiDAR Graph Setting ##########
        self.lidar_graph.setMouseEnabled(x=False, y=False)
        self.lidar_graph.disableAutoRange()

        self.lidar_graph.setXRange(-550, 550)
        self.lidar_graph.setYRange(-550, 550)
        # self.lidar_graph.setMouseEnabled()
        # self.lidar_graph.setTitle(title="Lidar GUI(mm)")
        self.lidar_graph.addLine(x=True)
        self.lidar_graph.addLine(y=True)
        # self.lidar_graph.showGrid(x=True, y=True)
        self.lidar_graph.setLabel('top', text='0°')
        self.lidar_graph.setLabel('bottom', text='180°')
        self.lidar_graph.setLabel('right', text='90°')
        self.lidar_graph.setLabel('left', text='240°')
        self.lidar_graph.setBackground('w')
        # self.robomation_png.scaledToWidth(10)

        self.theta = np.linspace(0, 2 * np.pi, 100)
        # self.radius = 1000
        self.a1 = 100 * np.cos(self.theta)
        self.b1 = 100 * np.sin(self.theta)
        self.a2 = 500 * np.cos(self.theta)
        self.b2 = 500 * np.sin(self.theta)
        self.a3 = 1200 * np.cos(self.theta)
        self.b3 = 1200 * np.sin(self.theta)
        self.a4 = 2000 * np.cos(self.theta)
        self.b4 = 2000 * np.sin(self.theta)
        self.a5 = 3000 * np.cos(self.theta)
        self.b5 = 3000 * np.sin(self.theta)
        self.a6 = 4000 * np.cos(self.theta)
        self.b6 = 4000 * np.sin(self.theta)
        self.a7 = 5000 * np.cos(self.theta)
        self.b7 = 5000 * np.sin(self.theta)
        self.a8 = 6000 * np.cos(self.theta)
        self.b8 = 6000 * np.sin(self.theta)
        self.a9 = 7000 * np.cos(self.theta)
        self.b9 = 7000 * np.sin(self.theta)
        self.a10 = 8000 * np.cos(self.theta)
        self.b10 = 8000 * np.sin(self.theta)
        self.a11 = 9000 * np.cos(self.theta)
        self.b11 = 9000 * np.sin(self.theta)
        self.a12 = 10000 * np.cos(self.theta)
        self.b12 = 10000 * np.sin(self.theta)
        self.a13 = 11000 * np.cos(self.theta)
        self.b13 = 11000 * np.sin(self.theta)



        self.a14 = [-8000, 8000]
        self.b14 = [13856, -13856]
        self.a15 = [-8000, 8000]
        self.b15 = [4619, -4619]
        self.a16 = [-8000, 8000]
        self.b16 = [-13856, 13856]
        self.a17 = [-8000, 8000]
        self.b17 = [-4619, 4619]


        self.lidar_graph.plot(self.a1, self.b1)
        self.lidar_graph.plot(self.a2, self.b2)
        self.lidar_graph.plot(self.a3, self.b3)
        self.lidar_graph.plot(self.a4, self.b4)
        self.lidar_graph.plot(self.a5, self.b5)
        self.lidar_graph.plot(self.a6, self.b6)
        self.lidar_graph.plot(self.a7, self.b7)
        self.lidar_graph.plot(self.a8, self.b8)
        self.lidar_graph.plot(self.a9, self.b9)
        self.lidar_graph.plot(self.a10, self.b10)
        self.lidar_graph.plot(self.a11, self.b11)
        self.lidar_graph.plot(self.a12, self.b12)
        self.lidar_graph.plot(self.a13, self.b13)
        self.lidar_graph.plot(self.a14, self.b14)
        self.lidar_graph.plot(self.a15, self.b15)
        self.lidar_graph.plot(self.a16, self.b16)
        self.lidar_graph.plot(self.a17, self.b17)

        self.curve = self.lidar_graph.plot(pen=None, symbol='o', symbolsize=0.001)
        self.curve.setSymbolSize(6)
        # self.curve = self.lidar_graph.plot(pen=None, symbol='+', symbolBrush=1)
        # self.angle_txt.setText(str(angle))
        # print(angle)



    ############################################### Button Click  ######################################################
        self.Move_btn.pressed.connect(self.Move_btn_pressed)
        self.Stop_btn.pressed.connect(self.Stop_btn_pressed)
        self.reset_btn.pressed.connect(self.Reset1_btn_pressed)
        self.reset_btn_2.pressed.connect(self.Reset2_btn_pressed)
        self.sound_stop_btn.pressed.connect(self.Sound_stop_btn_pressed)
        self.sound_btn.pressed.connect(self.Sound_btn_pressed)
        self.musical_btn.pressed.connect(self.Musical_btn_pressed)
        self.buzzer_btn.pressed.connect(self.Buzzer_btn_pressed)
        self.reset_btn_3.pressed.connect(self.Reset3_btn_pressed)
        self.reset_btn_4.pressed.connect(self.Reset4_btn_pressed)
        self.start_btn.pressed.connect(self.Lidar_start_pressed)
        self.stop_btn.pressed.connect(self.Lidar_stop_pressed)
        self.pushButton_2.pressed.connect(self.Gyro_apply_pressed)
        self.pushButton_3.pressed.connect(self.Acc_apply_pressed)
        self.mag_btn.pressed.connect(self.Mag_apply_pressed)


    def Move_btn_pressed(self):
        self.Move_btn.setStyleSheet("border-image: url(Move_in.png);")
    def Stop_btn_pressed(self):
        self.Stop_btn.setStyleSheet("border-image: url(Stop_in.png);")
    def Reset1_btn_pressed(self):
        self.reset_btn.setStyleSheet("border-image: url(Reset_in.png);")
    def Reset2_btn_pressed(self):
        self.reset_btn_2.setStyleSheet("border-image: url(Reset_in2x.png);")
    def Sound_stop_btn_pressed(self):
        self.sound_stop_btn.setStyleSheet("border-image: url(Stop_in2x.png);")
    def Sound_btn_pressed(self):
        self.sound_btn.setStyleSheet("border-image: url(Play_in2x.png);")
    def Musical_btn_pressed(self):
        self.musical_btn.setStyleSheet("border-image: url(Play_in2x.png);")
    def Buzzer_btn_pressed(self):
        self.buzzer_btn.setStyleSheet("border-image: url(Play_in2x.png);")
    def Reset3_btn_pressed(self):
        self.reset_btn_3.setStyleSheet("border-image: url(Reset_in2x.png);")
    def Reset4_btn_pressed(self):
        self.reset_btn_4.setStyleSheet("border-image: url(Reset_in2x.png);")
    def Lidar_start_pressed(self):
        self.start_btn.setStyleSheet("border-image: url(Start_in2x.png);")
    def Lidar_stop_pressed(self):
        self.stop_btn.setStyleSheet("border-image: url(Stop_in2x.png);")
    def Gyro_apply_pressed(self):
        self.pushButton_2.setStyleSheet("border-image: url(Apply_in.png);")
    def Acc_apply_pressed(self):
        self.pushButton_3.setStyleSheet("border-image: url(Apply_in.png);")
    def Mag_apply_pressed(self):
        self.mag_btn.setStyleSheet("border-image: url(Apply_in.png);")


    #############################################################################################

    ################# Gyro, Acc, Mag graph Update #####################

    def update(self):
        self.curve.setData(self.totalToFPosX, self.totalToFPosY)


    def show_data(self):
        self.ToFPosX0 = []
        self.ToFPosY0 = []

        self.ToFPosX1 = []
        self.ToFPosY1 = []

        self.ToFPosX2 = []
        self.ToFPosY2 = []

        self.ToFPosX3 = []
        self.ToFPosY3 = []

        self.totalToFPosX = []
        self.totalToFPosY = []

        try:

            for i in range(DataPos0):
                if ToFPosDataQ0[i] != 0xffff:
                    self.ToFPosX0.append(ToFPosDataQ0[i] * math.cos(-math.radians(((90 * i) / DataPos0) - 90)))
                    self.ToFPosY0.append(ToFPosDataQ0[i] * math.sin(-math.radians(((90 * i) / DataPos0) - 90)))


            for i in range(DataPos1):
                if ToFPosDataQ1[i] != 0xffff:
                    self.ToFPosX1.append(ToFPosDataQ1[i] * math.cos(-math.radians(((90 * i) / DataPos0))))
                    self.ToFPosY1.append(ToFPosDataQ1[i] * math.sin(-math.radians(((90 * i) / DataPos0))))

            for i in range(DataPos2):
                if ToFPosDataQ2[i] != 0xffff:
                    self.ToFPosX2.append(ToFPosDataQ2[i] * math.cos(-math.radians(((90 * i) / DataPos0) + 90)))
                    self.ToFPosY2.append(ToFPosDataQ2[i] * math.sin(-math.radians(((90 * i) / DataPos0) + 90)))

            for i in range(DataPos3):
                if ToFPosDataQ3[i] != 0xffff:
                    self.ToFPosX3.append(ToFPosDataQ3[i] * math.cos(-math.radians(((90 * i) / DataPos0) + 180)))
                    self.ToFPosY3.append(ToFPosDataQ3[i] * math.sin(-math.radians(((90 * i) / DataPos0) + 180)))

            self.totalToFPosX = self.ToFPosX0 + self.ToFPosX1 + self.ToFPosX2 + self.ToFPosX3
            self.totalToFPosY = self.ToFPosY0 + self.ToFPosY1 + self.ToFPosY2 + self.ToFPosY3
            ## plot
            self.update()
            # print(len(ToFPosDataQ0), len(ToFPosDataQ1), len(ToFPosDataQ2), len(ToFPosDataQ3))
            # print(self.ToFPosX0,end='\n')
        except:
            pass


    def update_line_gyro(self, i):

        try:

            y_g1 = imu_gyrox_n1_lst[-1]
            y_g2 = imu_gyroy_n1_lst[-1]
            y_g3 = imu_gyroz_n1_lst[-1]

            old_y_g1 = self.line_g1.get_ydata()
            old_y_g2 = self.line_g2.get_ydata()
            old_y_g3 = self.line_g3.get_ydata()
            # print(gyro)
            global gyro

            # print(gyro)
            if gyro != len(imu_gyrox_n1_lst):

                new_y_g1 = np.r_[old_y_g1[1:], y_g1]
                new_y_g2 = np.r_[old_y_g2[1:], y_g2]
                new_y_g3 = np.r_[old_y_g3[1:], y_g3]
                self.line_g1.set_ydata(new_y_g1)
                self.line_g2.set_ydata(new_y_g2)
                self.line_g3.set_ydata(new_y_g3)

                gyro = len(imu_gyrox_n1_lst)
                return [self.line_g1, self.line_g2, self.line_g3]

            else:

                return [self.line_g1, self.line_g2, self.line_g3]


        except:
            pass

    def update_line_acc(self, i):

        try:

            y_a1 = imu_accx_n1_lst[-1]
            y_a2 = imu_accy_n1_lst[-1]
            y_a3 = imu_accz_n1_lst[-1]

            old_y_a1 = self.line_a1.get_ydata()
            old_y_a2 = self.line_a2.get_ydata()
            old_y_a3 = self.line_a3.get_ydata()
            # old_y = self.line

            global acc
            # print(acc)
            # print(len(imu_accx_n1_lst))

            # print(gyro)
            if acc != len(imu_accx_n1_lst):
                new_y_a1 = np.r_[old_y_a1[1:], y_a1]
                new_y_a2 = np.r_[old_y_a2[1:], y_a2]
                new_y_a3 = np.r_[old_y_a3[1:], y_a3]
                # print("acc")
                # print(new_y_a1)
                self.line_a1.set_ydata(new_y_a1)
                self.line_a2.set_ydata(new_y_a2)
                self.line_a3.set_ydata(new_y_a3)
                acc = len(imu_accx_n1_lst)
                return [self.line_a1, self.line_a2, self.line_a3]
            else:

                return [self.line_a1, self.line_a2, self.line_a3]


        except:
            pass

    def update_line_mag(self, i):
        try:

            y_m1 = imu_compassx_n1_lst[-1]
            y_m2 = imu_compassy_n1_lst[-1]
            y_m3 = imu_compassz_n1_lst[-1]

            old_y_m1 = self.line_m1.get_ydata()
            old_y_m2 = self.line_m2.get_ydata()
            old_y_m3 = self.line_m3.get_ydata()
            # print(old_y_m1)
            global mag
            # print(mag)
            # print(len(imu_compassx_n1_lst))
            if mag != len(imu_compassx_n1_lst):
                new_y_m1 = np.r_[old_y_m1[1:], y_m1]
                new_y_m2 = np.r_[old_y_m2[1:], y_m2]
                new_y_m3 = np.r_[old_y_m3[1:], y_m3]
                # print("mag")
                # print(new_y_m1)
                self.line_m1.set_ydata(new_y_m1)
                self.line_m2.set_ydata(new_y_m2)
                self.line_m3.set_ydata(new_y_m3)
                mag = len(imu_compassx_n1_lst)

                return [self.line_m1, self.line_m2, self.line_m3]
            else:
                # print("aa")

                return [self.line_m1, self.line_m2, self.line_m3]


        except:
            pass

    ########## Pair Encoder Setting ##########
    def pps_click_func(self):
        if self.pps_mode.isChecked():
            self.pps = 1 #05ff89
            # self.PPS_mode_txt.setText("ON")
            # self.Tx_right_real = self.pps_val
            # self.Tx_left_real = self.pps_val
            if self.motor == 1:
                if abs(self.right_slider.value()) >= abs(self.left_slider.value()):
                    self.left_slider.setValue(self.right_slider.value())
                    self.left_val.setText(str(self.right_slider.value()))
                    self.Tx_left = self.right_slider.value()
                    self.Send_Start_Data()
                elif abs(self.right_slider.value()) < abs(self.left_slider.value()):
                    self.right_slider.setValue(self.left_slider.value())
                    self.right_val.setText(str(self.left_slider.value()))
                    self.Tx_right = self.left_slider.value()
                    self.Send_Start_Data()
            else:
                if abs(self.right_slider.value()) >= abs(self.left_slider.value()):
                    self.left_slider.setValue(self.right_slider.value())
                    self.left_val.setText(str(self.right_slider.value()))
                    self.Tx_left = self.right_slider.value()
                    # self.Send_Start_Data()
                elif abs(self.right_slider.value()) < abs(self.left_slider.value()):
                    self.right_slider.setValue(self.left_slider.value())
                    self.right_val.setText(str(self.left_slider.value()))
                    self.Tx_right = self.left_slider.value()
                    # self.Send_Start_Data()


            print("Check")

        else:
            self.pps = 0

            self.Tx_right_real = self.right_slider.value()
            self.Tx_left_real = self.left_slider.value()
            self.Tx_right = self.right_slider.value()
            self.Tx_left = self.left_slider.value()
            # self.Send_Start_Data()
            time.sleep(0.01)
            print("None Check")

    ########## Keyboard Controller ##########

    def keyboard_func(self):
        global keyboard_val, up, down, left, right
        if self.keyboard.isChecked():
            keyboard_val = 1
            print("checked")
            # print(keyboard_val)


        else:
            print("None Checked")
            keyboard_val = 0
            up = 0
            down = 0
            right = 0
            left = 0
            # print(keyboard_val)

    def keyboard_control(self):
        global up, down, right, left
        # self.keyboard_slider_val = self.keyboard_slider.value()
        # print(self.keyboard_slider_val)
        if up == 1 and right ==0 and left ==0:

            print("Up")
            # self.Tx_right = 0x0001 * self.keyboard_slider_val
            # self.Tx_left =  0x0001 * self.keyboard_slider_val
            self.Tx_right = 0x01f4
            self.Tx_left = 0x01f4
            # print(self.keyboard_slider_val)
            # print(self.Tx_right)
            # print(self.Tx_left)
            self.Send_Start_Data()
        elif up == 1 and right ==1 and left == 0:
            self.Tx_right = 0x00fa
            self.Tx_left = 0x01f4
            self.Send_Start_Data()
        elif up == 1 and right == 0 and left == 1:
            self.Tx_right = 0x01f4
            self.Tx_left = 0x00fa
            self.Send_Start_Data()


        if down == 1 and right == 0 and left == 0:

            self.Tx_right = -0x01f4
            self.Tx_left = -0x01f4
            self.Send_Start_Data()
        elif down == 1 and right == 1 and left == 0:

            self.Tx_right = -0x00fa
            self.Tx_left = -0x01f4
            self.Send_Start_Data()
        elif down == 1 and right == 0 and left == 1:

            self.Tx_right = -0x01f4
            self.Tx_left = -0x00fa
            self.Send_Start_Data()

        if right == 1 and left == 0 and up == 0 and down == 0:
            self.Tx_right = -0x01f4
            self.Tx_left = 0x01f4
            self.Send_Start_Data()
        if left == 1 and right == 0 and up == 0 and down == 0:
            self.Tx_right = 0x01f4
            self.Tx_left = -0x01f4
            self.Send_Start_Data()

        if up == 0 and down == 0 and right==0 and left == 0:
            self.Tx_right = 0x0000
            self.Tx_left = -0x0000
            self.Send_Start_Data()
        # if right == 1:
        #     self.Tx_right = 0x00fa
        #     self.Tx_left = 0x01f4
        #     self.Send_Start_Data()
        # if left == 1:
        #     self.Tx_right = 0x01f4
        #     self.Tx_left = 0x00fa
        #     self.Send_Start_Data()
        # if up == 1 and right == 1:
        #     self.Tx_right = 0x01f4
        #     self.Tx_left = 0x01f4
        #     self.Send_Start_Data()

        time.sleep(0.01)



    # def Write(self):
    #     pass

    ########## right Encoder control ##########
    def move(self):
        if self.continous == 1:
            self.Tx_right_real = self.right_slider.value()
            self.Tx_right = self.right_slider.value()
            self.right_val.setText(str(self.right_slider.value()))

            time.sleep(0.001)

            if self.pps == 0:
                if self.motor == 1:

                    self.Send_Start_Data()


            elif self.pps == 1:
                self.Tx_right_real = self.right_slider.value()
                self.Tx_left_real = self.right_slider.value()
                self.left_slider.setValue(self.right_slider.value())
                self.right_val.setText(str(self.right_slider.value()))
                self.left_val.setText(str(self.right_slider.value()))

                self.Tx_right = self.right_slider.value()
                self.Tx_left = self.right_slider.value()
                time.sleep(0.001)
                if self.motor == 1:
                    self.Send_Start_Data()

        elif self.distance == 1:
            if self.pps == 0:
                self.Tx_right_real = self.right_slider.value()
                self.right_val.setText(str(self.right_slider.value()))

                time.sleep(0.001)

            elif self.pps == 1:
                self.Tx_right_real = self.right_slider.value()
                self.Tx_left_real = self.right_slider.value()
                self.left_slider.setValue(self.right_slider.value())
                self.right_val.setText(str(self.right_slider.value()))
                self.left_val.setText(str(self.right_slider.value()))

    ########## Left Encoder control ##########

    def move2(self):
        if self.continous == 1:
            self.Tx_left_real = self.left_slider.value()
            self.left_val.setText(str(self.left_slider.value()))
            self.Tx_left = self.left_slider.value()

            time.sleep(0.001)

            if self.pps == 0:
                if self.motor == 1:

                    self.Send_Start_Data()

            elif self.pps == 1:
                self.Tx_left_real = self.left_slider.value()
                self.Tx_right_real = self.left_slider.value()
                self.right_slider.setValue(self.left_slider.value())
                self.right_val.setText(str(self.left_slider.value()))
                self.left_val.setText(str(self.left_slider.value()))
                self.Tx_right = self.left_slider.value()
                self.Tx_left = self.left_slider.value()
                time.sleep(0.001)
                if self.motor == 1:
                    self.Send_Start_Data()

        elif self.distance == 1:
            if self.pps == 0:
                self.Tx_left_real = self.left_slider.value()
                self.left_val.setText(str(self.left_slider.value()))

            elif self.pps == 1:
                self.Tx_left_real = self.left_slider.value()
                self.Tx_right_real = self.left_slider.value()
                self.right_slider.setValue(self.left_slider.value())
                self.right_val.setText(str(self.left_slider.value()))
                self.left_val.setText(str(self.left_slider.value()))
                print("check")


    ########## Encoder Reset ##########
    def reset(self):
        # MUTE,BEEP,BEEP2,BEEP3,BEEP_REP,BEEP_RND,BEEP_RND_REP,SNORE,SNORE_REP,SIREN,SIREN_REP,ENGINE,ENGINE_REP,FART_A,FART_B,NOISE,NOISE_REP,WISTLE,CHOP_CHOP,CHOP_CHOP_REP
        # R2D2,DIBIDIBIDIP,SIMPLE_MELODY,FINISH,HAPPY_MOOD,ANGRY_MOOD,SAD_MOOD,SLEEP_MOOD,TOY_MARCH,BIRTHDAY_SONG
        self.motor_operating.setStyleSheet("border-image: url(Stop_01.png);")
        self.reset_btn.setStyleSheet("border-image: url(Reset.png);")
        self.right_slider.setValue(0x00)
        self.left_slider.setValue(0x00)
        self.Tx_right = 0x00
        self.Tx_left = 0x00
        self.motor = 0


        self.distance_range_slider.setValue(0x00)

        # self.PPS_mode_txt

        # self.sound_comboBox.setItemText(1)


        self.Tx_motor_mode = 0x01
        self.Tx_distance_range = 0x00

        if self.pps_mode.isChecked():
            self.pps = 1
        else:
            self.pps = 0
        print("Reset")

        self.Send_Start_Data()

    ########## Sound Reset ##########
    def Sound_reset(self):
        print("Sound reset")
        self.reset_btn_2.setStyleSheet("border-image: url(Reset2x.png);")
        self.buzzer_slider.setValue(0x00)
        self.musical_slider.setValue(0x00)
        self.sound_comboBox.setCurrentText("MUTE")
        self.Tx_sound_clip = 0x00
        self.Tx_buzzer = 0x00
        self.Tx_buzzer_high = 0x00
        self.Tx_buzzer_middle = 0x00
        self.Tx_buzzer_low = 0x00
        self.Tx_musical = 0x00


        self.Send_Start_Data()

    ########## Servo Reset ##########
    def Servo_reset(self):
        print("Servo reset")
        self.reset_btn_3.setStyleSheet("border-image: url(Reset2x.png);")
        self.servo1_speed_slider.setValue(0x00)
        self.servo2_speed_slider.setValue(0x00)
        self.servo3_speed_slider.setValue(0x00)
        self.servo1_angle_slider.setValue(0x5a)
        self.servo2_angle_slider.setValue(0x5a)
        self.servo3_angle_slider.setValue(0x5a)

        self.Send_Start_Data()

    ########## IMU Controller Reset ##########
    def IMU_reset(self):
        print("IMU_reset")
        self.reset_btn_4.setStyleSheet("border-image: url(Reset2x.png);")
        self.Gyro_ODR_BW.setCurrentText("Default")
        self.Gyro_range.setCurrentText("Default")
        self.ACC_ODR_BW.setCurrentText("Default")
        self.ACC_range.setCurrentText("Default")
        self.MAG_RECOMMEND.setCurrentText("Default")
        self.MAG_ODR.setCurrentText("Default")
        self.mag_rep_xy_slider.setValue(47)
        self.mag_rep_z_slider.setValue(83)

        ####
        self.gyro_range = self.gyro_range_real
        self.gyro_odr = self.gyro_odr_real
        self.gyro_index += 1
        # print("self.gyro_index", self.gyro_index)

        self.acc_index += 1
        self.acc_range = self.acc_range_real
        self.acc_odr = self.acc_odr_real

        self.mag_index += 1
        self.mag_rep_z = self.mag_rep_z_real
        self.mag_rep_xy = self.mag_rep_xy_real
        self.mag_odr = self.mag_odr_real
        self.Send_IMU_Data()
        ####

        # self.Send_Start_Data()
        self.Send_IMU_Data()



    ########## Motor Controller ##########
    # If you click button, you can control your beagle
    def Move_btn_func(self):
        print("Move btn Clicked")
        self.Move_btn.setStyleSheet("border-image: url(Move.png);")
        self.motor_operating.setStyleSheet("border-image: url(Running_01.png);")
        # self.Stop_btn.setStyleSheet(
        #     "background-color : #d7dddc;" "border-color : black;" "border-style : solid;" "border-width : 2px;" "font-family:Yu Gothic UI Semibold")

        self.Tx_right = 0x00
        self.Tx_left = 0x00
        self.Send_Start_Data()
        self.motor = 1
        if self.continous == 1:
            print("continous")
            self.Tx_right = self.Tx_right_real
            self.Tx_left = self.Tx_left_real
            # print(self.Tx_right_real, self.Tx_left_real)
            # print(self.Tx_right , self.Tx_left)
            self.Send_Start_Data()

        if self.distance == 1:
            print("distance")
            self.Tx_distance_range = self.distance_range_slider.value()
            self.Tx_distance_id += 1
            self.Tx_right = self.Tx_right_real
            self.Tx_left = self.Tx_left_real
            self.Send_Start_Data()
            time.sleep(0.001)
            self.Send_Start_Data()

    ########## Stop button on Motor Controller ##########
    def Stop_btn_func(self):
        print("Stop btn Clicked")

        self.Stop_btn.setStyleSheet("border-image: url(Stop.png);")
        self.motor_operating.setStyleSheet("border-image: url(Stop_01.png);")
        # self.Stop_btn.setStyleSheet(
            # "background-color : red;" "border-color : black;" "border-style : solid;" "border-width : 2px;" "font-family:Yu Gothic UI Semibold" "border-image: url(robomation.png);")
        self.motor = 0
        self.Tx_right = 0x00
        self.Tx_left = 0x00
        self.Send_Start_Data()

    ########## Distance mode Text ##########
    def distance_value_func(self):
        self.distance_mode_txt.setText(str(self.distance_range_slider.value()))
        time.sleep(0.001)

    ########## Choose Keep or Sleep ##########
    # Sleep is default
    def data_length_func(self):
        if self.keep_radio.isChecked():
            print("keep")
            self.Tx_motor_mode = 0x04
            self.Tx_right = 0x00
            self.Tx_left = 0x00
        elif self.sleep_radio.isChecked():
            self.Tx_motor_mode = 0x00
            self.Tx_right = 0x00
            self.Tx_left = 0x00
        self.Send_Start_Data()

    # mode setting (continous mode | distance mode)
    def Mode_func(self):
        if self.continous_radio.isChecked():
            print("continous")
            self.continous = 1
            self.distance = 0
        elif self.distance_radio.isChecked():
            print("distance")
            self.continous = 0
            self.distance = 1
            self.Tx_right = 0x00
            self.Tx_left = 0x00
            self.Send_Start_Data()

    ########## Buzzer ##########
    def buzzer_func(self):
        self.Tx_buzzer_high = 0x00
        self.Tx_buzzer_middle = 0x00
        self.Tx_buzzer_low = 0x00

        self.Tx_buzzer = self.buzzer_slider.value()
        self.Tx_buzzer_high_real = (self.Tx_buzzer & 0xff0000) >> 16
        self.Tx_buzzer_middle_real = (self.Tx_buzzer & 0xff00) >> 8
        self.Tx_buzzer_low_real = self.Tx_buzzer & 0xff
        self.buzzer_val.setText(str(self.buzzer_slider.value()))
        if self.buzzer == 1:

            self.Tx_buzzer_high = (self.Tx_buzzer & 0xff0000) >> 16
            self.Tx_buzzer_middle = (self.Tx_buzzer & 0xff00) >> 8
            self.Tx_buzzer_low = self.Tx_buzzer & 0xff
            self.buzzer_val.setText(str(self.buzzer_slider.value()))
            self.Send_Start_Data()

        time.sleep(0.001)


    def Buzzer_Set_func(self):

        self.buzzer_btn.setStyleSheet("border-image: url(Play2x.png);")



        self.soundclip = 0
        self.musical = 0
        self.buzzer = 1

        if self.Tx_sound_clip != 0x00:
            self.Tx_sound_clip = 0x00
            self.Tx_musical = 0x00
            self.Tx_buzzer_high = self.Tx_buzzer_high_real
            self.Tx_buzzer_middle = self.Tx_buzzer_middle_real
            self.Tx_buzzer_low = self.Tx_buzzer_low_real
            print(self.Tx_buzzer_high, self.Tx_buzzer_middle, self.Tx_buzzer_low)

            self.Send_Start_Data()
            time.sleep(0.1)



        self.Tx_sound_clip = 0x00
        self.Tx_musical = 0x00
        self.Tx_buzzer_high = self.Tx_buzzer_high_real
        self.Tx_buzzer_middle = self.Tx_buzzer_middle_real
        self.Tx_buzzer_low = self.Tx_buzzer_low_real
        print(self.Tx_buzzer_high,self.Tx_buzzer_middle,self.Tx_buzzer_low)

        self.Send_Start_Data()

    ########## Musical ##########

    def musical_func(self):
        self.Tx_musical_real = self.musical_slider.value()
        # print(self.Tx_musical_real)
        self.musical_val.setText(str(self.musical_slider.value()))
        if self.musical == 1:
            self.Tx_musical = self.musical_slider.value()
            self.musical_val.setText(str(self.musical_slider.value()))
            self.Send_Start_Data()


        time.sleep(0.001)

    def Musical_Set_func(self):

        # print(self.Tx_sound_clip)

        self.musical_btn.setStyleSheet("border-image: url(Play2x.png);")

        self.Send_Start_Data()

        self.soundclip = 0
        self.musical = 1
        self.buzzer = 0

        if self.Tx_sound_clip != 0x00:
            self.Tx_sound_clip = 0x00
            self.Tx_buzzer_high = 0x00
            self.Tx_buzzer_middle = 0x00
            self.Tx_buzzer_low = 0x00
            self.Tx_musical = self.Tx_musical_real
            self.Send_Start_Data()
            time.sleep(0.1)

        self.Tx_sound_clip = 0x00
        self.Tx_buzzer_high = 0x00
        self.Tx_buzzer_middle = 0x00
        self.Tx_buzzer_low = 0x00

        # self.Send_Start_Data()

        self.Tx_musical = self.Tx_musical_real
        print(self.Tx_musical)

        self.Send_Start_Data()

    ########## Servo ##########
    def servo_speed_func(self):
        self.Tx_servo1_speed = self.servo1_speed_slider.value()
        self.Tx_servo2_speed = self.servo2_speed_slider.value()
        self.Tx_servo3_speed = self.servo3_speed_slider.value()
        self.ser1_speed_val.setText(str(self.servo1_speed_slider.value()))
        self.ser2_speed_val.setText(str(self.servo2_speed_slider.value()))
        self.ser3_speed_val.setText(str(self.servo3_speed_slider.value()))
        time.sleep(0.001)
        self.Send_Start_Data()

    def servo_angle_func(self):

        if self.checkBox_1.isChecked():
            self.Tx_servo1_angle = 0x00
            self.ser1_angle_val.setText(str(0x00))
        else:
            self.Tx_servo1_angle = self.servo1_angle_slider.value()
            self.ser1_angle_val.setText(str(self.servo1_angle_slider.value()))

        if self.checkBox_2.isChecked():
            self.Tx_servo2_angle = 0x00
            self.ser2_angle_val.setText(str(0))
        else:
            self.Tx_servo2_angle = self.servo2_angle_slider.value()
            self.ser2_angle_val.setText(str(self.servo2_angle_slider.value()))

        if self.checkBox_3.isChecked():
            self.Tx_servo3_angle = 0x00
            self.ser3_angle_val.setText(str(0))
        else:
            self.Tx_servo3_angle = self.servo3_angle_slider.value()
            self.ser3_angle_val.setText(str(self.servo3_angle_slider.value()))

        # self.Tx_servo2_angle = self.servo2_angle_slider.value()
        # self.Tx_servo3_angle = self.servo3_angle_slider.value()
        # self.ser1_angle_val.setText(str(self.servo1_angle_slider.value()))
        # self.ser2_angle_val.setText(str(self.servo2_angle_slider.value()))
        # self.ser3_angle_val.setText(str(self.servo3_angle_slider.value()))
        time.sleep(0.001)
        self.Send_Start_Data()

    ########## Disable Servo Motor ##########
    def S1_EN_func(self):

        if self.checkBox_1.isChecked():
            self.Tx_servo1_angle = 0x00
            self.ser1_angle_val.setText(str(0x00))
        else:
            self.Tx_servo1_angle = self.servo1_angle_slider.value()
            self.ser1_angle_val.setText(str(self.servo1_angle_slider.value()))

        if self.checkBox_2.isChecked():
            self.Tx_servo2_angle = 0x00
            self.ser2_angle_val.setText(str(0X00))
        else:
            self.Tx_servo2_angle = self.servo2_angle_slider.value()
            self.ser2_angle_val.setText(str(self.servo2_angle_slider.value()))

        if self.checkBox_3.isChecked():
            self.Tx_servo3_angle = 0x00
            self.ser3_angle_val.setText(str(0X00))
        else:
            self.Tx_servo3_angle = self.servo3_angle_slider.value()
            self.ser3_angle_val.setText(str(self.servo3_angle_slider.value()))


        time.sleep(0.001)
        self.Send_Start_Data()

    ########## Select the Gyro range from the combo box ##########
    def Gyro_range_func(self):
        if self.Gyro_range.currentText() == "Default":  # 250DPS
            self.gyro_range_real = 0x00
            # print(self.gyro_range)
        elif self.Gyro_range.currentText() == "125DPS":
            self.gyro_range_real = 0x01
            # print(self.gyro_range)
        elif self.Gyro_range.currentText() == "250DPS":
            self.gyro_range_real = 0x02
            # print(self.gyro_range)
        elif self.Gyro_range.currentText() == "500DPS":
            self.gyro_range_real = 0x04
            # print(self.gyro_range)
        elif self.Gyro_range.currentText() == "1000DPS":
            self.gyro_range_real = 0x08
            # print(self.gyro_range)
        elif self.Gyro_range.currentText() == "2000DPS":
            self.gyro_range_real = 0x10
            # print(self.gyro_range)


        # self.Gyro_Set_func()

    ########## Select the Gyro ODR from the combo box ##########
    def Gyro_ODR_func(self):
        if self.Gyro_ODR_BW.currentText() == "Default":  # ODR-100HZ/BW-12HZ
            self.gyro_odr_real = 0x00
            # print(self.gyro_odr)
        elif self.Gyro_ODR_BW.currentText() == "ODR100Hz/BW12Hz":
            self.gyro_odr_real = 0x20
            # print(self.gyro_odr)
        elif self.Gyro_ODR_BW.currentText() == "ODR100Hz/BW32Hz":
            self.gyro_odr_real = 0x80
            # print(self.gyro_odr)
        elif self.Gyro_ODR_BW.currentText() == "ODR200Hz/BW64Hz":
            self.gyro_odr_real = 0x40
            # print(self.gyro_odr)
        elif self.Gyro_ODR_BW.currentText() == "ODR200Hz/BW23Hz":
            self.gyro_odr_real = 0x10
            # print(self.gyro_odr)
        # elif self.Gyro_ODR_BW.currentText() == "ODR400Hz/BW47Hz":
        #     self.gyro_odr = 0x08
        #     print(self.gyro_odr)
        # elif self.Gyro_ODR_BW.currentText() == "ODR1000Hz/BW47Hz":
        #     self.gyro_odr = 0x04
        #     print(self.gyro_odr)
        # elif self.Gyro_ODR_BW.currentText() == "ODR2000Hz/BW230Hz":
        #     self.gyro_odr = 0x02
        #     print(self.gyro_odr)
        # elif self.Gyro_ODR_BW.currentText() == "ODR2000Hz/BW532Hz":
        #     self.gyro_odr = 0x01
        #     print(self.gyro_odr)



    ########## Gyro Set function ##########
    def Gyro_Set_func(self):
        print("Gyro_Set")
        self.pushButton_2.setStyleSheet("border-image: url(Apply.png);")

        self.gyro_range = self.gyro_range_real
        self.gyro_odr = self.gyro_odr_real
        self.gyro_index += 1
        # print("self.gyro_index", self.gyro_index)
        self.Send_IMU_Data()

    ########## Select the Acc ODR from the combo box ##########
    def ACC_ODR_func(self):
        if self.ACC_ODR_BW.currentText() == "Default":  # ODR-125HZ/BW-62.5HZ
            self.acc_odr_real = 0x00

        elif self.ACC_ODR_BW.currentText() == "ODR15.63Hz/BW7.81Hz":
            self.acc_odr_real = 0x01

        elif self.ACC_ODR_BW.currentText() == "ODR31.25Hz/BW15.63Hz":
            self.acc_odr_real = 0x02

        elif self.ACC_ODR_BW.currentText() == "ODR62.5Hz/BW31.25Hz":
            self.acc_odr_real = 0x04

        elif self.ACC_ODR_BW.currentText() == "ODR125Hz/BW62.5Hz":
            self.acc_odr_real = 0x08

        elif self.ACC_ODR_BW.currentText() == "ODR250Hz/BW125Hz":
            self.acc_odr_real = 0x10

        # elif self.ACC_ODR_BW.currentText() == "ODR500Hz/BW250Hz":
        #     self.acc_odr = 0x20
        #     print(self.acc_odr)
        # elif self.ACC_ODR_BW.currentText() == "ODR1000Hz/BW500Hz":
        #     self.acc_odr = 0x40
        #     print(self.acc_odr)
        # elif self.ACC_ODR_BW.currentText() == "ODR2000Hz/BW1000Hz":
        #     self.acc_odr = 0x80
        #     print(self.acc_odr)

    ########## Select the Acc Range from the combo box ##########
    def ACC_range_func(self):
        if self.ACC_range.currentText() == "Default":  # 2G
            self.acc_range_real = 0x00

        elif self.ACC_range.currentText() == "2G":
            self.acc_range_real = 0x01

        elif self.ACC_range.currentText() == "4G":
            self.acc_range_real = 0x02

        elif self.ACC_range.currentText() == "8G":
            self.acc_range_real = 0x04

        elif self.ACC_range.currentText() == "16G":
            self.acc_range_real = 0x08


        # self.Acc_Set_func()

    ########## Acc Set Function ##########
    def Acc_Set_func(self):
        print("Acc_Set")

        self.pushButton_3.setStyleSheet("border-image: url(Apply.png);")

        self.acc_index += 1
        self.acc_range = self.acc_range_real
        self.acc_odr = self.acc_odr_real
        self.Send_IMU_Data()

    ########## Select the Mag ODR from the combo box ##########
    def MAG_ODR_func(self):
        if self.MAG_ODR.currentText() == "Default":  # ODR-20HZ
            self.mag_odr_real = 0x00
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "2Hz":
            self.mag_odr_real = 0x01
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "6Hz":
            self.mag_odr_real = 0x02
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "8Hz":
            self.mag_odr_real = 0x04
        elif self.MAG_ODR.currentText() == "10Hz":
            self.mag_odr_real = 0x08
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "15Hz":
            self.mag_odr_real = 0x10
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "20Hz":
            self.mag_odr_real = 0x20
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "25Hz":
            self.mag_odr_real = 0x40
            # print(self.mag_odr2)
        elif self.MAG_ODR.currentText() == "30Hz":
            self.mag_odr_real = 0x80
            # print(self.mag_odr2)

    ########## Select the Mag Rep XY from the combo box ##########
    def MAG_Rep_XY_func(self):
        # self.mag_rep_xy_txt.setText(str((self.mag_rep_xy_slider.value()-1) * 2 + 1))
        self.mag_rep_xy_txt.setText(str(self.mag_rep_xy_slider.value()))
        print((self.mag_rep_xy_slider.value() -1)/2)
        self.mag_rep_xy_real = int((self.mag_rep_xy_slider.value() -1)/2)
        time.sleep(0.001)

    ########## Select the Mag Rep Z from the combo box ##########
    def MAG_Rep_Z_func(self):
        self.mag_rep_z_txt.setText(str(self.mag_rep_z_slider.value()))
        self.mag_rep_z_real = self.mag_rep_z_slider.value() -1
        time.sleep(0.001)

    ########## Select Mag function ##########
    def Mag_Set_func(self):
        print("Mag_Set")

        self.mag_btn.setStyleSheet("border-image: url(Apply.png);")
        self.mag_index += 1
        self.mag_rep_z = self.mag_rep_z_real
        self.mag_rep_xy = self.mag_rep_xy_real
        self.mag_odr = self.mag_odr_real

        self.Send_IMU_Data()
        time.sleep(0.1)

    ########## Mag Recommend ##########
    def Mag_recommend_func(self):
        if self.MAG_RECOMMEND.currentText() == "Default":
            # self.mag_rep_xy_real = 0x23
            # self.mag_rep_z_real = 0x82
            # self.mag_odr = 0x20
            self.mag_rep_xy_slider.setValue(47)
            self.mag_rep_z_slider.setValue(83)
            self.MAG_ODR.setCurrentText("20Hz")


        elif self.MAG_RECOMMEND.currentText() == "Low power preset":
            # self.mag_rep_xy_real = 0x01
            # self.mag_rep_z_real = 0x02
            # self.mag_odr_real = 0x10
            self.mag_rep_xy_slider.setValue(3)
            self.mag_rep_z_slider.setValue(3)
            self.MAG_ODR.setCurrentText("10Hz")
            # self.MAG_ODR.setCurrentText("")



        elif self.MAG_RECOMMEND.currentText() == "Regular preset":
            # self.mag_rep_xy_real = 0x05
            # self.mag_rep_z_real = 0x14
            # self.mag_odr_real = 0x10
            self.mag_rep_xy_slider.setValue(9)
            self.mag_rep_z_slider.setValue(15)
            self.MAG_ODR.setCurrentText("10Hz")


        elif self.MAG_RECOMMEND.currentText() == "Enhanced regular preset":
            # self.mag_rep_xy_real = 0x08
            # self.mag_rep_z_real = 0x26
            # self.mag_odr_real = 0x10
            self.mag_rep_xy_slider.setValue(15)
            self.mag_rep_z_slider.setValue(27)
            self.MAG_ODR.setCurrentText("10Hz")


        elif self.MAG_RECOMMEND.currentText() == "High accuracy preset":
            # self.mag_rep_xy_real = 0x24
            # self.mag_rep_z_real = 0x82
            # self.mag_odr_real = 0x20
            self.mag_rep_xy_slider.setValue(47)
            self.mag_rep_z_slider.setValue(83)
            self.MAG_ODR.setCurrentText("20Hz")

    ########## Clip ##########
    def clip(self):
        if self.sound_comboBox.currentText() == "MUTE":
            self.Tx_sound_clip_real = 0x00
        elif self.sound_comboBox.currentText() == "BEEP":
            self.Tx_sound_clip_real = 0x01
        elif self.sound_comboBox.currentText() == "BEEP2":
            self.Tx_sound_clip_real = 0x02
        elif self.sound_comboBox.currentText() == "BEEP3":
            self.Tx_sound_clip_real = 0x03
        elif self.sound_comboBox.currentText() == "BEEP_REP":
            self.Tx_sound_clip_real = 0x04
        elif self.sound_comboBox.currentText() == "BEEP_RND":
            self.Tx_sound_clip_real = 0x05
        elif self.sound_comboBox.currentText() == "BEEP_RND_REP":
            self.Tx_sound_clip_real = 0x06
        elif self.sound_comboBox.currentText() == "SNORE":
            self.Tx_sound_clip_real = 0x07
        elif self.sound_comboBox.currentText() == "SNORE_REP":
            self.Tx_sound_clip_real = 0x08
        elif self.sound_comboBox.currentText() == "SIREN":
            self.Tx_sound_clip_real = 0x09
        elif self.sound_comboBox.currentText() == "SIREN_REP":
            self.Tx_sound_clip_real = 0x0A
        elif self.sound_comboBox.currentText() == "ENGINE":
            self.Tx_sound_clip_real = 0x0B
        elif self.sound_comboBox.currentText() == "ENGINE_REP":
            self.Tx_sound_clip_real = 0x0C
        elif self.sound_comboBox.currentText() == "FART_A":
            self.Tx_sound_clip_real = 0x0D
        elif self.sound_comboBox.currentText() == "FART_B":
            self.Tx_sound_clip_real = 0x0E
        elif self.sound_comboBox.currentText() == "NOISE":
            self.Tx_sound_clip_real = 0x0F
        elif self.sound_comboBox.currentText() == "NOISE_REP":
            self.Tx_sound_clip_real = 0x10
        elif self.sound_comboBox.currentText() == "WISTLE":
            self.Tx_sound_clip_real = 0x11
        elif self.sound_comboBox.currentText() == "CHOP_CHOP":
            self.Tx_sound_clip_real = 0x12
        elif self.sound_comboBox.currentText() == "CHOP_CHOP_REP":
            self.Tx_sound_clip_real = 0x13
        elif self.sound_comboBox.currentText() == "R2D2":
            self.Tx_sound_clip_real = 0x20
        elif self.sound_comboBox.currentText() == "DIBIDIBIDIP":
            self.Tx_sound_clip_real = 0x21
        elif self.sound_comboBox.currentText() == "SIMPLE_MELODY":
            self.Tx_sound_clip_real = 0x22
        elif self.sound_comboBox.currentText() == "FINISH":
            self.Tx_sound_clip_real = 0x23
        elif self.sound_comboBox.currentText() == "HAPPY_MOOD":
            self.Tx_sound_clip_real = 0x30
        elif self.sound_comboBox.currentText() == "ANGRY_MOOD":
            self.Tx_sound_clip_real = 0x31
        elif self.sound_comboBox.currentText() == "SAD_MOOD":
            self.Tx_sound_clip_real = 0x32
        elif self.sound_comboBox.currentText() == "SLEEP_MOOD":
            self.Tx_sound_clip_real = 0x33
        elif self.sound_comboBox.currentText() == "TOY_MARCH":
            self.Tx_sound_clip_real = 0x34
        elif self.sound_comboBox.currentText() == "BIRTHDAY_SONG":
            self.Tx_sound_clip_real = 0x35

        print(self.sound_comboBox.currentText())

    ########## Sound Clip Toggle ##########
    def Sound_Set_func(self):
        if self.toggle == 0:
            self.msb = 0
            self.toggle = 1
        else:
            self.msb = 0x80
            self.toggle = 0

        self.Tx_sound_clip = self.Tx_sound_clip_real + self.msb

        print(self.Tx_sound_clip)
        self.sound_btn.setStyleSheet("border-image: url(Play2x.png);")
        self.Send_Start_Data()

    ########## Stop Sound, Musical, Buzzer ##########
    def Sound_Stop_func(self):
        print("stop")

        self.sound_stop_btn.setStyleSheet("border-image: url(Stop2x.png);")

        self.soundclip = 0
        self.musical = 0
        self.buzzer = 0

        self.Tx_sound_clip = 0x00
        print(self.Tx_sound_clip)
        self.Tx_musical = 0x00
        self.Tx_buzzer_high = 0x00
        self.Tx_buzzer_middle = 0x00
        self.Tx_buzzer_low = 0x00

        self.Send_Start_Data()

    # show data on the GUI

    ########## Tx IMU ##########
    def Send_IMU_Data(self):
        global imu_index
        imu_index += 1
        if imu_index > 255:
            imu_index = 0
        if serial_init.ser.writable():
            start_imu = bytearray()
            start_imu.append(0x52)
            start_imu.append(0x49)
            start_imu.append(0x0A)

            start_imu.append(0x20)
            start_imu.append(0x08)
            # start_imu.append(0x00)
            start_imu.append((self.gyro_index & 0x03) + ((self.acc_index & 0x03) * 4) + ((self.mag_index & 0x03) * 16))
            # print("start_imu_index",start_imu[-1])
            start_imu.append(self.gyro_odr)
            start_imu.append(self.gyro_range)
            start_imu.append(self.acc_odr)
            start_imu.append(self.acc_range)
            start_imu.append(self.mag_odr)
            start_imu.append(self.mag_rep_xy)
            start_imu.append(self.mag_rep_z)

            CK_A = 0
            CK_B = 0

            for i in range(len(start_imu)):
                CK_A = CK_A + start_imu[i]
                CK_B = CK_A + CK_B

            CK_A = CK_A & 0x000000ff
            CK_B = CK_B & 0x000000ff
            start_imu.append(CK_A)
            start_imu.append(CK_B)

            serial_init.ser.write(start_imu)
            # print(start_imu)
            time.sleep(0.001)

    ########## Tx Beagle Data ##########
    def Send_Start_Data(self):
        global n, d
        n = n + 1
        d = d + 1
        # self.Tx_distance
        if (n > 255):
            n = 0
        if (d > 65535):
            d = 0

        if serial_init.ser.writable():

            start_rc = bytearray()
            start_rc.append(0x52)
            start_rc.append(0x49)
            start_rc.append(0x16)
            start_rc.append(0x10)
            start_rc.append(0x14)
            start_rc.append(n)
            start_rc.append(self.Tx_motor_mode)  # Motor Mode

            # start_rc.append(forward_lst[0])
            # start_rc.append(int(self.hex_forward,16))

            # PPS
            start_rc.append(self.Tx_left & 0x00ff)
            start_rc.append((self.Tx_left & 0xff00) >> 8)
            start_rc.append(self.Tx_right & 0x00ff)
            start_rc.append((self.Tx_right & 0xff00) >> 8)

            # Distance

            start_rc.append(self.Tx_distance_id)  # Distance ID
            start_rc.append(self.Tx_distance_range & 0x00ff)  # Distance range2
            start_rc.append((self.Tx_distance_range & 0xff00) >> 8)  # Distance range1
            # print(self.Tx_distance_range)
            # print(self.Tx_distance_id)
            # Sound
            start_rc.append(self.Tx_buzzer_low)
            start_rc.append(self.Tx_buzzer_middle)
            start_rc.append(self.Tx_buzzer_high)
            # start_rc.append(0x00)
            # start_rc.append(0x00)
            # start_rc.append(0x00)
            start_rc.append(self.Tx_musical & 0x00ff)
            start_rc.append(self.Tx_sound_clip & 0x00ff)
            # start_rc.append(self.Tx_sound_clip)

            start_rc.append(self.Tx_servo1_speed & 0x00ff)
            start_rc.append(self.Tx_servo1_angle & 0x00ff)
            start_rc.append(self.Tx_servo2_speed & 0x00ff)
            start_rc.append(self.Tx_servo2_angle & 0x00ff)
            start_rc.append(self.Tx_servo3_speed & 0x00ff)
            start_rc.append(self.Tx_servo3_angle & 0x00ff)

            CK_A = 0
            CK_B = 0
            # print(len(start_rc))
            for i in range(len(start_rc)):
                CK_A = CK_A + start_rc[i]
                CK_B = CK_A + CK_B

            CK_A = CK_A & 0x000000ff
            CK_B = CK_B & 0x000000ff
            start_rc.append(CK_A)
            start_rc.append(CK_B)
            # print(CK_A, CK_B)

            # print(start_rc)
            serial_init.ser.write(start_rc)
            # print("DATA Write")

    ########## Lidar Start function ##########
    def lidar_start_func(self):  ## 시작 protocol 전송
        global wait_flag, start_lidar
        wait_flag = 1
        self.start_btn.setStyleSheet("border-image: url(Start2x.png);")

        if start_lidar == 1:
            global lidar_n
            lidar_n = lidar_n + 1
            if (lidar_n > 255):
                lidar_n = 0

            start_lidar = bytearray()
            start_lidar.append(0x52)
            start_lidar.append(0x49)
            start_lidar.append(6)
            start_lidar.append(0x30)
            start_lidar.append(0x04)
            start_lidar.append(0xfd)
            start_lidar.append(0xe0)
            start_lidar.append(lidar_n)
            start_lidar.append(0x0f)
            # print(lidar_n)
            # print(start_lidar)
            # print(len(start_lidar))

            # start_lidar.append(0xfd)
            # start_lidar.append(0xe0)
            # start_lidar.append(n)
            # start_lidar.append(0x0f)

            CK_A = 0
            CK_B = 0

            for i in range(len(start_lidar)):
                CK_A = CK_A + start_lidar[i]
                CK_B = CK_A + CK_B

            CK_A = CK_A & 0x000000ff
            CK_B = CK_B & 0x000000ff

            # print(CK_A)
            # print(CK_B)
            start_lidar.append(CK_A)
            start_lidar.append(CK_B)

            # print(start_lidar[3])
            # print(start_lidar)
            serial_init.ser.write(start_lidar)
            start_lidar = 0
            print("\nLidar start\n")
        else:
            print("Running")

    ########## Lidar Stop function ##########
    def lidar_stop_func(self):   ## Stop protocol transfer
        global wait_flag,start_lidar
        wait_flag = 0

        self.stop_btn.setStyleSheet("border-image: url(Stop2x.png);")
        global lidar_n
        lidar_n = lidar_n + 1
        if (lidar_n > 255):
            lidar_n = 0

        stop_lidar = bytearray()
        stop_lidar.append(0x52)
        stop_lidar.append(0x49)
        stop_lidar.append(6)
        stop_lidar.append(0x30)
        stop_lidar.append(0x04)
        stop_lidar.append(0xfd)
        stop_lidar.append(0xe0)
        stop_lidar.append(lidar_n)
        stop_lidar.append(0xf0)

        CK_A = 0
        CK_B = 0

        for i in range(len(stop_lidar)):
            CK_A = CK_A + stop_lidar[i]
            CK_B = CK_A + CK_B

        CK_A = CK_A & 0x000000ff
        CK_B = CK_B & 0x000000ff

        stop_lidar.append(CK_A)
        stop_lidar.append(CK_B)
        serial_init.ser.write(stop_lidar)
        # window.run_state_txt.setText("run")
        # self.run_state_txt.setText("wait")

        self.run_state_txt.setStyleSheet("border-image: url(Stop_022x);")

        # self.run_state_txt.setText("STOP")
        # self.run_state_txt.setFont(QtGui.QFont("Yu Gothic UI Semibold", 20))
        # self.run_state_txt.setStyleSheet("Color : red")
        start_lidar = 1
        print("\nLidar stop\n")

    ########## NANO LIDAR scale ##########
    def Slider_Scale(self):
        self.scale = self.scale_slider.value()
        # self.lidar_graph.setXRange(-(60 * self.scale), (60 * self.scale))
        # self.lidar_graph.setYRange(-(60 * self.scale), (60 * self.scale))
        ##############################################
        # slider bar range = 1 ~ 109
        self.min = -(550+50*self.scale)
        self.max = (550+50*self.scale)
        self.lidar_graph.setXRange(self.min, self.max)
        self.lidar_graph.setYRange(self.min, self.max)
        ################################################

        # self.min = -(200 * self.scale)
        # self.max = (200 * self.scale)
        # self.lidar_graph.setXRange(self.min, self.max)
        # self.lidar_graph.setYRange(self.min, self.max)




        # self.lidar_graph.setXRange(-(550+50*self.scale), (550+50*self.scale))
        # self.lidar_graph.setYRange(-(550+50*self.scale), (550+50*self.scale))

        # ##### Scaling Text #####
        # self.range_txt.setText("Current Range (±" + str(self.max) + ")")
        # print(550+50*self.scale)
        # time.sleep(0.001)

    # def Scale(self):
    #     if self.scale_comboBox.currentText() == "Default":
    #         self.lidar_graph.setXRange(-1000, 1000)
    #         self.lidar_graph.setYRange(-1000, 1000)
    #     elif self.scale_comboBox.currentText() == "500":
    #         self.lidar_graph.setXRange(-500, 500)
    #         self.lidar_graph.setYRange(-500, 500)
    #     elif self.scale_comboBox.currentText() == "1000":
    #         self.lidar_graph.setXRange(-1000, 1000)
    #         self.lidar_graph.setYRange(-1000, 1000)
    #     elif self.scale_comboBox.currentText() == "1500":
    #         self.lidar_graph.setXRange(-1500, 1500)
    #         self.lidar_graph.setYRange(-1500, 1500)
    #     elif self.scale_comboBox.currentText() == "2000":
    #         self.lidar_graph.setXRange(-2000, 2000)
    #         self.lidar_graph.setYRange(-2000, 2000)
    #     elif self.scale_comboBox.currentText() == "3000":
    #         self.lidar_graph.setXRange(-3000, 3000)
    #         self.lidar_graph.setYRange(-3000, 3000)
    #     elif self.scale_comboBox.currentText() == "4000":
    #         self.lidar_graph.setXRange(-4000, 4000)
    #         self.lidar_graph.setYRange(-4000, 4000)
    #     elif self.scale_comboBox.currentText() == "5000":
    #         self.lidar_graph.setXRange(-5000, 5000)
    #         self.lidar_graph.setYRange(-5000, 5000)
    ########## Exit ############
    def closeEvent(self, QCloseEvent):
        global thread
        re = QMessageBox.question(self, "Exit Confirmation", "Are you sure you want to quit?",
                                  QMessageBox.Yes | QMessageBox.No)

        if re == QMessageBox.Yes:
            print("END")
            thread = 0
            time.sleep(1)
            QCloseEvent.accept()
        else:
            QCloseEvent.ignore()

########## Serial ##########
class Serial():
    global COM
    def __init__(self):
        # if COM == "COM4"
        print("Serial funciton")
        print(COM)
        self.ser = serial.Serial(port=COM,
                                 # baudrate=115200,
                                 baudrate=460800,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE)

########## Receive data from serial communication ##########
class Get_data(Serial):
    def __init__(self):
        global thread
        # parsing hex
        while True:
            if thread == 1:
        # if serial_init.ser.readable():
            # if Serial().ser.readable():
                if serial_init.ser.readable():
                # while True:
                    rxData = serial_init.ser.read(1)  # 1byte씩 값을 받아옴
                    # print(rxData)
                    Data = binascii.b2a_hex(rxData).decode('ascii')
                    # print(Data)
                    hexStr = ' 0x'.join(re.findall('(.{2})', Data))
                    # print(hexStr)
                    hexStr = '0x' + hexStr
                    # print(hexStr)

                    self.data = hexStr.split(" ")

                    q_box.put(self.data)
                else:
                    print("disreadable")
            else:
                print("Break Get Data")
                break

                # time.sleep(0.2)

########## Parsing data from Get data class ##########
class Parsing_data():
    def __init__(self):
        global data_length, robo_index_lst_val, imu_index_lst_val
        global Data_length, com_start, thread
        self.wait_data = []
        self.distance_data = []
        while True:
            if thread == 1:
                self.header1 = q_box.get()

                # time.sleep(0.9)
                # if self.header == '0x52':
                data_length = 0
                if (int(self.header1[0], 16) == 0x52):
                    self.header2 = q_box.get()
                    if int(self.header2[0], 16) == 0x4f:

                        # print(self.header2[0], True)
                        self.data_list = []
                        data_length = q_box.get()[0]
                        # print(data_length)

                        self.data_list.append(self.header1[0])
                        self.data_list.append(self.header2[0])
                        self.data_list.append(data_length)
                        data_length = int(data_length, 16)

                        for i in range(data_length + 2):
                            self.data_list.append(q_box.get()[0])

                        # print(self.data_list)
                        try:
                            if int(self.data_list[3], 16) == 0x10:
                                # print("RoboticsCar Data")
                                self.robo_message_ID = self.data_list[3]
                                self.robo_data_length = self.data_list[4]
                                self.robo_index = self.data_list[5]
                                self.robo_time_stamp1 = self.data_list[6][2:]
                                self.robo_time_stamp2 = self.data_list[7]
                                # self.imu_time_stamp_real = (int(self.imu_time_stamp2, 16) * 256 + int(self.imu_time_stamp1, 16))
                                self.robo_time_stamp_real = self.robo_time_stamp2 + self.robo_time_stamp1

                                self.robo_encoder_left1 = self.data_list[8][2:]
                                self.robo_encoder_left2 = self.data_list[9][2:]
                                self.robo_encoder_left3 = self.data_list[10][2:]
                                self.robo_encoder_left4 = self.data_list[11]
                                self.robo_encoder_left_real = self.robo_encoder_left4 + self.robo_encoder_left3 + self.robo_encoder_left2 + self.robo_encoder_left1
                                self.left_encoder = self.robo_encoder_left_real
                                self.left_encoder = Bits(hex=self.left_encoder)
                                self.robo_encoder_left_real = self.left_encoder.int

                                self.robo_encoder_right1 = self.data_list[12][2:]
                                self.robo_encoder_right2 = self.data_list[13][2:]
                                self.robo_encoder_right3 = self.data_list[14][2:]
                                self.robo_encoder_right4 = self.data_list[15]
                                self.robo_encoder_right_real = self.robo_encoder_right4 + self.robo_encoder_right3 + self.robo_encoder_right2 + self.robo_encoder_right1
                                self.right_encoder = self.robo_encoder_right_real
                                self.right_encoder = Bits(hex=self.right_encoder)
                                self.robo_encoder_right_real = self.right_encoder.int

                                self.robo_temperature = self.data_list[16]
                                self.robo_temperature = int(self.robo_temperature, 16)

                                if self.robo_temperature & 0xf0 != 0x00:
                                    self.robo_temperature_var = self.robo_temperature
                                    self.robo_temperature_var = Bits(hex(self.robo_temperature))
                                    self.robo_temperature = self.robo_temperature_var.int

                                self.robo_temperature = 23 + self.robo_temperature / 2

                                self.robo_rssi = Bits(hex=self.data_list[17]).int

                                self.robo_battery_voltage = int(self.data_list[18], 16)
                                self.robo_battery_voltage = 2.0 + self.robo_battery_voltage / 100

                                self.robo_battery_state = self.data_list[19]
                                # print(self.robo_battery_state)
                                self.robo_battery_state = int(self.data_list[19],16)

                                if self.robo_battery_state & 0x01 == 0x01:
                                    self.robo_battery_state = "Charging"
                                elif self.robo_battery_state & 0x06 == 0x000:
                                    self.robo_battery_state = "Normal"
                                elif self.robo_battery_state & 0x06 == 0b010:
                                    self.robo_battery_state = "Middle"
                                elif self.robo_battery_state & 0x06 == 0b100:
                                    self.robo_battery_state = "Low"
                                elif self.robo_battery_state & 0x06 == 0b110:
                                    self.robo_battery_state = "Empty"

                                # print(self.robo_battery_state)
                                self.robo_motor_state = self.data_list[20]
                                self.robo_sound_state = self.data_list[21]
                                self.robo_servo1_speed = int(self.data_list[22], 16)
                                self.robo_servo1_angle = int(self.data_list[23], 16)
                                self.robo_servo2_speed = int(self.data_list[24], 16)
                                self.robo_servo2_angle = int(self.data_list[25], 16)
                                self.robo_servo3_speed = int(self.data_list[26], 16)
                                self.robo_servo3_angle = int(self.data_list[27], 16)

                                robo_message_ID_lst.append(self.data_list[3])
                                robo_data_length_lst.append(self.data_list[4])
                                robo_index_lst.append(self.data_list[5])
                                robo_time_stamp1_lst.append(self.robo_time_stamp_real)
                                robo_encoder_left1_lst.append(self.robo_encoder_left_real)
                                robo_encoder_right1_lst.append(self.robo_encoder_right_real)
                                robo_temperature_lst.append(self.robo_temperature)
                                robo_rssi_lst.append(self.robo_rssi)
                                robo_battery_voltage_lst.append(self.robo_battery_voltage)

                                robo_battery_state_lst.append(self.robo_battery_state)

                                robo_motor_state_lst.append(self.data_list[20])
                                robo_sound_state_lst.append(self.data_list[21])
                                robo_servo1_speed_lst.append(self.robo_servo1_speed)
                                robo_servo1_angle_lst.append(self.robo_servo1_angle)
                                robo_servo2_speed_lst.append(self.robo_servo2_speed)
                                robo_servo2_angle_lst.append(self.robo_servo2_angle)
                                robo_servo3_speed_lst.append(self.robo_servo3_speed)
                                robo_servo3_angle_lst.append(self.robo_servo3_angle)


                                #########################################################


                            elif int(self.data_list[3], 16) == 0x20:

                                # print(self.data_list)
                                # print("RoboticsCar IMU Data")
                                self.imu_message_ID = self.data_list[3]

                                self.imu_data_length = self.data_list[4]
                                self.imu_index = self.data_list[5]
                                self.imu_time_stamp1 = self.data_list[6]
                                self.imu_time_stamp2 = self.data_list[7]
                                self.imu_time_stamp_real = (
                                            int(self.imu_time_stamp2, 16) * 256 + int(self.imu_time_stamp1, 16))
                                self.imu_gyro = self.data_list[8]
                                self.imu_gyro_range = self.data_list[9]
                                self.imu_acc = self.data_list[10]
                                self.imu_acc_range = self.data_list[11]
                                self.imu_mag_odr = self.data_list[12]
                                self.imu_mag_rep_xy = self.data_list[13]
                                self.imu_mag_rep_z = self.data_list[14]
                                # if int(self.data_list[15],16) == 0x10:

                                # print(self.imu_data_length)
                                self.imu_data_length_int = int(self.imu_data_length, 16)
                                imu_message_ID_lst.append(self.imu_message_ID)
                                imu_data_length_lst.append(self.imu_data_length)
                                imu_index_lst.append(self.imu_index)
                                imu_time_stamp1_lst.append(self.imu_time_stamp_real)
                                imu_gyro_lst.append(self.imu_gyro)
                                imu_gyro_range_lst.append(self.imu_gyro_range)
                                imu_acc_lst.append(self.imu_acc)
                                imu_acc_range_lst.append(self.imu_acc_range)
                                imu_mag_odr_lst.append(self.imu_mag_odr)
                                imu_mag_rep_xy_lst.append(self.imu_mag_rep_xy)
                                imu_mag_rep_z_lst.append(self.imu_mag_rep_z)






                                ##### Sensor Data #1 #####
                                # print("Frist Sensor data",int(self.data_list[15],16) & 0xf0,int(self.data_list[15],16) & 0x0f)
                                if int(self.data_list[15], 16) & 0xf0 == 0x10:

                                    # self.imu_data_id1 = int(self.data_list[15],16) &0x0f
                                    self.imu_data_id1 = int(self.data_list[15], 16) & 0x0f
                                    gyro_ind_lst.append(self.imu_data_id1)
                                    self.imu_gyrox_n1 = self.data_list[16]
                                    self.imu_gyrox_n2 = self.data_list[17]
                                    # self.imu_gyrox_real = (int(self.imu_gyrox_n2,16)<<8 | int(self.imu_gyrox_n1,16))
                                    self.imu_gyrox_real = (int(self.imu_gyrox_n2, 16) * 256 + int(self.imu_gyrox_n1, 16))

                                    self.gyrox = self.imu_gyrox_real
                                    self.gyrox = Bits(hex(self.gyrox))
                                    self.imu_gyrox_real = self.gyrox.int

                                    self.imu_gyroy_n1 = self.data_list[18]
                                    self.imu_gyroy_n2 = self.data_list[19]
                                    self.imu_gyroy_real = int(self.imu_gyroy_n2, 16) * 256 + int(self.imu_gyroy_n1, 16)
                                    self.gyroy = self.imu_gyroy_real
                                    self.gyroy = Bits(hex(self.gyroy))
                                    self.imu_gyroy_real = self.gyroy.int

                                    self.imu_gyroz_n1 = self.data_list[20]
                                    self.imu_gyroz_n2 = self.data_list[21]
                                    self.imu_gyroz_real = int(self.imu_gyroz_n2, 16) * 256 + int(self.imu_gyroz_n1, 16)
                                    self.gyroz = self.imu_gyroz_real
                                    self.gyroz = Bits(hex(self.gyroz))
                                    self.imu_gyroz_real = self.gyroz.int

                                    self.id_num = 22

                                    imu_gyrox_len.append(len(imu_gyrox_n1_lst))
                                    imu_data_id1_lst.append(self.imu_data_id1)
                                    imu_gyrox_n1_lst.append(self.imu_gyrox_real)
                                    imu_gyroy_n1_lst.append(self.imu_gyroy_real)
                                    imu_gyroz_n1_lst.append(self.imu_gyroz_real)


                                elif int(self.data_list[15], 16) & 0xf0 == 0x20:

                                    self.imu_data_id2 = int(self.data_list[15], 16) & 0x0f
                                    self.imu_accx_n1 = self.data_list[16]
                                    self.imu_accx_n2 = self.data_list[17]
                                    self.imu_accx_real = int(self.imu_accx_n2, 16) * 256 + int(self.imu_accx_n1, 16)
                                    self.imu_accx_real = self.imu_accx_real & 0x0fff
                                    self.accx = self.imu_accx_real
                                    self.accx = Bits(hex(self.accx))
                                    self.imu_accx_real = self.accx.int

                                    self.imu_accy_n1 = self.data_list[18]
                                    self.imu_accy_n2 = self.data_list[19]
                                    self.imu_accy_real = int(self.imu_accy_n2, 16) * 256 + int(self.imu_accy_n1, 16)
                                    self.imu_accy_real = self.imu_accy_real & 0x0fff
                                    self.accy = self.imu_accy_real
                                    self.accy = Bits(hex(self.imu_accy_real))
                                    self.imu_accy_real = self.accy.int

                                    self.imu_accz_n1 = self.data_list[20]
                                    self.imu_accz_n2 = self.data_list[21]
                                    self.imu_accz_real = int(self.imu_accz_n2, 16) * 256 + int(self.imu_accz_n1, 16)
                                    self.imu_accz_real = self.imu_accz_real & 0x0fff
                                    self.accz = self.imu_accz_real
                                    self.accz = Bits(hex(self.imu_accz_real))
                                    self.imu_accz_real = self.accz.int

                                    self.imu_acc_temp_n = self.data_list[22]
                                    self.imu_acc_temp_n = int(self.imu_acc_temp_n, 16)
                                    if self.imu_acc_temp_n & 0xf0 != 0x00:
                                        self.temperature = self.imu_acc_temp_n
                                        self.temperature = Bits(hex(self.imu_acc_temp_n))
                                        self.imu_acc_temp_n = self.temperature.int

                                    # print(self.imu_acc_temp_n)
                                    self.imu_acc_temp_n = 23 + self.imu_acc_temp_n / 2

                                    self.id_num = 23
                                    imu_data_id2_lst.append(self.imu_data_id2)
                                    imu_accx_n1_lst.append(self.imu_accx_real)
                                    imu_accy_n1_lst.append(self.imu_accy_real)
                                    imu_accz_n1_lst.append(self.imu_accz_real)
                                    imu_acc_temp_n_lst.append(self.imu_acc_temp_n)



                                elif int(self.data_list[15], 16) & 0xf0 == 0x30:

                                    # print("imu_data_id3__1 == 0x30")
                                    self.imu_data_id3 = int(self.data_list[15], 16) & 0x0f
                                    self.imu_compassx_n1 = self.data_list[16][2:]
                                    self.imu_compassx_n2 = self.data_list[17][2:]
                                    self.imu_compassx_n3 = self.data_list[18][2:]
                                    self.imu_compassx_n4 = self.data_list[19]
                                    self.imu_compassx_real = self.imu_compassx_n4 + self.imu_compassx_n3 + self.imu_compassx_n2 + self.imu_compassx_n1
                                    self.imu_compassx_real_int = int(self.imu_compassx_real, 16)
                                    self.imu_compassx_real_int = struct.pack("I", self.imu_compassx_real_int)
                                    self.imu_compassx_real = round(struct.unpack('f', self.imu_compassx_real_int)[0], 3)

                                    self.imu_compassy_n1 = self.data_list[20][2:]
                                    self.imu_compassy_n2 = self.data_list[21][2:]
                                    self.imu_compassy_n3 = self.data_list[22][2:]
                                    self.imu_compassy_n4 = self.data_list[23]
                                    self.imu_compassy_real = self.imu_compassy_n4 + self.imu_compassy_n3 + self.imu_compassy_n2 + self.imu_compassy_n1
                                    self.imu_compassy_real_int = int(self.imu_compassy_real, 16)
                                    self.imu_compassy_real_int = struct.pack("I", self.imu_compassy_real_int)
                                    self.imu_compassy_real = round(struct.unpack('f', self.imu_compassy_real_int)[0], 3)

                                    self.imu_compassz_n1 = self.data_list[24][2:]
                                    self.imu_compassz_n2 = self.data_list[25][2:]
                                    self.imu_compassz_n3 = self.data_list[26][2:]
                                    self.imu_compassz_n4 = self.data_list[27]
                                    self.imu_compassz_real = self.imu_compassz_n4 + self.imu_compassz_n3 + self.imu_compassz_n2 + self.imu_compassz_n1
                                    self.imu_compassz_real_int = int(self.imu_compassz_real, 16)
                                    self.imu_compassz_real_int = struct.pack("I", self.imu_compassz_real_int)
                                    self.imu_compassz_real = round(struct.unpack('f', self.imu_compassz_real_int)[0], 3)
                                    self.id_num = 28

                                    imu_data_id3_lst.append(self.imu_data_id3)
                                    imu_compassx_n1_lst.append(self.imu_compassx_real)
                                    imu_compassy_n1_lst.append(self.imu_compassy_real)
                                    imu_compassz_n1_lst.append(self.imu_compassz_real)

                                else:
                                    pass

                                ################## 2 ################
                                # print("Data2", int(self.data_list[self.id_num], 16) & 0xf0, int(self.data_list[self.id_num], 16) & 0x0f)
                                if int(self.data_list[self.id_num], 16) & 0xf0 == 0x10:

                                    self.imu_data_id1 = int(self.data_list[self.id_num], 16) & 0x0f
                                    gyro_ind_lst.append(self.imu_data_id1)
                                    self.imu_gyrox_n1 = self.data_list[self.id_num + 1]
                                    self.imu_gyrox_n2 = self.data_list[self.id_num + 2]
                                    # self.imu_gyrox_real = (int(self.imu_gyrox_n2,16)<<8 | int(self.imu_gyrox_n1,16))
                                    self.imu_gyrox_real = (int(self.imu_gyrox_n2, 16) * 256 + int(self.imu_gyrox_n1, 16))
                                    self.gyrox = self.imu_gyrox_real
                                    self.gyrox = Bits(hex(self.gyrox))
                                    self.imu_gyrox_real = self.gyrox.int

                                    self.imu_gyroy_n1 = self.data_list[self.id_num + 3]
                                    self.imu_gyroy_n2 = self.data_list[self.id_num + 4]
                                    self.imu_gyroy_real = int(self.imu_gyroy_n2, 16) * 256 + int(self.imu_gyroy_n1, 16)
                                    self.gyroy = self.imu_gyroy_real
                                    self.gyroy = Bits(hex(self.gyroy))
                                    self.imu_gyroy_real = self.gyroy.int

                                    self.imu_gyroz_n1 = self.data_list[self.id_num + 5]
                                    self.imu_gyroz_n2 = self.data_list[self.id_num + 6]
                                    self.imu_gyroz_real = int(self.imu_gyroz_n2, 16) * 256 + int(self.imu_gyroz_n1, 16)
                                    self.gyroz = self.imu_gyroz_real
                                    self.gyroz = Bits(hex(self.gyroz))
                                    self.imu_gyroz_real = self.gyroz.int

                                    self.id_num = self.id_num + 7
                                    imu_gyrox_len.append(len(imu_gyrox_n1_lst))
                                    imu_data_id1_lst.append(self.imu_data_id1)
                                    imu_gyrox_n1_lst.append(self.imu_gyrox_real)
                                    imu_gyroy_n1_lst.append(self.imu_gyroy_real)
                                    imu_gyroz_n1_lst.append(self.imu_gyroz_real)

                                elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x20:

                                    self.imu_data_id2 = int(self.data_list[self.id_num], 16) & 0x0f
                                    self.imu_accx_n1 = self.data_list[self.id_num + 1]
                                    self.imu_accx_n2 = self.data_list[self.id_num + 2]
                                    self.imu_accx_real = int(self.imu_accx_n2, 16) * 256 + int(self.imu_accx_n1, 16)
                                    self.imu_accx_real = self.imu_accx_real & 0x0fff
                                    self.accx = self.imu_accx_real
                                    self.accx = Bits(hex(self.accx))
                                    self.imu_accx_real = self.accx.int

                                    self.imu_accy_n1 = self.data_list[self.id_num + 3]
                                    self.imu_accy_n2 = self.data_list[self.id_num + 4]
                                    self.imu_accy_real = int(self.imu_accy_n2, 16) * 256 + int(self.imu_accy_n1, 16)
                                    self.imu_accy_real = self.imu_accy_real & 0x0fff
                                    self.accy = self.imu_accy_real
                                    self.accy = Bits(hex(self.imu_accy_real))
                                    self.imu_accy_real = self.accy.int

                                    self.imu_accz_n1 = self.data_list[self.id_num + 5]
                                    self.imu_accz_n2 = self.data_list[self.id_num + 6]
                                    self.imu_accz_real = int(self.imu_accz_n2, 16) * 256 + int(self.imu_accz_n1, 16)
                                    self.imu_accz_real = self.imu_accz_real & 0x0fff
                                    self.accz = self.imu_accz_real
                                    self.accz = Bits(hex(self.imu_accz_real))
                                    self.imu_accz_real = self.accz.int

                                    self.imu_acc_temp_n = self.data_list[self.id_num + 7]

                                    self.imu_acc_temp_n = int(self.imu_acc_temp_n, 16)
                                    if self.imu_acc_temp_n & 0xf0 != 0x00:
                                        self.temperature = self.imu_acc_temp_n
                                        self.temperature = Bits(hex(self.imu_acc_temp_n))
                                        self.imu_acc_temp_n = self.temperature.int

                                    self.imu_acc_temp_n = 23 + self.imu_acc_temp_n / 2

                                    self.id_num = self.id_num + 8

                                    imu_data_id2_lst.append(self.imu_data_id2)
                                    imu_accx_n1_lst.append(self.imu_accx_real)
                                    imu_accy_n1_lst.append(self.imu_accy_real)
                                    imu_accz_n1_lst.append(self.imu_accz_real)
                                    imu_acc_temp_n_lst.append(self.imu_acc_temp_n)


                                elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x30:

                                    # print("imu_data_id3__2 == 0x30")
                                    self.imu_data_id3 = int(self.data_list[self.id_num], 16) & 0x0f
                                    self.imu_compassx_n1 = self.data_list[self.id_num + 1][2:]
                                    self.imu_compassx_n2 = self.data_list[self.id_num + 2][2:]
                                    self.imu_compassx_n3 = self.data_list[self.id_num + 3][2:]
                                    self.imu_compassx_n4 = self.data_list[self.id_num + 4]
                                    self.imu_compassx_real = self.imu_compassx_n4 + self.imu_compassx_n3 + self.imu_compassx_n2 + self.imu_compassx_n1
                                    self.imu_compassx_real_int = int(self.imu_compassx_real, 16)
                                    self.imu_compassx_real_int = struct.pack("I", self.imu_compassx_real_int)
                                    self.imu_compassx_real = round(struct.unpack('f', self.imu_compassx_real_int)[0], 3)

                                    self.imu_compassy_n1 = self.data_list[self.id_num + 5][2:]
                                    self.imu_compassy_n2 = self.data_list[self.id_num + 6][2:]
                                    self.imu_compassy_n3 = self.data_list[self.id_num + 7][2:]
                                    self.imu_compassy_n4 = self.data_list[self.id_num + 8]
                                    self.imu_compassy_real = self.imu_compassy_n4 + self.imu_compassy_n3 + self.imu_compassy_n2 + self.imu_compassy_n1
                                    self.imu_compassy_real_int = int(self.imu_compassy_real, 16)
                                    self.imu_compassy_real_int = struct.pack("I", self.imu_compassy_real_int)
                                    self.imu_compassy_real = round(struct.unpack('f', self.imu_compassy_real_int)[0], 3)

                                    self.imu_compassz_n1 = self.data_list[self.id_num + 9][2:]
                                    self.imu_compassz_n2 = self.data_list[self.id_num + 10][2:]
                                    self.imu_compassz_n3 = self.data_list[self.id_num + 11][2:]
                                    self.imu_compassz_n4 = self.data_list[self.id_num + 12]
                                    self.imu_compassz_real = self.imu_compassz_n4 + self.imu_compassz_n3 + self.imu_compassz_n2 + self.imu_compassz_n1
                                    self.imu_compassz_real_int = int(self.imu_compassz_real, 16)
                                    self.imu_compassz_real_int = struct.pack("I", self.imu_compassz_real_int)
                                    self.imu_compassz_real = round(struct.unpack('f', self.imu_compassz_real_int)[0], 3)

                                    self.id_num = self.id_num + 13

                                    imu_data_id3_lst.append(self.imu_data_id3)
                                    imu_compassx_n1_lst.append(self.imu_compassx_real)
                                    imu_compassy_n1_lst.append(self.imu_compassy_real)
                                    imu_compassz_n1_lst.append(self.imu_compassz_real)
                                else:
                                    pass

                                ###################### 3 #####################
                                # print("Data3", int(self.data_list[self.id_num], 16) & 0xf0,int(self.data_list[self.id_num], 16) & 0x0f)
                                if int(self.data_list[self.id_num], 16) & 0xf0 == 0x10:

                                    self.imu_data_id1 = int(self.data_list[self.id_num], 16) & 0x0f
                                    gyro_ind_lst.append(self.imu_data_id1)
                                    self.imu_gyrox_n1 = self.data_list[self.id_num + 1]
                                    self.imu_gyrox_n2 = self.data_list[self.id_num + 2]
                                    # self.imu_gyrox_real = (int(self.imu_gyrox_n2,16)<<8 | int(self.imu_gyrox_n1,16))
                                    self.imu_gyrox_real = (int(self.imu_gyrox_n2, 16) * 256 + int(self.imu_gyrox_n1, 16))
                                    self.gyrox = self.imu_gyrox_real
                                    self.gyrox = Bits(hex(self.gyrox))
                                    self.imu_gyrox_real = self.gyrox.int

                                    self.imu_gyroy_n1 = self.data_list[self.id_num + 3]
                                    self.imu_gyroy_n2 = self.data_list[self.id_num + 4]
                                    self.imu_gyroy_real = int(self.imu_gyroy_n2, 16) * 256 + int(self.imu_gyroy_n1, 16)
                                    self.gyroy = self.imu_gyroy_real
                                    self.gyroy = Bits(hex(self.gyroy))
                                    self.imu_gyroy_real = self.gyroy.int

                                    self.imu_gyroz_n1 = self.data_list[self.id_num + 5]
                                    self.imu_gyroz_n2 = self.data_list[self.id_num + 6]
                                    self.imu_gyroz_real = int(self.imu_gyroz_n2, 16) * 256 + int(self.imu_gyroz_n1, 16)
                                    self.gyroz = self.imu_gyroz_real
                                    self.gyroz = Bits(hex(self.gyroz))
                                    self.imu_gyroz_real = self.gyroz.int

                                    self.id_num = self.id_num + 7

                                    imu_data_id1_lst.append(self.imu_data_id1)
                                    imu_gyrox_n1_lst.append(self.imu_gyrox_real)
                                    imu_gyroy_n1_lst.append(self.imu_gyroy_real)
                                    imu_gyroz_n1_lst.append(self.imu_gyroz_real)


                                elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x20:

                                    self.imu_data_id2 = int(self.data_list[self.id_num], 16) & 0x0f
                                    self.imu_accx_n1 = self.data_list[self.id_num + 1]
                                    self.imu_accx_n2 = self.data_list[self.id_num + 2]
                                    self.imu_accx_real = int(self.imu_accx_n2, 16) * 256 + int(self.imu_accx_n1, 16)
                                    self.imu_accx_real = self.imu_accx_real & 0x0fff
                                    self.accx = self.imu_accx_real
                                    self.accx = Bits(hex(self.accx))
                                    self.imu_accx_real = self.accx.int

                                    self.imu_accy_n1 = self.data_list[self.id_num + 3]
                                    self.imu_accy_n2 = self.data_list[self.id_num + 4]
                                    self.imu_accy_real = int(self.imu_accy_n2, 16) * 256 + int(self.imu_accy_n1, 16)
                                    self.imu_accy_real = self.imu_accy_real & 0x0fff
                                    self.accy = self.imu_accy_real
                                    self.accy = Bits(hex(self.imu_accy_real))
                                    self.imu_accy_real = self.accy.int

                                    self.imu_accz_n1 = self.data_list[self.id_num + 5]
                                    self.imu_accz_n2 = self.data_list[self.id_num + 6]
                                    self.imu_accz_real = int(self.imu_accz_n2, 16) * 256 + int(self.imu_accz_n1, 16)
                                    self.imu_accz_real = self.imu_accz_real & 0x0fff
                                    self.accz = self.imu_accz_real
                                    self.accz = Bits(hex(self.imu_accz_real))
                                    self.imu_accz_real = self.accz.int

                                    self.imu_acc_temp_n = self.data_list[self.id_num + 7]
                                    # print("")
                                    self.imu_acc_temp_n = int(self.imu_acc_temp_n, 16)
                                    if self.imu_acc_temp_n & 0xf0 != 0x00:
                                        self.temperature = self.imu_acc_temp_n
                                        self.temperature = Bits(hex(self.imu_acc_temp_n))
                                        self.imu_acc_temp_n = self.temperature.int

                                    self.imu_acc_temp_n = 23 + self.imu_acc_temp_n / 2
                                    self.id_num = self.id_num + 8

                                    imu_data_id2_lst.append(self.imu_data_id2)
                                    imu_accx_n1_lst.append(self.imu_accx_real)
                                    imu_accy_n1_lst.append(self.imu_accy_real)
                                    imu_accz_n1_lst.append(self.imu_accz_real)
                                    imu_acc_temp_n_lst.append(self.imu_acc_temp_n)


                                elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x30:

                                    # print("imu_data_id3__3 == 0x30")
                                    self.imu_data_id3 = int(self.data_list[self.id_num], 16) & 0x0f
                                    self.imu_compassx_n1 = self.data_list[self.id_num + 1][2:]
                                    self.imu_compassx_n2 = self.data_list[self.id_num + 2][2:]
                                    self.imu_compassx_n3 = self.data_list[self.id_num + 3][2:]
                                    self.imu_compassx_n4 = self.data_list[self.id_num + 4]
                                    self.imu_compassx_real = self.imu_compassx_n4 + self.imu_compassx_n3 + self.imu_compassx_n2 + self.imu_compassx_n1
                                    self.imu_compassx_real_int = int(self.imu_compassx_real, 16)
                                    self.imu_compassx_real_int = struct.pack("I", self.imu_compassx_real_int)
                                    self.imu_compassx_real = round(struct.unpack('f', self.imu_compassx_real_int)[0], 3)

                                    self.imu_compassy_n1 = self.data_list[self.id_num + 5][2:]
                                    self.imu_compassy_n2 = self.data_list[self.id_num + 6][2:]
                                    self.imu_compassy_n3 = self.data_list[self.id_num + 7][2:]
                                    self.imu_compassy_n4 = self.data_list[self.id_num + 8]
                                    self.imu_compassy_real = self.imu_compassy_n4 + self.imu_compassy_n3 + self.imu_compassy_n2 + self.imu_compassy_n1
                                    self.imu_compassy_real_int = int(self.imu_compassy_real, 16)
                                    self.imu_compassy_real_int = struct.pack("I", self.imu_compassy_real_int)
                                    self.imu_compassy_real = round(struct.unpack('f', self.imu_compassy_real_int)[0], 3)

                                    self.imu_compassz_n1 = self.data_list[self.id_num + 9][2:]
                                    self.imu_compassz_n2 = self.data_list[self.id_num + 10][2:]
                                    self.imu_compassz_n3 = self.data_list[self.id_num + 11][2:]
                                    self.imu_compassz_n4 = self.data_list[self.id_num + 12]
                                    self.imu_compassz_real = self.imu_compassz_n4 + self.imu_compassz_n3 + self.imu_compassz_n2 + self.imu_compassz_n1
                                    self.imu_compassz_real_int = int(self.imu_compassz_real, 16)
                                    self.imu_compassz_real_int = struct.pack("I", self.imu_compassz_real_int)
                                    self.imu_compassz_real = round(struct.unpack('f', self.imu_compassz_real_int)[0], 3)

                                    self.id_num = self.id_num + 13

                                    imu_data_id3_lst.append(self.imu_data_id3)
                                    imu_compassx_n1_lst.append(self.imu_compassx_real)
                                    imu_compassy_n1_lst.append(self.imu_compassy_real)
                                    imu_compassz_n1_lst.append(self.imu_compassz_real)

                                ###############From 4~ #####
                                # print(self.id_num)
                                self.imu_data_length_int = int(self.imu_data_length, 16)
                                # print("data_length",self.imu_data_length_int)

                                # print('')
                                # print("start",self.id_num)
                                # print("data length",self.imu_data_length_int)

                                for i in range(self.imu_data_length_int - self.id_num):
                                    # if self.id_num+2 << self.imu_data_length_int:
                                    #     print("id_num",self.id_num)

                                    if self.imu_data_length_int <= self.id_num + 2:
                                        break

                                    # print("data_list",int(self.data_list[self.id_num] ,16) & 0xf0)
                                    # time.sleep(0.01)
                                    if int(self.data_list[self.id_num], 16) & 0xf0 == 0x10:

                                        self.imu_data_id1 = int(self.data_list[self.id_num], 16) & 0x0f
                                        gyro_ind_lst.append(self.imu_data_id1)
                                        self.imu_gyrox_n1 = self.data_list[self.id_num + 1]
                                        self.imu_gyrox_n2 = self.data_list[self.id_num + 2]
                                        # self.imu_gyrox_real = (int(self.imu_gyrox_n2,16)<<8 | int(self.imu_gyrox_n1,16))
                                        self.imu_gyrox_real = (
                                                    int(self.imu_gyrox_n2, 16) * 256 + int(self.imu_gyrox_n1, 16))
                                        self.gyrox = self.imu_gyrox_real
                                        self.gyrox = Bits(hex(self.gyrox))
                                        self.imu_gyrox_real = self.gyrox.int

                                        self.imu_gyroy_n1 = self.data_list[self.id_num + 3]
                                        self.imu_gyroy_n2 = self.data_list[self.id_num + 4]
                                        self.imu_gyroy_real = int(self.imu_gyroy_n2, 16) * 256 + int(self.imu_gyroy_n1, 16)
                                        self.gyroy = self.imu_gyroy_real
                                        self.gyroy = Bits(hex(self.gyroy))
                                        self.imu_gyroy_real = self.gyroy.int

                                        self.imu_gyroz_n1 = self.data_list[self.id_num + 5]
                                        self.imu_gyroz_n2 = self.data_list[self.id_num + 6]
                                        self.imu_gyroz_real = int(self.imu_gyroz_n2, 16) * 256 + int(self.imu_gyroz_n1, 16)
                                        self.gyroz = self.imu_gyroz_real
                                        self.gyroz = Bits(hex(self.gyroz))
                                        self.imu_gyroz_real = self.gyroz.int

                                        self.id_num = self.id_num + 7

                                        imu_data_id1_lst.append(self.imu_data_id1)
                                        imu_gyrox_n1_lst.append(self.imu_gyrox_real)
                                        imu_gyroy_n1_lst.append(self.imu_gyroy_real)
                                        imu_gyroz_n1_lst.append(self.imu_gyroz_real)



                                    elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x20:

                                        self.imu_data_id2 = int(self.data_list[self.id_num], 16) & 0x0f
                                        self.imu_accx_n1 = self.data_list[self.id_num + 1]
                                        self.imu_accx_n2 = self.data_list[self.id_num + 2]
                                        self.imu_accx_real = int(self.imu_accx_n2, 16) * 256 + int(self.imu_accx_n1, 16)
                                        self.imu_accx_real = self.imu_accx_real & 0x0fff
                                        self.accx = self.imu_accx_real
                                        self.accx = Bits(hex(self.accx))
                                        self.imu_accx_real = self.accx.int

                                        self.imu_accy_n1 = self.data_list[self.id_num + 3]
                                        self.imu_accy_n2 = self.data_list[self.id_num + 4]
                                        self.imu_accy_real = int(self.imu_accy_n2, 16) * 256 + int(self.imu_accy_n1, 16)
                                        self.imu_accy_real = self.imu_accy_real & 0x0fff
                                        self.accy = self.imu_accy_real
                                        self.accy = Bits(hex(self.imu_accy_real))
                                        self.imu_accy_real = self.accy.int

                                        self.imu_accz_n1 = self.data_list[self.id_num + 5]
                                        self.imu_accz_n2 = self.data_list[self.id_num + 6]
                                        self.imu_accz_real = int(self.imu_accz_n2, 16) * 256 + int(self.imu_accz_n1, 16)
                                        self.imu_accz_real = self.imu_accz_real & 0x0fff
                                        self.accz = self.imu_accz_real
                                        self.accz = Bits(hex(self.imu_accz_real))
                                        self.imu_accz_real = self.accz.int

                                        self.imu_acc_temp_n = self.data_list[self.id_num + 7]
                                        # print("")
                                        self.imu_acc_temp_n = int(self.imu_acc_temp_n, 16)
                                        if self.imu_acc_temp_n & 0xf0 != 0x00:
                                            self.temperature = self.imu_acc_temp_n
                                            self.temperature = Bits(hex(self.imu_acc_temp_n))
                                            self.imu_acc_temp_n = self.temperature.int

                                        self.imu_acc_temp_n = 23 + self.imu_acc_temp_n / 2
                                        self.id_num = self.id_num + 8

                                        imu_data_id2_lst.append(self.imu_data_id2)
                                        imu_accx_n1_lst.append(self.imu_accx_real)
                                        imu_accy_n1_lst.append(self.imu_accy_real)
                                        imu_accz_n1_lst.append(self.imu_accz_real)
                                        imu_acc_temp_n_lst.append(self.imu_acc_temp_n)



                                    elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x30:

                                        # print("imu_data_id3__3 == 0x30")
                                        self.imu_data_id3 = int(self.data_list[self.id_num], 16) & 0x0f
                                        self.imu_compassx_n1 = self.data_list[self.id_num + 1][2:]
                                        self.imu_compassx_n2 = self.data_list[self.id_num + 2][2:]
                                        self.imu_compassx_n3 = self.data_list[self.id_num + 3][2:]
                                        self.imu_compassx_n4 = self.data_list[self.id_num + 4]
                                        self.imu_compassx_real = self.imu_compassx_n4 + self.imu_compassx_n3 + self.imu_compassx_n2 + self.imu_compassx_n1
                                        self.imu_compassx_real_int = int(self.imu_compassx_real, 16)
                                        self.imu_compassx_real_int = struct.pack("I", self.imu_compassx_real_int)
                                        self.imu_compassx_real = round(struct.unpack('f', self.imu_compassx_real_int)[0], 3)

                                        self.imu_compassy_n1 = self.data_list[self.id_num + 5][2:]
                                        self.imu_compassy_n2 = self.data_list[self.id_num + 6][2:]
                                        self.imu_compassy_n3 = self.data_list[self.id_num + 7][2:]
                                        self.imu_compassy_n4 = self.data_list[self.id_num + 8]
                                        self.imu_compassy_real = self.imu_compassy_n4 + self.imu_compassy_n3 + self.imu_compassy_n2 + self.imu_compassy_n1
                                        self.imu_compassy_real_int = int(self.imu_compassy_real, 16)
                                        self.imu_compassy_real_int = struct.pack("I", self.imu_compassy_real_int)
                                        self.imu_compassy_real = round(struct.unpack('f', self.imu_compassy_real_int)[0], 3)

                                        self.imu_compassz_n1 = self.data_list[self.id_num + 9][2:]
                                        self.imu_compassz_n2 = self.data_list[self.id_num + 10][2:]
                                        self.imu_compassz_n3 = self.data_list[self.id_num + 11][2:]
                                        self.imu_compassz_n4 = self.data_list[self.id_num + 12]
                                        self.imu_compassz_real = self.imu_compassz_n4 + self.imu_compassz_n3 + self.imu_compassz_n2 + self.imu_compassz_n1
                                        self.imu_compassz_real_int = int(self.imu_compassz_real, 16)
                                        self.imu_compassz_real_int = struct.pack("I", self.imu_compassz_real_int)
                                        self.imu_compassz_real = round(struct.unpack('f', self.imu_compassz_real_int)[0], 3)

                                        self.id_num = self.id_num + 13

                                        imu_data_id3_lst.append(self.imu_data_id3)
                                        imu_compassx_n1_lst.append(self.imu_compassx_real)
                                        imu_compassy_n1_lst.append(self.imu_compassy_real)
                                        imu_compassz_n1_lst.append(self.imu_compassz_real)



                                    elif int(self.data_list[self.id_num], 16) & 0xf0 == 0x00:
                                        pass

                                    else:
                                        print("Sensor data Error")
                                        print(int(self.data_list[self.id_num], 16) & 0xf0)
                                        break

                                    time.sleep(0.00001)
                                time.sleep(0.00001)


                                #### losted imu data
                                # if int(imu_index_lst[-1], 16) - int(imu_index_lst[-2], 16) != 1:
                                #     if int(imu_index_lst[-1], 16) - int(imu_index_lst[-2], 16) != -255:
                                #         # print("Lost imu_index")
                                #         imu_index_lst_val = imu_index_lst_val + 1
                                #         print("Losted_imu_Data", imu_index_lst_val)
                                #         print(int(imu_index_lst[-1], 16) - int(imu_index_lst[-2], 16))
                                #         print(int(imu_index_lst[-2], 16))
                                #         print(int(imu_index_lst[-1], 16))
                                #         print("")

                                # print(gyro_ind_lst)
                                self.data_id_and_data = self.data_list[self.id_num:]
                                data_id_and_data_lst.append(self.data_id_and_data)

                            elif int(self.data_list[3], 16) == 0x30: # Message ID
                                # print("\nLidar data")
                                # print("data_list1", self.data_list)
                                if (wait_flag == 1):
                                    # print("wait_flag == 1")
                                    # print(self.data_list)
                                    myWindow.run_state_txt.setStyleSheet("border-image: url(Running_022x.png);")
                                    # myWindow.run_state_txt.setText("RUNNING")
                                    # myWindow.run_state_txt.setStyleSheet("Color : green")
                                    if (int(self.data_list[5], 16) == 0xfe): # Header
                                        if (int(self.data_list[6],16) == 0x10): # Message ID
                                        # self.id = q_box.get()
                                        # print("lidar")
                                        # if (int(self.id[0], 16) == 0x10):
                                            self.lidar_message = self.data_list[5]
                                            self.lidar_id = self.data_list[6]
                                            self.distance_index = self.data_list[7]
                                            self.system_sate = self.data_list[8]
                                            self.synchronous = self.data_list[9]
                                            self.angle_resolution = self.data_list[10]
                                            self.output_rate = self.data_list[11]
                                            self.length = self.data_list[12]
                                            self.length2 = self.data_list[13]


                                            Data_length = (int(self.length2, 16) << 8 | int(self.length, 16))
                                            Pos_flag = int((self.synchronous), 16)
                                            angle = int(self.angle_resolution, 16) / 10.0

                                            self.distance_data = []
                                            self.distance_data.append(self.lidar_message)
                                            self.distance_data.append(self.lidar_id)
                                            self.distance_data.append(self.distance_index)
                                            self.distance_data.append(self.system_sate)
                                            self.distance_data.append(self.synchronous)
                                            self.distance_data.append(self.angle_resolution)
                                            self.distance_data.append(self.output_rate)
                                            self.distance_data.append(self.length)
                                            self.distance_data.append(self.length2)

                                            for i in range(len(self.data_list[14:])):
                                                self.distance_data.append(self.data_list[i+14])


                                            # show value
                                            myWindow.index_txt.setText(str(int(self.distance_index, 16)))
                                            myWindow.systemstate_txt.setText(str(int(self.system_sate, 16)))
                                            myWindow.synchronous_txt.setText(str(int(self.synchronous, 16)))
                                            myWindow.angle_txt.setText(str(angle) + " °")
                                            myWindow.output_txt.setText(str(int(self.output_rate, 16) / 10) + " Hz")
                                            myWindow.distance_length_txt.setText(str(Data_length))
                                            # print("Distance data :",self.distance_data)
                                            # print("Distance data len :",(len(self.distance_data)-9-2)/2+2)
                                            # print(Data_length)

                                        ## checksum
                                        self.distance_header = self.distance_data[0]
                                        self.distance_message_id = self.distance_data[1]

                                        # self.distance_header = self.data_list[5]
                                        # self.distance_message_id = self.data_list[6]

                                        if (self.distance_header == '0xfe'):
                                            # print("header true")
                                            if (self.Msg_Checksum() == True):
                                                self.distance = []
                                                for i in range(0, Data_length):
                                                    # self.distance.append(int(self.distance_data[10 + (i*2)], 16) << 8 | int(self.distance_data[9 + (i*2)], 16))
                                                    self.distance.append((int(self.distance_data[10 + (i * 2)], 16) << 8 | int(self.distance_data[9 + (i * 2)], 16)))


                                                # lidar 사분면 별로 나눈다.
                                                self.Copy_data(Pos_flag, Data_length)
                                                # print(self.distance)

                                                # Drawing
                                                if (Pos_flag == 4):
                                                    self.draw()


                                            else:
                                                print("error_distance_checksum")

                                        else:
                                            print("Header error", self.data_list[5])
                                            print("header error", self.data_list)




                                    else:
                                        print("header != 0xfe",self.data_list)

                            else:
                                print("Parsing Error", int(self.data_list[3], 16))


                        except:
                            pass


                else:
                    pass
            elif thread == 0:
                print("Break Parsing")
                break

    def Msg_Checksum(self):
        CK_A = 0
        CK_B = 0

        for i in range((Data_length * 2) + 14):
            CK_A = CK_A + int(self.data_list[i], 16)
            CK_B = CK_A + CK_B

        CK_A = CK_A & 0x000000ff
        CK_B = CK_B & 0x000000ff

        if (CK_A == int(self.distance_data[-2], 16) and CK_B == int(self.distance_data[-1], 16)):
            return True
        else:
            return False



    def draw(self):
        global DataPos0
        global DataPos1
        global DataPos2
        global DataPos3

        global ToFPosDataQ0
        global ToFPosDataQ1
        global ToFPosDataQ2
        global ToFPosDataQ3

        print("-------draw-------")
        myWindow.show_data()

        DataPos0 = 0
        DataPos1 = 0
        DataPos2 = 0
        DataPos3 = 0
        ToFPosDataQ0 = []
        ToFPosDataQ1 = []
        ToFPosDataQ2 = []
        ToFPosDataQ3 = []

    def Copy_data(self, Pos_flag, Data_length):
        global DataPos0
        global DataPos1
        global DataPos2
        global DataPos3

        if Pos_flag == 1:
            for i in range(Data_length):
                ToFPosDataQ0.append(self.distance[i])
            DataPos0 = Data_length
            # print(Pos_flag, DataPos0)

        elif Pos_flag == 2:
            for i in range(Data_length):
                ToFPosDataQ1.append(self.distance[i])
            DataPos1 = Data_length
            # print(Pos_flag, DataPos1)

        elif Pos_flag == 3:
            for i in range(Data_length):
                ToFPosDataQ2.append(self.distance[i])
            DataPos2 = Data_length
            # print(Pos_flag, DataPos2)

        elif Pos_flag == 4:
            for i in range(Data_length):
                ToFPosDataQ3.append(self.distance[i])
            DataPos3 = Data_length
            # print(Pos_flag, DataPos3)
        else:
            pass
########## Display data values in GUI ##########
class Show_txt(Serial):
    def __init__(self):
        global thread
        while True:
            if thread == 1:
            # if self.a == 1:
                myWindow.Message_ID_txt.setText(str(robo_message_ID_lst[-1]))

                myWindow.Data_length_txt.setText(str(int(str(robo_data_length_lst[-1]), 16)))
                myWindow.Index_txt.setText(str(int(str(robo_index_lst[-1]), 16)))
                myWindow.Time_stamp_txt.setText(str(int(str(robo_time_stamp1_lst[-1]), 16)))
                myWindow.Encoder_left_txt.setText(str(robo_encoder_left1_lst[-1]))
                myWindow.Encoder_right_txt.setText(str(robo_encoder_right1_lst[-1]))
                myWindow.Temperature_txt.setText(str(robo_temperature_lst[-1]))
                myWindow.Rssi_txt.setText(str(robo_rssi_lst[-1]))
                myWindow.Battery_voltage_txt.setText(str(robo_battery_voltage_lst[-1])[:4])
                myWindow.Battery_state_txt.setText(str(robo_battery_state_lst[-1]))
                myWindow.Motor_state_txt.setText(str(int(str(robo_motor_state_lst[-1]), 16)))
                myWindow.Sound_state_txt.setText(str(int(str(robo_sound_state_lst[-1]), 16)))
                myWindow.Servo1_speed_txt.setText(str(robo_servo1_speed_lst[-1]))
                myWindow.Servo1_angle_txt.setText(str(robo_servo1_angle_lst[-1]))
                myWindow.Servo2_speed_txt.setText(str(robo_servo2_speed_lst[-1]))
                myWindow.Servo2_angle_txt.setText(str(robo_servo2_angle_lst[-1]))
                myWindow.Servo3_speed_txt.setText(str(robo_servo3_speed_lst[-1]))
                myWindow.Servo3_angle_txt.setText(str(robo_servo3_angle_lst[-1]))

                # print("RoboticsCar_IMU_Data")
                myWindow.Message_ID2_txt.setText(str(imu_message_ID_lst[-1]))
                myWindow.Data_length2_txt.setText(str(int(str(imu_data_length_lst[-1]), 16)))
                myWindow.Index2_txt.setText(str(int(str(imu_index_lst[-1]), 16)))
                myWindow.Time_stamp2_txt.setText(str(imu_time_stamp1_lst[-1]))
                myWindow.gyro2_txt.setText(str(int(str(imu_gyro_lst[-1]), 16)))
                myWindow.gyro_range2_txt.setText(str(int(str(imu_gyro_range_lst[-1]), 16)))
                myWindow.acc2_txt.setText(str(int(str(imu_acc_lst[-1]), 16)))
                myWindow.acc_range2_txt.setText(str(int(str(imu_acc_range_lst[-1]), 16)))
                myWindow.mag_odr2_txt.setText(str(int(str(imu_mag_odr_lst[-1]), 16)))
                myWindow.mag_rep_xy2_txt.setText(str(int(str(imu_mag_rep_xy_lst[-1]), 16)))
                myWindow.mag_rep_z2_txt.setText(str(int(str(imu_mag_rep_z_lst[-1]), 16)))

                myWindow.data_id2_1_txt.setText(str(imu_data_id1_lst[-1]))
                myWindow.gyrox2_txt.setText(str(imu_gyrox_n1_lst[-1]))
                myWindow.gyroy2_txt.setText(str(imu_gyroy_n1_lst[-1]))
                myWindow.gyroz2_txt.setText(str(imu_gyroz_n1_lst[-1]))
                myWindow.data_id2_2_txt.setText(str(imu_data_id2_lst[-1]))
                myWindow.accx2_txt.setText(str(imu_accx_n1_lst[-1]))
                myWindow.accy2_txt.setText(str(imu_accy_n1_lst[-1]))
                myWindow.accz2_txt.setText(str(imu_accz_n1_lst[-1]))
                myWindow.acc_temp2_txt.setText(str(imu_acc_temp_n_lst[-1]))
                myWindow.data_id2_3_txt.setText(str(imu_data_id3_lst[-1]))
                myWindow.compassx2_txt.setText(str(imu_compassx_n1_lst[-1]))
                myWindow.compassy2_txt.setText(str(imu_compassy_n1_lst[-1]))
                myWindow.compassz2_txt.setText(str(imu_compassz_n1_lst[-1]))
                # self.id_and_data2_txt.setText(data_id_and_data_lst[-1])
                time.sleep(0.1)
            else:
                print("Break Show Txt")
                break
        # else:
        #
        #         if self.a == 0:
        #             break


# class key_thread():
#     def __init__(self):
#         global keyboard_val,up,right, left, down
#         while True:
#             # print("key")
#             # print(keyboard_val)
#             if keyboard_val == 1:
#                 if keyboard.is_pressed("up") or keyboard.is_pressed("down") or keyboard.is_pressed("left") or keyboard.is_pressed("right"):
#                     if keyboard.is_pressed("up"):
#                         if keyboard.is_pressed("left"):
#                             up = 1
#                             down = 0
#                             right = 0
#                             left = 1
#
#                         elif keyboard.is_pressed("right"):
#                             up = 1
#                             down = 0
#                             right = 1
#                             left = 0
#
#                         else:
#                             up = 1
#                             down =0
#                             right = 0
#                             left = 0
#
#
#
#                     if keyboard.is_pressed("down"):
#                         if keyboard.is_pressed("left"):
#                             up = 0
#                             down = 1
#                             right = 0
#                             left = 1
#
#                         elif keyboard.is_pressed("right"):
#                             up = 0
#                             down = 1
#                             right = 1
#                             left = 0
#
#                         else:
#                             up = 0
#                             down =1
#                             right = 0
#                             left = 0
#                     if keyboard.is_pressed("right"):
#                         if keyboard.is_pressed("up"):
#                             up = 1
#                             down = 0
#                             right = 1
#                             left = 0
#                         elif keyboard.is_pressed("down"):
#                             up = 0
#                             down = 1
#                             right = 1
#                             left = 0
#                         else:
#
#                             up = 0
#                             down = 0
#                             right = 1
#                             left = 0
#
#                     if keyboard.is_pressed("left"):
#                         if keyboard.is_pressed("up"):
#                             up = 1
#                             down = 0
#                             right = 0
#                             left = 1
#                         elif keyboard.is_pressed("down"):
#                             up = 0
#                             down = 1
#                             right = 0
#                             left = 1
#                         else:
#                             up = 0
#                             down = 0
#                             right = 0
#                             left = 1
#
#                     time.sleep(0.05)
#
#
#                 else:
#                     up = 0
#                     down = 0
#                     right = 0
#                     left = 0
#
#                     time.sleep(0.05)
#                 WindowClass().keyboard_control()
#
#
#             else:
#                 pass
#             time.sleep(0.0001)


if __name__ == "__main__":
    # QApplication : Class that executes the program
    app = QApplication(sys.argv)
    # Make WindowClass instance
    myWindow = WindowClass()
    # Code for show program
    myWindow.show()

    # serial_thr = threading.Thread(target=Serial)
    # serial_thr.start()
    try:
        serial_init = Serial()  ## Start serial

        get = threading.Thread(target=Get_data)
        get.daemon = True
        get.start()


        parsing = threading.Thread(target=Parsing_data)
        parsing.daemon = True
        parsing.start()
        time.sleep(0.01)

        show_txt_thread = threading.Thread(target = Show_txt)
        show_txt_thread.daemon = True
        show_txt_thread.start()

        # key_thr = threading.Thread(target=key_thread)
        # key_thr.start()
    except serial.serialutil.SerialException:
        print("could not open port 'COM'")


    # app.exec_()
    sys.exit(app.exec_())
    # sys.exit()