#!/usr/bin/python3

__author__ = "Trevor Slack, Luca Kushner"
__copyright__ = "Copyright 2021, ARGOS"
__credits__ = ["Trevor Slack", "Luca Kushner"]
__license__ = "GPL"
__version__ = "1.2.1"
__maintainer__ = "Trevor Slack"
__email__ = "trevorpslack@gmail.com"
__status__ = "Production"

import sys
import datetime
import time
import rospy
import math
import numpy as np
import tf
# video feed
import cv2
# pyqt
from PyQt5.QtWidgets import  QWidget, QTabWidget, QTextEdit, QLabel, QPushButton, QLineEdit, QApplication, QAction, QMainWindow, QHBoxLayout, QVBoxLayout, QMessageBox
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWebEngineWidgets import *
# map
from map_widget import Map
# armed led
from QLed import QLed
# ros messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import Range, NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus
import inputs
from inputs import get_gamepad


# global variables
move_msg = Twist()
# use to lock global variable to prevent parallel accessing
mutex = QMutex()

# Temperature feed
class TemperatureFeed(QLabel):
    def __init__(self, parent=None):
        super(TemperatureFeed, self).__init__(parent)
        self.setText("Temperature [k]: Waiting for Signal")
        self.setFont(QFont('Helvetica', 40)) 
        self.setStyleSheet("background-color: red; border: 1px solid black")

    # update temp
    def update(self,value):
        self.setText("Temperature [k]: " + str(value))
        # background color based on temp (start blue -> red)
        r = 255/398*value
        g = 0
        b = -255/398*value+255
        if r<0:
            r = 0
        elif r>225:
            r = 225
        if b<0:
            b=0
        elif b>225:
            b = 225
        self.setStyleSheet("background-color: rgb(" + str(r) + "," + str(g) + "," + str(b) + "); border: 1px solid black")

# Communication Feed
class CommunicationStatus(QLabel):
    def __init__(self, parent=None):
        super(CommunicationStatus, self).__init__(parent)
        self.setText("Comms: Waiting For Signal")
        self.setFont(QFont('Helvetica', 40)) 
        self.setStyleSheet("background-color: red; border: 1px solid black")
        self.last_ping = time.time()

    # update comms status
    def update(self,status):
        # get time from last update
        value = round((time.time()-self.last_ping),2)
        self.last_ping = time.time()
        if status == 1:
            self.setText("Comms: Connected. ping [s]:" + str(value))
            self.setStyleSheet("background-color: green; border: 1px solid black")
        elif status == -1:
            self.setText("Comms: Disconnected")
            self.setStyleSheet("background-color: red; border: 1px solid black")

# Tilt Angle
class TiltAngle(QLabel):
    def __init__(self, axis,parent=None):
        super(TiltAngle, self).__init__(parent)
        self.axis = axis
        self.setText("Tilt " + self.axis + " [deg]: Waiting for Signal")
        self.setFont(QFont('Helvetica', 40)) 
        self.setStyleSheet("background-color: red; border: 1px solid black")
    
    # update tilt angle, color based on deploy limits
    def update(self,value):
        self.setText("Tilt " + self.axis + " [deg]: " + str(value))
        self.setStyleSheet("background-color: white; border: 1px solid black")
        if abs(value) > 15:
            if abs(value) > 20:
                self.setStyleSheet("background-color: red; border: 1px solid black")
            else:
                self.setStyleSheet("background-color: orange; border: 1px solid black")
        else:
            self.setStyleSheet("background-color: green; border: 1px solid black")


# Live Video Feed Thread
class Thread(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        # video location, can be 0,1,... for live video feed
        self.cap = cv2.VideoCapture(0)
        self.playback = True
        # loop to update image
        while self.playback:
            ret, frame = self.cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(1350, 600, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
        self.cap.release()

# URF Display
class URFBar(QLabel):
    def __init__(self, parent=None):
        super(URFBar, self).__init__(parent)
        self.setText("URF\nWaiting for Signal")
        self.setFont(QFont('Helvetica', 20)) 
        self.setStyleSheet("background-color: red; border: 1px solid black")
        self.resize(400,100)

    # update temp
    def update(self,value):
        if value < 0:
            self.setText("Out of Range")
            self.setStyleSheet("background-color: white; border: 1px solid black")
            return
        value = round(value,2)
        self.setText(str(value) + " [m]")
        self.setStyleSheet("background-color: white; border: 1px solid black")
        # background color based on distance (white -> red)
        r = -255/4*value+255
        g = 255/4*value
        b = 0
        if r<0:
            r = 0
        elif r>225:
            r = 225
        if g<0:
            g=0
        elif g>225:
            g = 225
        self.setStyleSheet("background-color: rgb(" + str(r) + "," + str(g) + "," + str(b) + "); border: 1px solid black")

# joystick thread
class GamepadThread(QObject):
    status = pyqtSignal(str)

    def run(self):
        self.joy_x = 0
        self.joy_y = 0
        self.dead_zone = 3500
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1
        pads = inputs.devices.gamepads
        if len(pads) == 0:
            self.status.emit("Error: Gamepad Not Connected! Please connect controller and restart UI")
            return
        while not rospy.is_shutdown():
            # get events from controller
            events = get_gamepad()
            for event in events:
                if event.code == "ABS_X":
                    # dead zone
                    if abs(event.state)<self.dead_zone:
                        self.joy_x = 0
                    else:
                        self.joy_x = event.state
                elif event.code == "ABS_Y":
                    # dead zone
                    if abs(event.state)<self.dead_zone:
                        self.joy_y = 0
                    else:
                        self.joy_y = event.state
            global move_msg
            mutex.lock()
            move_msg = self.command()
            mutex.unlock()

    def command(self):
        move = Twist()
        theta = math.atan2(self.joy_x,self.joy_y)
        magnitude = math.sqrt(self.joy_x**2+self.joy_y**2)
        linear_mag = magnitude/38000*self.max_linear_speed
        angular_mag = magnitude/38000*self.max_angular_speed
        direction_mag = math.sin(theta)
        if abs(theta)>math.pi/2:
            linear_dir = 1
        else:
            linear_dir = -1

        move.linear.x = linear_mag*linear_dir*abs(math.cos(theta))
        move.angular.z = angular_mag*-direction_mag
        
        return move

# run move publisher on different thread
class GamepadPublishThread(QObject):

    def run(self):
        self.flag = 1
        #self.move = Twist()
        global move_msg
        self.move_pub = rospy.Publisher("/argos/argos_velocity_controller/cmd_vel",Twist,queue_size=10)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # armed
            if self.flag == 0:
                mutex.lock()
                self.move_pub.publish(move_msg)
                mutex.unlock()
            else:
                # disarmed
                m = Twist()
                m.linear.x = 0
                m.angular.z = 0
                self.move_pub.publish(m)
            r.sleep()

    def pause(self):
        self.flag = 1

    def resume(self):
        self.flag = 0

# Thermometer thread
class TempThread(QObject):
    temp_data = pyqtSignal(float)
    temp_warn = pyqtSignal(str)

    def run(self):
        self.temp_sub = rospy.Subscriber("/argos/temperature", Float32, self.tempCallback)

    # Temperature sensor callback
    def tempCallback(self,d):
        temp = d.data
        self.temp_data.emit(temp)
        if temp > 343.15:
            self.temp_warn.emit("Danger: Reached Commercial Electronics Temperature Limit")
        elif temp > 338:
            self.temp_warn.emit("Warn: Approaching Commercial Electronics Temperature Limit of 343.15K")


# IMU Thread
class IMUThread(QObject):
    x_data = pyqtSignal(float)
    y_data = pyqtSignal(float)

    def run(self):
        self.tilt_sub = rospy.Subscriber("/argos/odometry/local_filtered",Odometry, self.imuCallback)

    # IMU Callback
    def imuCallback(self,data):
        q = data.pose.pose.orientation
        quat = [q.x,q.y,q.z,q.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        x = euler[0]*(180/3.14159)
        y = euler[1]*(180/3.14159)
        x = round(x,2)
        y = round(y,2)
        self.x_data.emit(x)
        self.y_data.emit(y)


# URF Thread
class URFThread(QObject):
    left_data = pyqtSignal(float)
    right_data = pyqtSignal(float)
    dist_warn = pyqtSignal(str)

    def run(self):
        self.urf_sub = rospy.Subscriber("/argos/urf",Float32MultiArray, self.urfCallback)

    def urfCallback(self,d):
        left = d.data[0]
        right = d.data[1]
        self.left_data.emit(left)
        self.right_data.emit(right)
        if left<1.15 and left>0:
            self.dist_warn.emit("Danger: Too close to obstacle on left side")
        elif left<1.3 and left>0:
            self.dist_warn.emit("Warn: Object on left side approaching 1.15m safety distance")
        if right<1.15 and right>0:
            self.dist_warn.emit("Danger: Too close to obstacle on right side")
        elif left<1.3 and right>0:
            self.dist_warn.emit("Warn: Object on right side approaching 1.15m safety distance")

class CommsThread(QObject):
    ping = pyqtSignal(float)
    max_lapse = 5

    def run(self):
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            try:
                tmp = rospy.wait_for_message("/argos/status",DiagnosticStatus,timeout=self.max_lapse)
                self.ping.emit(1)
            except:
                self.ping.emit(-1)
            r.sleep()


class PingPubClass(QObject):
    temp = pyqtSignal(float)

    def run(self):
        self.comms_pub = rospy.Publisher("/ground_station/status",DiagnosticStatus,queue_size=10)
        r = rospy.Rate(10)
        while not rospy.is_shutdown(): 
            m = DiagnosticStatus()
            m.level = 0
            self.comms_pub.publish(m)  
            r.sleep()     
    #     self.comms_sub = rospy.Subscriber("/argos/status",DiagnosticStatus,self.commsCallback)

    # def commsCallback(self,msg):
    #     m = DiagnosticStatus()
    #     m.level = 0
    #     self.comms_pub.publish(m)
    #     p = float(msg.message)
    #     print(p)
    #     if p >= 1:
    #         self.ping.emit(-1)
    #     else:
    #         self.ping.emit(1)



# GUI Class
class ARGOSGUI(QWidget):
    def __init__(self, parent,start_lat,start_lng):
        super(QWidget, self).__init__(parent)
        self.extended = False
        self.CT = False
        self.armed = False
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1
        self.start_lat = start_lat
        self.start_lng = start_lng
        self.initUI()

    # create UI
    def initUI(self):
        # create movement tab objects
        self.createMovement()
        # create survallience tab objects
        self.createSurveillance()

        # intialize tab
        self.tabs = QTabWidget()
        self.tab_move = QWidget()
        self.tab_surv = QWidget()
        self.tabs.addTab(self.tab_move,"Movement")
        self.tabs.addTab(self.tab_surv,"Surveillance")

        # add to tabs
        self.tab_move.setLayout(self.layout_movement)
        self.tab_surv.setLayout(self.layout_surv)

        # tab layout
        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)



    # Surveillance Tab
    def createSurveillance(self):

        ''' Extend Led '''
        self.extend_button = QPushButton()
        self.extend_led = QLed(onColour=QLed.Green, shape=QLed.Circle)
        self.extend_button.setText("Extend")
        self.extend_button.clicked.connect(self.mastDeploy)
        self.extend_layout = QHBoxLayout()
        self.extend_layout.addWidget(self.extend_button,75)
        self.extend_layout.addWidget(self.extend_led,25)

        ''' MOSFET Button '''
        self.camera_on_button = QPushButton()
        self.camera_on_led = QLed(onColour=QLed.Green, shape=QLed.Circle)
        self.camera_on_button.setText("Turn Camera On")
        self.camera_on_button.clicked.connect(self.cameraOn)
        self.camera_on_layout = QHBoxLayout()
        self.camera_on_layout.addWidget(self.camera_on_button,75)
        self.camera_on_layout.addWidget(self.camera_on_led,25)

        ''' Harzards '''
        self.hazards_surv = QTextEdit()
        self.hazards_surv.setFontPointSize(16)

        ''' Status Box '''
        # thermometer
        self.thermometer_surv = TemperatureFeed()
        # comms
        self.comms_surv = CommunicationStatus()
        # tilt angle
        self.tiltx_surv = TiltAngle("X")
        self.tilty_surv = TiltAngle("Y")

        ''' Layouts '''
        # left vertical box
        self.lvbox_surv = QVBoxLayout()
        self.lvbox_surv.addStretch(2)
        self.lvbox_surv.addLayout(self.extend_layout,1)
        self.lvbox_surv.addLayout(self.camera_on_layout,1)
        # self.lvbox_surv.addLayout(self.web_layout,50)
        self.lvbox_surv.addWidget(self.hazards_surv,25)
        self.lvbox_surv.addWidget(self.comms_surv,6)
        self.lvbox_surv.addWidget(self.thermometer_surv,6)
        self.lvbox_surv.addWidget(self.tiltx_surv,6)
        self.lvbox_surv.addWidget(self.tilty_surv,6)

         # right vertical box
        self.rvbox_surv = QVBoxLayout()
        self.rvbox_surv.addStretch(2)
        
        # first horizontal box
        self.layout_surv = QHBoxLayout()
        self.layout_surv.addLayout(self.lvbox_surv,25)


    # Movement Tab
    def createMovement(self):
        ''' Video '''
        # video feed
        self.th_camera = QThread()
        self.th_movement = Thread()
        self.th_movement.moveToThread(self.th_camera)
        self.th_camera.started.connect(self.th_movement.run)
        self.th_movement.changePixmap.connect(self.setImage)
        self.th_camera.start()

        # video feed label
        self.video_label = QLabel(self)
        self.video_label.resize(1350, 600)

        ''' Map '''
        self.myMap_movement = Map(self.start_lat,self.start_lng)

        self.way_in_btn = QPushButton()
        self.way_in_btn.clicked.connect(self.addNewWaypoint)
        self.way_in_btn.setText("Add Waypoint [Name,lat,lng]")

        self.lat_in = QLineEdit()
        self.lng_in = QLineEdit()
        self.name_in = QLineEdit()

        self.way_dist = QPushButton()
        self.way_dist.setText("Get Distance to Waypoint")
        self.way_dist.clicked.connect(self.getWayDist)

        self.way_in_layout = QHBoxLayout()
        self.way_in_layout.addWidget(self.way_in_btn,20)
        self.way_in_layout.addWidget(self.name_in,20)
        self.way_in_layout.addWidget(self.lat_in,20)
        self.way_in_layout.addWidget(self.lng_in,20)
        self.way_in_layout.addWidget(self.way_dist,20)


        ''' Armed LED '''
        self.arm_button = QPushButton()
        self.arm_button.setText("Arm")
        self.arm_led = QLed(onColour=QLed.Green, shape=QLed.Circle)
        self.arm_button.clicked.connect(self.armCallback)
        self.arm_layout = QHBoxLayout()
        self.arm_layout.addWidget(self.arm_button,75)
        self.arm_layout.addWidget(self.arm_led,25)

        ''' URF '''
        self.URFLeft = URFBar()
        self.URFRight = URFBar()
        self.joystick_layout = QHBoxLayout()
        self.joystick_layout.addWidget(self.URFLeft,15)
        self.joystick_layout.addWidget(self.URFRight,15)

        ''' Harzards '''
        self.hazards_movement = QTextEdit()
        self.hazards_movement.setFontPointSize(16)

        ''' Status Box '''
        # thermometer
        self.thermometer = TemperatureFeed()
        # comms
        self.comms_mov = CommunicationStatus()
        # tilt angle
        self.tiltx = TiltAngle("X")
        self.tilty = TiltAngle("Y")

        ''' Layouts '''
        # left vertical box
        self.lvbox = QVBoxLayout()
        self.lvbox.addStretch(2)
        self.lvbox.addLayout(self.arm_layout,1)
        self.lvbox.addLayout(self.joystick_layout,25)
        self.lvbox.addWidget(self.hazards_movement,25)
        self.lvbox.addWidget(self.comms_mov,6)
        self.lvbox.addWidget(self.thermometer,6)
        self.lvbox.addWidget(self.tiltx,6)
        self.lvbox.addWidget(self.tilty,6)
        # right vertical box
        self.rvbox = QVBoxLayout()
        self.rvbox.addStretch(1)
        self.rvbox.addWidget(self.video_label,40)
        self.rvbox.addWidget(self.myMap_movement,40)
        self.rvbox.addLayout(self.way_in_layout,20)

        # first horizontal box
        self.layout_movement = QHBoxLayout()
        self.layout_movement.addLayout(self.lvbox,25)
        self.layout_movement.addLayout(self.rvbox,75)

    # connects the controls to ROS topics using QThreads for each topic
    def connectToROS(self):
        # ros node


        ''' Joystick '''
        # create threads
        self.thr1 = QThread()
        self.joy_pad = GamepadThread()
        self.joy_pad.moveToThread(self.thr1)
        self.thr1.started.connect(self.joy_pad.run)
        self.joy_pad.status.connect(self.updateHazards)
        self.thr1.start()

        self.thr2 = QThread()
        self.joy_pub_utility = GamepadPublishThread()
        self.joy_pub_utility.moveToThread(self.thr2)
        self.thr2.started.connect(self.joy_pub_utility.run)
        self.thr2.start()

        ''' Temp '''
        self.thr3 = QThread()
        self.temp = TempThread()
        self.temp.moveToThread(self.thr3)
        self.thr3.started.connect(self.temp.run)
        self.temp.temp_data.connect(self.thermometer.update)
        self.temp.temp_data.connect(self.thermometer_surv.update)
        self.temp.temp_warn.connect(self.updateHazards)
        self.thr3.start()

        ''' IMU '''
        self.thr4 = QThread()
        self.imu = IMUThread()
        self.imu.moveToThread(self.thr4)
        self.thr4.started.connect(self.imu.run)
        self.imu.x_data.connect(self.tiltx.update)
        self.imu.x_data.connect(self.tiltx_surv.update)
        self.imu.y_data.connect(self.tilty.update)
        self.imu.y_data.connect(self.tilty_surv.update)
        self.thr4.start()

        ''' URF '''
        self.thr5 = QThread()
        self.urf = URFThread()
        self.urf.moveToThread(self.thr5)
        self.thr5.started.connect(self.urf.run)
        self.urf.left_data.connect(self.URFLeft.update)
        self.urf.right_data.connect(self.URFRight.update)
        self.urf.dist_warn.connect(self.updateHazards)
        self.thr5.start()

        ''' Comms '''
        self.thr6 = QThread()
        self.comms = CommsThread()
        self.comms.moveToThread(self.thr6)
        self.thr6.started.connect(self.comms.run)
        self.comms.ping.connect(self.comms_surv.update)
        self.comms.ping.connect(self.comms_mov.update)
        self.thr6.start()

        self.thr7 = QThread()
        self.comms_pinger = PingPubClass()
        self.comms_pinger.moveToThread(self.thr7)
        self.thr7.started.connect(self.comms_pinger.run)
        self.thr7.start()

        ''' Location Data '''
        self.gps_sub = rospy.Subscriber("/argos/gps/filtered", NavSatFix, self.gpsCallback)

        # mast publisher
        self.mast_pub = rospy.Publisher("/argos/mast_status",Bool,queue_size=1)

        # camera publisher
        self.camera_pub = rospy.Publisher("/argos/camera_status", Bool,queue_size=1)


    @pyqtSlot(QImage)
    def setImage(self, image):
        self.video_label.setPixmap(QPixmap.fromImage(image))

    @pyqtSlot()
    def on_click(self):
        print("\n")
        for currentQTableWidgetItem in self.tableWidget.selectedItems():
            print(currentQTableWidgetItem.row(), currentQTableWidgetItem.column(), currentQTableWidgetItem.text())


    # arm rover
    def armCallback(self):
        if self.armed == False:
            self.armed = True
            self.arm_led.value = True
            self.arm_button.setText("DISARM")
            # allow publishing
            self.joy_pub_utility.resume()
        else:
            self.armed = False
            self.arm_led.value = False
            self.arm_button.setText("ARM")
            # disable publishing
            self.joy_pub_utility.pause()

    # mast deploy rover
    def mastDeploy(self):
        if self.extended == False:
            self.extended = True
            self.extend_led.value = True
            self.extend_button.setText("Retract")
            d = Bool()
            d.data = self.extended
            self.mast_pub.publish(d)
        else:
            self.extended = False
            self.extend_led.value = False
            self.extend_button.setText("Extend")
            d = Bool()
            d.data = self.extended
            self.mast_pub.publish(d)

    #turn on camera
    def cameraOn(self):
        if self.CT == False:
            self.CT = True
            self.camera_on_led.value = True
            self.camera_on_button.setText("Turn Camera Off")
            d = Bool()
            d.data = self.CT
            self.camera_pub.publish(d)

        else:
            self.CT = False
            self.camera_on_led.value = False
            self.camera_on_button.setText("Turn Camera On")
            d = Bool()
            d.data = self.CT
            self.camera_pub.publish(d)

    # update hazards box
    def updateHazards(self,msg):
        text = msg.split(":")
        t = datetime.datetime.now()
        redColor = QColor(225,0,0)
        blackColor = QColor(0,0,0)
        orangeColor = QColor(225,165,0)
        # error message
        if text[0] == "Error" or text[0] == "Danger":
            self.hazards_movement.setTextColor(redColor)
            self.hazards_movement.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))
            self.hazards_surv.setTextColor(redColor)
            self.hazards_surv.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))            
        elif text[0] == "Warn":
            self.hazards_movement.setTextColor(orangeColor)
            self.hazards_movement.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))
            self.hazards_surv.setTextColor(orangeColor)
            self.hazards_surv.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))  
        elif text[0] == "Log":
            self.hazards_movement.setTextColor(blackColor)
            self.hazards_movement.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))
            self.hazards_surv.setTextColor(blackColor)
            self.hazards_surv.append("[{}:{}:{}] {}".format(t.hour,t.minute,t.second,msg))  

    # gps update
    def gpsCallback(self,msg):
        lat = msg.latitude
        lng = msg.longitude
        self.myMap_movement.updatePath(lat,lng)


    # add a new waypoint
    def addNewWaypoint(self):
        name = self.name_in.text()
        lat = float(self.lat_in.text())
        lng = float(self.lng_in.text())
        self.myMap_movement.createMarker(lat,lng,name)

    # get current distance to waypoint
    def getWayDist(self):
        d = self.myMap_movement.getDist()
        self.updateHazards("Log: Distance to Waypoint = {} m".format(round(d,2)))


class App(QMainWindow):

    def __init__(self,start_lat,start_lng):
        super().__init__()
        self.left = 100
        self.top = 100
        self.width = 640
        self.height = 480
        self.setWindowTitle('ARGOS GUI')
        ''' Top bar '''
        # exit button
        self.exitAct = QAction(QIcon('exit.png'), '&Exit', self)
        self.exitAct.setShortcut('Ctrl+Q')
        self.exitAct.setStatusTip('Exit application')
        self.exitAct.triggered.connect(self.quitUI)
        # drop down bar
        self.statusBar()
        self.menubar = self.menuBar()
        self.fileMenu = self.menubar.addMenu('&File')
        self.fileMenu.addAction(self.exitAct)
        self.main_widget = ARGOSGUI(self,start_lat,start_lng)
        self.setCentralWidget(self.main_widget)
        self.resize(1800, 1200)
        self.setWindowIcon(QIcon('argos_icon.png'))
        self.setWindowTitle('ARGOS GUI')
        self.show()

        self.main_widget.connectToROS()


    # quit UI function
    def quitUI(self):
        # end video
        self.playback = False
        # end window
        self.close() 


def main(args):
    rosnode = rospy.init_node('GUI_node',anonymous=True)
    app = QApplication(sys.argv)
    ex = App(float(args[1]),float(args[2]))
    sys.exit(app.exec_())

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logfatal("Not enough argvs. Should be argos_gui.py <latitude> <longitude>")
    else:
        main(sys.argv)