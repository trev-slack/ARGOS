#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import Enum
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import  QWidget, QLabel, QApplication, QAction, QMainWindow, QHBoxLayout, QVBoxLayout, QMessageBox
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QIcon

# joystick utility
class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class JoyPub():
    def __init__(self,lin,ang):
        self.latch = False
        self.max_linear_speed = lin
        self.max_angular_speed = ang
        #self.joy_pub = rospy.Publisher("/argos/argos_velocity_controller/cmd_vel",Twist,queue_size=10)

    # def latchmessage(self,msg):
    #     #while self.latch == True:
    #     self.joy_pub.publish(msg)
    #         #rospy.sleep(10)



# joystick class
class Joystick(QWidget):
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(400, 400)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 200
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1
        #self.pub = JoyPub(self.max_linear_speed,self.max_angular_speed)
        #self.rosnode = rospy.init_node('Joy_node',anonymous=True)
        #self.joy_pub = rospy.Publisher("/argos/argos_velocity_controller/cmd_vel",Twist,latch=True)

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width()/2, self.height()/2)


    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        return (angle,distance)
        # if 45 <= angle < 135:
        #     return (Direction.Up, distance)
        # elif 135 <= angle < 225:
        #     return (Direction.Left, distance)
        # elif 225 <= angle < 315:
        #     return (Direction.Down, distance)
        # return (Direction.Right, distance)


    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.pub.latch = False
        self.update()


    def mouseMoveEvent(self, event):
        if self.grabCenter:
            print("Moving")
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        # publish to ros
        #self.joy_val = self.joystickDirection()
        print(self.joy_val)
        
        # ang = -math.cos(self.joy_val[0]*math.pi/180)
        # turning = ang*self.max_angular_speed
        # lin = self.joy_val[1]*self.max_linear_speed*np.sign(math.sin(self.joy_val[0]*math.pi/180))
        # move_msg = Twist()
        # move_msg.linear.x = lin
        # move_msg.angular.z = turning
        # self.pub.latch = True
        # self.pub.latchmessage(move_msg)
        #print(self.joy_val)