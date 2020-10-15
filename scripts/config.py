#!/usr/bin/env python
import sys
print(sys.version)
import warnings
warnings.filterwarnings('ignore')

import rospy
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QSlider, QLineEdit, QBoxLayout, QWidget
from PyQt5.QtCore import Qt 

class ConfigWindow(QWidget):
    def __init__(self):
        QWidget.__init__(self, flags=Qt.Widget)
        self.init_widget()
        self.load_params()

    def load_params(self):
        self.x_dist = rospy.get_param("/scale_truck_control/params/width2dist")
        self.y_dist = rospy.get_param("/scale_truck_control/params/dist")
        self.SteerKp = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Kp")
        self.SteerKi = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Ki")
        self.SteerKd = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Kd")
        self.Steerdt = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/dt")
        self.tbSteerXd.setText(str(self.x_dist))
        self.tbSteerYd.setText(str(self.y_dist))
        self.tbSteerKp.setText(str(self.SteerKp))
        self.tbSteerKi.setText(str(self.SteerKi))
        self.tbSteerKd.setText(str(self.SteerKd))

        self.TargetSpeed = rospy.get_param("/scale_truck_control/params/target_speed")
        self.SpeedMode = rospy.get_param("/scale_truck_control/params/speed_mode")
        self.SpeedKp = rospy.get_param("/scale_truck_control/params/pid/Kp")
        self.SpeedKi = rospy.get_param("/scale_truck_control/params/pid/Ki")
        self.SpeedKd = rospy.get_param("/scale_truck_control/params/pid/Kd")
        self.tbSpeedTs.setText(str(self.TargetSpeed))
        self.tbSpeedSm.setText(str(self.SpeedMode))
        self.tbSpeedKp.setText(str(self.SpeedKp))
        self.tbSpeedKi.setText(str(self.SpeedKi))
        self.tbSpeedKd.setText(str(self.SpeedKd))

    def SetSteerPID(self):
        self.status.setText("Set Steer Params!")
        rospy.set_param("/scale_truck_control/params/width2dist", float(self.tbSteerXd.text()))
        rospy.set_param("/scale_truck_control/params/dist", float(self.tbSteerYd.text()))
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Kp",float(self.tbSteerKp.text()))
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Ki",float(self.tbSteerKi.text()))
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Kd",float(self.tbSteerKd.text()))
        self.load_params()

    def SetSpeedPID(self):
        self.status.setText("Set Speed Params!")
        rospy.set_param("/scale_truck_control/params/target_speed",float(self.tbSpeedTs.text()))
        rospy.set_param("/scale_truck_control/params/speed_mode",float(self.tbSpeedSm.text()))
        rospy.set_param("/scale_truck_control/params/Kp",float(self.tbSpeedKp.text()))
        rospy.set_param("/scale_truck_control/params/Ki",float(self.tbSpeedKi.text()))
        rospy.set_param("/scale_truck_control/params/Kd",float(self.tbSpeedKd.text()))
        self.load_params()

    def init_widget(self):
        self.setWindowTitle("Config Params")
        self.setGeometry(100, 100, 300, 100) # x, y, width, height

        self.lSteerTitle = QLabel("STEER")
        self.lSteerTitle.setAlignment(Qt.AlignCenter)
        self.lSteerXd = QLabel("X")
        self.tbSteerXd = QLineEdit(self)
        self.lSteerYd = QLabel("Y")
        self.tbSteerYd = QLineEdit(self)
        self.lSteerKp = QLabel("Kp")
        self.tbSteerKp = QLineEdit(self)
        self.lSteerKi = QLabel("Ki")
        self.tbSteerKi = QLineEdit(self)
        self.lSteerKd = QLabel("Kd")
        self.tbSteerKd = QLineEdit(self)
        self.pbSteer = QtWidgets.QPushButton(self)
        self.pbSteer.setObjectName("SteerPID")
        self.pbSteer.setText("SET PID")
        self.pbSteer.clicked.connect(self.SetSteerPID)
         
        leftLayout = QtWidgets.QGridLayout()
        leftLayout.addWidget(self.lSteerTitle,0,0,1,2)
        leftLayout.addWidget(self.lSteerXd, 1,0)
        leftLayout.addWidget(self.tbSteerXd, 1,1)
        leftLayout.addWidget(self.lSteerYd, 2,0)
        leftLayout.addWidget(self.tbSteerYd, 2,1)
        leftLayout.addWidget(self.lSteerKp, 3,0)
        leftLayout.addWidget(self.tbSteerKp, 3,1)
        leftLayout.addWidget(self.lSteerKi, 4,0)
        leftLayout.addWidget(self.tbSteerKi, 4,1)
        leftLayout.addWidget(self.lSteerKd, 5,0)
        leftLayout.addWidget(self.tbSteerKd, 5,1)
        leftLayout.addWidget(self.pbSteer, 6,0,1,2)

        self.lSpeedTitle = QLabel("SPEED")
        self.lSpeedTitle.setAlignment(Qt.AlignCenter)
        self.lSpeedTs = QLabel("Target")
        self.tbSpeedTs = QLineEdit(self)
        self.lSpeedSm = QLabel("Mode")
        self.tbSpeedSm = QLineEdit(self)
        self.lSpeedKp = QLabel("Kp")
        self.tbSpeedKp = QLineEdit(self)
        self.lSpeedKi = QLabel("Ki")
        self.tbSpeedKi = QLineEdit(self)
        self.lSpeedKd = QLabel("Kd")
        self.tbSpeedKd = QLineEdit(self)
        self.pbSpeed = QtWidgets.QPushButton(self)
        self.pbSpeed.setObjectName("SpeedPID")
        self.pbSpeed.setText("SET PID")
        self.pbSpeed.clicked.connect(self.SetSpeedPID)

        rightLayout = QtWidgets.QGridLayout()
        rightLayout.addWidget(self.lSpeedTitle,0,0,1,2)
        rightLayout.addWidget(self.lSpeedTs, 1,0)
        rightLayout.addWidget(self.tbSpeedTs, 1,1)
        rightLayout.addWidget(self.lSpeedSm, 2,0)
        rightLayout.addWidget(self.tbSpeedSm, 2,1)
        rightLayout.addWidget(self.lSpeedKp, 3,0)
        rightLayout.addWidget(self.tbSpeedKp, 3,1)
        rightLayout.addWidget(self.lSpeedKi, 4,0)
        rightLayout.addWidget(self.tbSpeedKi, 4,1)
        rightLayout.addWidget(self.lSpeedKd, 5,0)
        rightLayout.addWidget(self.tbSpeedKd, 5,1)
        rightLayout.addWidget(self.pbSpeed, 6,0,1,2)

        self.status = QLabel(self)
        self.status.setAlignment(Qt.AlignCenter)
        mainLayout = QtWidgets.QGridLayout()
        mainLayout.addWidget(self.status,0,0,1,2)
        mainLayout.addLayout(leftLayout,1,0)
        mainLayout.addLayout(rightLayout,1,1)
        self.setLayout(mainLayout)

if __name__ == "__main__":
    rospy.init_node('config_node')
    app = QtWidgets.QApplication(sys.argv)
    mainWin = ConfigWindow()
    mainWin.show()
    sys.exit( app.exec_() )
