#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
from scale_truck_control.msg import lane_coef
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox
import matplotlib.gridspec as gridspec

width = rospy.get_param("/scale_truck_control/ROI/width", 1280)
height = rospy.get_param("/scale_truck_control/ROI/height", 720)
center_height = rospy.get_param("/scale_truck_control/LaneDetector/center_height", 0.5)
center_height = center_height*height

plt.rcParams["figure.figsize"] = (13.3, 7.5)
plt.rcParams["axes.grid"] = True

t = np.arange(0, 720, 1)

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.cv = self.ax.scatter([], [], s=60)
        self.L_data, self.R_data, self.C_data, self.Degree = [], [], [], []
        self.center_pos = 640
        self.text1 = self.fig.text(0.915,0.5,str(self.Degree))

    def Submit(self, event):
        print("Push")

    def plot_init(self):
        self.ax.set_xlim(0, 640)
        self.ax.set_ylim(0, 480)
        self.ax.invert_yaxis()
        plt.xticks(range(0,640,20))
        plt.yticks(range(0,480,30))
        self.fig.subplots_adjust(left = 0.05, bottom =0.05)
        return self.ax
    
    def lane_callback(self, msg):
        self.L_data = msg.left.a*np.power(t,2)+msg.left.b*t+msg.left.c
        self.R_data = msg.right.a*np.power(t,2)+msg.right.b*t+msg.right.c
        self.C_data = msg.center.a*np.power(t,2)+msg.center.b*t+msg.center.c
        center_value = msg.center.a*np.power(center_height,2)+msg.center.b*center_height+msg.center.c
    
    def update_plot(self, frame):
        plt.plot(self.L_data, t)
        plt.plot(self.R_data, t)
        plt.plot(self.C_data, t)
        data = np.hstack((self.center_pos, center_height))
        self.cv.set_offsets(data)
        #self.cv.set_offsets(self.Center_value, center_height)
        return self.ax

if __name__ == '__main__':
    vis = Visualiser()
    rospy.init_node('lane_visual_node')
    sub = rospy.Subscriber('/lane_msg', lane_coef, vis.lane_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True) 
