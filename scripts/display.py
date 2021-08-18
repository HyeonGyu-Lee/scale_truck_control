#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
from scale_truck_control.msg import Lane
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
        self.ll, = self.ax.plot([], [], '-b', linewidth=3)
        self.rl, = self.ax.plot([], [], '-r', linewidth=3)
        self.cl, = self.ax.plot([], [], '-g', linewidth=3)
        self.cv = self.ax.scatter([], [], s=60)
        self.L_data, self.R_data, self.C_data, self.Degree = [], [], [], []
        self.center_pos = 640
        self.I_err = 0;
        self.D_err = 0;
        self.prev_err = 0;
        self.x_pix2dist = rospy.get_param("/scale_truck_control/params/width2dist",30)
        self.y_pix2dist = rospy.get_param("/scale_truck_control/params/dist")
        self.Angle_Kp = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Kp", 0.5)
        self.Angle_Ki = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Ki", 0.001)
        self.Angle_Kd = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/Kd", 0.01)
        self.dt = rospy.get_param("/scale_truck_control/LaneDetector/pid_params/dt", 0.1)
        self.Steer_Kp = 8.0
        self.Steer_Ki = 0.0001
        self.Steer_Kd = 0.25
        self.text1 = self.fig.text(0.915,0.5,str(self.Degree))

    def Submit(self, event):
        print("Push")
        self.set_params()

    def set_params(self):
        data = [float(self.tb.text) for self.tb in [self.tb_pa,self.tb_ia,self.tb_da,self.tb_ps,self.tb_is,self.tb_ds]]
        print(data)
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Kp", data[0])
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Ki", data[1])
        rospy.set_param("/scale_truck_control/LaneDetector/pid_params/Kd", data[2])
        rospy.set_param("/scale_truck_control/params/pid/Kp", data[3])
        rospy.set_param("/scale_truck_control/params/pid/Ki", data[4])
        rospy.set_param("/scale_truck_control/params/pid/Kd", data[5])


    def plot_init(self):
        self.ax.set_xlim(0, 1280)
        self.ax.set_ylim(0, 720)
        self.ax.invert_yaxis()
        plt.xticks(range(0,1280,160))
        plt.yticks(range(0,720,180))
        self.fig.subplots_adjust(left = 0.05, bottom =0.05)
        self.gs = gridspec.GridSpec(6,1)
        self.gs.update(left=0.95, right=0.985, bottom=0.15, top=0.45, hspace=0.2)
        axesbutton = plt.axes([0.925,0.05,0.05,0.05])
        axes = [self.fig.add_subplot(self.gs[i,j]) for i,j in [[0,0],[1,0],[2,0],[3,0],[4,0],[5,0]]]
        self.tb_pa = TextBox(axes[0],'A_p', hovercolor='0.975')
        self.tb_ia = TextBox(axes[1],'A_i', hovercolor='0.975')
        self.tb_da = TextBox(axes[2],'A_d', hovercolor='0.975')
        self.tb_ps = TextBox(axes[3],'S_p', hovercolor='0.975')
        self.tb_is = TextBox(axes[4],'S_i', hovercolor='0.975')
        self.tb_ds = TextBox(axes[5],'S_d', hovercolor='0.975')
        self.bsubmit = Button(axesbutton, 'Submit')
        self.bsubmit.on_clicked(self.Submit)
        self.tb_pa.text = str(self.Angle_Kp)
        self.tb_ia.text = str(self.Angle_Ki)
        self.tb_da.text = str(self.Angle_Kd)
        self.tb_ps.text = str(self.Steer_Kp)
        self.tb_is.text = str(self.Steer_Ki)
        self.tb_ds.text = str(self.Steer_Kd)
        #for self.tb in [self.tb_pa,self.tb_ia,self.tb_da,self.tb_ps,self.tb_is,self.tb_ds]:
        #    self.tb.on_submit(self.set_params)
        return self.ax
    
    def lane_callback(self, msg):
        self.L_data = msg.Left.x*np.power(t,2)+msg.Left.y*t+msg.Left.z
        self.R_data = msg.Right.x*np.power(t,2)+msg.Right.y*t+msg.Right.z
        self.C_data = msg.Center.x*np.power(t,2)+msg.Center.y*t+msg.Center.z
        center_value = msg.Center.x*np.power(center_height,2)+msg.Center.y*center_height+msg.Center.z
        if (center_value >= 0) and (center_value <= 1280):
            err = center_value - self.center_pos
            self.I_err += err * self.dt
            self.D_err = (err - self.prev_err) / self.dt
            self.prev_err = err
            result = self.Angle_Kp * err + self.Angle_Ki * self.I_err + self.Angle_Kd * self.D_err
            self.center_pos += result
        weight = (self.center_pos - 640) / 640 * (-1.0)
        self.Degree = np.arctan(self.x_pix2dist*weight/self.y_pix2dist)*(180/np.pi)
        self.text1.set_text(str(round(self.Degree,3))+" Degree");
    
    def update_plot(self, frame):
        self.ll.set_data(self.L_data, t)
        self.rl.set_data(self.R_data, t)
        self.cl.set_data(self.C_data, t)
        data = np.hstack((self.center_pos, center_height))
        self.cv.set_offsets(data)
        #self.cv.set_offsets(self.Center_value, center_height)
        return self.ax

if __name__ == '__main__':
    vis = Visualiser()
    rospy.init_node('lane_visual_node')
    sub = rospy.Subscriber('/lane_msg', Lane, vis.lane_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True) 
