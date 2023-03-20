#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from anti_drone.msg import custom_msgs
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

class plot():

    def __init__(self):

        self.ax = plt.figure().gca(projection='3d')

        # circular rosbag

        # self.x = -13.85009479522705
        # self.y = -16.234638214111328
        # self.z = 9.992222785949707

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.x_anti = 0.0
        self.y_anti = 0.0
        self.z_anti = 0.0

        self.vel_x = 0.001
        self.vel_y = 0.001
        self.vel_z = 0.001

        self.flag = 0.0
        self.flag_2 = 0.0

        self.net_launch_x = 0.0
        self.net_launch_y = 0.0
        self.net_launch_z = 0.0
        self.throw_net = False

        rospy.Subscriber('/mavros/local_position/pose/old', PoseStamped, self.pose_callback)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.anti_drone_pose_callback)

        rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel', TwistStamped, self.anti_drone_vel_callback)

        rospy.Subscriber('/custom_topic', custom_msgs, self.custom_topic)

    def anti_drone_vel_callback(self, vel_msg):

        self.vel_x = vel_msg.twist.linear.x
        self.vel_y = vel_msg.twist.linear.y
        self.vel_z = vel_msg.twist.linear.z

    def pose_callback(self, pose_msg):

        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.z = pose_msg.pose.position.z

    def anti_drone_pose_callback(self, anti_pose_msg):

        self.x_anti = anti_pose_msg.pose.position.x
        self.y_anti = anti_pose_msg.pose.position.y
        self.z_anti = anti_pose_msg.pose.position.z

    def custom_topic(self, custom_msg):

        self.net_launch_x = custom_msg.net_launch_x
        self.net_launch_y = custom_msg.net_launch_y
        self.net_launch_z = custom_msg.net_launch_z
        self.throw_net = custom_msg.throw_net

    def plotter(self):

        rate = rospy.Rate(20)

        ax = plt.axes(projection='3d')

        # drone position array
        s1 = []
        s2 = []
        s3 = []

        # anti-drone position array
        s4 = []
        s5 = []
        s6 = []

        # impact position array
        s7 = []
        s8 = []
        s9 = []

        # drone current position array
        s10 = []
        s11 = []
        s12 = []

        # ani-drone position array
        s13 = []
        s14 = []
        s15 = []

        # net launch position
        s16 = []
        s17 = []
        s18 = []

        while not rospy.is_shutdown():

            if (self.flag_2 != 1.0):

                s1.append(self.x)
                s2.append(self.y)
                s3.append(self.z)

                s10.append(self.x)
                s11.append(self.y)
                s12.append(self.z)

            s4.append(self.x_anti)
            s5.append(self.y_anti)
            s6.append(self.z_anti)

            s13.append(self.x_anti)
            s14.append(self.y_anti)
            s15.append(self.z_anti)

            s16.append(self.net_launch_x)
            s17.append(self.net_launch_y)
            s18.append(self.net_launch_z)


            ax.plot(s1, s2, s3, 'green', label = 'drone trajectory')
            ax.plot(s10, s11, s12, 'green', marker="o", markersize=10, label = 'drone_position')
            ax.plot(s4, s5, s6, 'red', label = 'anti_drone_trajectory')
            ax.plot(s13, s14, s15, 'red', marker="o", markersize=10, label = 'anti_drone_position')
            ax.plot(s16, s17, s18, 'black', label = 'anti_drone_position')

            if (self.throw_net == True and self.flag == 0.0):

                s7.append(self.x)
                s8.append(self.y)
                s9.append(self.z)

                ax.plot(s7, s8, s9, marker="*", markersize=20, markerfacecolor="blue" , label = 'Impact Point')
                self.flag = 1.0

            if (self.flag == 1.0):

                ax.plot(s7, s8, s9, marker="*", markersize=20, markerfacecolor="blue" , label = 'Impact Point')
                self.flag_2 = 1.0

            if (self.flag_2 != 1.0):

                s10.pop()
                s11.pop()
                s12.pop()

            s13.pop()
            s14.pop()
            s15.pop()

            plt.pause(0.001)

            plt.cla()

            # circular rosbag

            #ax.set_xlim([-25, 5])
            #ax.set_ylim([-25, 5])
            ax.set_zlim([0, 15])
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('Anti_drone system')

            rate.sleep()

if __name__ == '__main__':

    try:

        rospy.init_node("plotter")
        plot_obj = plot()
        plot_obj.plotter()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
