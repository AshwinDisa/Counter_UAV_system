#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, OverrideRCIn
from anti_drone.msg import custom_msgs
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr
import tf

global time
time = 0.0

class controller():

    def __init__(self):

        self.looping_time = 0.05

        self.current_state = State()

        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def control(self):

        rate = rospy.Rate(1/self.looping_time)      # 20 Hz

        rospy.sleep(1)

        # main while loop

        while(not rospy.is_shutdown()):

            trajectory_mode.control()
            global time
            time += self.looping_time
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

class trajectory():

    def __init__(self):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.anti_drone_current_x = 0.0
        self.anti_drone_current_y = 0.0
        self.anti_drone_current_z = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.anti_drone_current_vel_x = 0.0
        self.anti_drone_current_vel_x = 0.0
        self.anti_drone_current_vel_x = 0.0

        self.looping_time = 0.05

        self.displacement = 0.0

        self.abs_weightage = 0.0
        self.rel_weightage = 0.0
        self.denominator = 0.0

        self.prev_vel_unit_sum = 0.0

        self.yaw_rate_P_gain = 0.02
        self.yaw_rate_D_gain = 0.01

        self.prev_yaw_error = 0.0

        self.throw_net = False

        self.array = np.array([])

        self.max_vel = 12.0

        self.vel_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                    TwistStamped, queue_size = 10)

        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

        self.custom_topic = rospy.Publisher('/custom_topic', custom_msgs,
                                                                queue_size = 10)

        rospy.Subscriber('/mavros/local_position/pose/old',
                            PoseStamped, self.drone_position_callback)

        rospy.Subscriber('/mavros/local_position/pose',
                            PoseStamped, self.anti_drone_position_callback)

        rospy.Subscriber('mavros/local_position/velocity_local/old',
                            TwistStamped, self.drone_velocity_callback)

        rospy.Subscriber('mavros/local_position/velocity_local',
                            TwistStamped, self.anti_drone_velocity_callback)

        rospy.Subscriber("/mavros/state", State, self.state_callback)

        self.rc_publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 10)

        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def control(self):

        x_j = self.drone_current_x
        x_i = self.anti_drone_current_x
        y_j = self.drone_current_y
        y_i = self.anti_drone_current_y
        v_j = math.sqrt(pow((self.drone_current_vel_x),2) +
                                        pow((self.drone_current_vel_y),2)
                                        + pow((self.drone_current_vel_z),2))

        # to estimate heading between anti-drone and drone in terms of degrees

        heading_yaw = 180 / math.pi * math.atan2((y_j - y_i),(x_j - x_i))

        # to estimate anti-drone current heading (yaw)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.anti_drone_quaternion)

        yaw_deg = 180 / math.pi * yaw

        # PD controller to control anti-drone heading towards drone at all times

        yaw_error = heading_yaw - yaw_deg

        drone_yaw_rate = self.yaw_rate_P_gain * (yaw_error) + self.yaw_rate_D_gain * (yaw_error - self.prev_yaw_error)

        self.prev_yaw_error = yaw_error

        # to estimate the heading of the drone

        drone_vel_vector = np.array([self.drone_current_vel_x, self.drone_current_vel_y,
                                                                    self.drone_current_vel_z])

        anti_drone_vel_vector = np.array([self.anti_drone_current_vel_x, self.anti_drone_current_vel_y,
                                                                    self.anti_drone_current_vel_z])

        drone_heading = self.heading(drone_vel_vector)

        anti_drone_heading = self.heading(anti_drone_vel_vector)

        drone_heading = math.atan2(drone_heading[1],drone_heading[0])

        anti_drone_heading = math.atan2(anti_drone_heading[1],anti_drone_heading[0])

        heading_error = drone_heading * 180 / math.pi - anti_drone_heading*180/math.pi

        # anti drone position == net launching position 1 meter behind and 1 meter above

        net_launch_x = math.cos(drone_heading) * -1 - math.sin(drone_heading) * 0 + self.drone_current_x
        net_launch_y = math.sin(drone_heading) * -1 + math.cos(drone_heading) * 0 + self.drone_current_y
        net_launch_z = self.drone_current_z + 1

        # to estimate heading between drone and anti-drone in terms of unit vector

        anti_drone_position_vector = np.array([- self.anti_drone_current_x + net_launch_x,
                                                - self.anti_drone_current_y + net_launch_y,
                                                - self.anti_drone_current_z + net_launch_z])

        heading_btw_drones = self.heading(anti_drone_position_vector)

        # displacement between drone and anti_drone

        self.displacement = math.sqrt(pow((self.anti_drone_current_x - net_launch_x),2) +
                                        pow((self.anti_drone_current_y - net_launch_y),2)
                                        + pow((self.anti_drone_current_z - net_launch_z),2))

        # print(self.displacement, heading_error)

        a = self.max_vel
        b = v_j

        V = ((a-b)/2 * math.tanh((self.displacement - 25)/25) + (a+b)/2) * heading_btw_drones

        # drone is stationary

        if (b < 1 and self.displacement < 8.0 and yaw_error < 5 and yaw_error > -5):

            V = ((a-b)/2 * math.tanh((self.displacement - 100)/25) + (a+b)/2) * heading_btw_drones

            v_i_x = V[0]
            v_i_y = V[1]
            v_i_z = V[2]

            self.array.append(self.displacement)

            print(len(self.array))

            if (len(self.array) > 100):

                print("DEAD")

                v_i_x = 0.0
                v_i_y = 0.0
                v_i_z = 0.0

                self.throw_net = True

                self.publisher(v_i_x, v_i_y, v_i_z, drone_yaw_rate, net_launch_x,
                                        net_launch_y, net_launch_z, self.throw_net)

                rospy.sleep(1)

                rospy.signal_shutdown("mission completed")

        # drone is moving and is in range of anti-drone 

        elif (self.displacement < 10.0 and heading_error < 60 and heading_error > -60):

            self.array.append(self.displacement)

            v_i_x = V[0]
            v_i_y = V[1]
            v_i_z = V[2]

            print(len(self.array))

            if (len(self.array) > 100):

                print("DEAD")

                v_i_x = 0.0
                v_i_y = 0.0
                v_i_z = 0.0

                self.throw_net = True

                self.publisher(v_i_x, v_i_y, v_i_z, drone_yaw_rate, net_launch_x,
                                        net_launch_y, net_launch_z, self.throw_net)

                rospy.sleep(1)

                rospy.signal_shutdown("mission completed")

        # drone is moving and out of range of anti-drone

        else:

            self.array = []

            v_i_x = V[0]
            v_i_y = V[1]
            v_i_z = V[2]

        self.publisher(v_i_x, v_i_y, v_i_z, drone_yaw_rate, net_launch_x,
                                        net_launch_y, net_launch_z, self.throw_net)

    def heading(self, vector):

        # returns a unit vector

        vector_magnitude = self.magnitude(vector)

        return vector / vector_magnitude

    def drone_path(self, drone_vel_vector):

        vel_unit_vector = drone_vel_vector / self.magnitude(drone_vel_vector)

        vel_unit_sum = vel_unit_vector[0] + vel_unit_vector[1] + vel_unit_vector[2]

        vel_change_diff = vel_unit_sum - self.prev_vel_unit_sum

        self.prev_vel_unit_sum = vel_unit_sum

        return vel_change_diff

    def auto_rtl(self):

        v_i_x = 0.0
        v_i_y = 0.0
        v_i_z = 0.0

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y
        vel_msg.twist.linear.z = v_i_z

        self.vel_publisher.publish(vel_msg)

        print("THROW NET")

        rospy.sleep(1)

        # service request for Return to Home

        # offb_set_mode = SetModeRequest()
        # offb_set_mode.custom_mode = 'AUTO.RTL'

        # if (self.current_state.mode != "AUTO.RTL"):

        #     if (self.set_mode_client.call(offb_set_mode).mode_sent == True):

        #         rospy.loginfo("RTL enabled")
        #         rospy.signal_shutdown("impact")

        rospy.signal_shutdown("impact")

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def magnitude(self, vector):

        return math.sqrt(sum(pow(element, 2) for element in vector))

    def publisher(self, v_i_x, v_i_y, v_i_z, v_i_yaw, net_launch_x, net_launch_y,
                                                            net_launch_z, throw_net):

        # limit velocities in all directions

        # if (v_i_x > 4.0):

        #     v_i_x = 3.0
        #     print("limiting vel")

        # if (v_i_x < -4.0):

        #     v_i_x = -3.0
        #     print("limiting vel")

        # if (v_i_y > 4.0):

        #     v_i_y = 3.0
        #     print("limiting vel")

        # if (v_i_y < -4.0):

        #     v_i_y = -3.0
        #     print("limiting vel")

        # if (v_i_z > 4.0):

        #     v_i_z = 3.0
        #     print("limiting vel")

        # if (v_i_z < -4.0):

        #     v_i_z = -3.0
        #     print("limiting vel")

        # publish velocity message

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y
        vel_msg.twist.linear.z = v_i_z

        vel_msg.twist.angular.z = v_i_yaw

        custom_msg = custom_msgs()

        custom_msg.net_launch_x = net_launch_x
        custom_msg.net_launch_y = net_launch_y
        custom_msg.net_launch_z = net_launch_z
        custom_msg.throw_net = throw_net

        self.vel_publisher.publish(vel_msg)

        self.custom_topic.publish(custom_msg)

        if (throw_net == True):

            rc_msg = OverrideRCIn()

            rc_msg.channels = [65535, 65535, 65535, 65535, 65535, 65535, 65535, 2000, 65535, 
                                65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]

            self.rc_publisher.publish(rc_msg)            

    def drone_position_callback(self, data):

        self.drone_current_x = data.pose.position.x
        self.drone_current_y = data.pose.position.y
        self.drone_current_z = data.pose.position.z

    def anti_drone_position_callback(self, data):

        self.anti_drone_current_x = data.pose.position.x
        self.anti_drone_current_y = data.pose.position.y
        self.anti_drone_current_z = data.pose.position.z

        self.anti_drone_quat_x = data.pose.orientation.x
        self.anti_drone_quat_y = data.pose.orientation.y
        self.anti_drone_quat_z = data.pose.orientation.z
        self.anti_drone_quat_w = data.pose.orientation.w

        self.anti_drone_quaternion = np.array([self.anti_drone_quat_x,
            self.anti_drone_quat_y, self.anti_drone_quat_z, self.anti_drone_quat_w])

    def drone_velocity_callback(self, data):

        self.drone_current_vel_x = data.twist.linear.x
        self.drone_current_vel_y = data.twist.linear.y
        self.drone_current_vel_z = data.twist.linear.z

    def anti_drone_velocity_callback(self, data):

        self.anti_drone_current_vel_x = data.twist.linear.x
        self.anti_drone_current_vel_y = data.twist.linear.y
        self.anti_drone_current_vel_z = data.twist.linear.z

if __name__ == "__main__":

    try:

        rospy.init_node("anti_drone_method_2")

        # initialize trajectory object

        trajectory_mode = trajectory()

        # initialize main control object and call control function

        drone_controller = controller()
        drone_controller.control()

    except rospy.ROSInterruptException:

        print("Total time = ", time)
        pass

