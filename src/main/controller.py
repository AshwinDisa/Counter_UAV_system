#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import StatusText, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest


class pose():

    def __init__(self):
    
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        self.statustext_publisher = rospy.Publisher("/mavros/statustext/send", 
        						StatusText, queue_size = 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
    def pose_callback(self, pose_data):
    
        self.current_x = pose_data.pose.position.x
        self.current_y = pose_data.pose.position.y
        self.current_z = pose_data.pose.position.z      
    
    def control(self):
    
        current_x = round(self.current_x, 2)
        current_y = round(self.current_y, 2)
        current_z = round(self.current_z, 2)

        msg = "001: x = ", current_x, "y = ", current_y, "z = ", current_z
        return str(msg)
            
class arm():

    def __init__(self):
    
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    def control(self):
    
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
                
        if (self.arming_client.call(arm_cmd).success == True):
                    
            rospy.loginfo("Vehicle armed")         
            return "001: armed"
            
        else:
        
            rospy.loginfo("Vehicle not armed")
            return "001: not armed"   
            
class waypoint():

    def __init__(self):

        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.pose_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 10)
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.statustext_publisher = rospy.Publisher("/mavros/statustext/send", 
        						StatusText, queue_size = 10)
        						
        self.armer = arm()
        
    def state_callback(self, state_msg):
    
        self.state_msg = state_msg
    
    def control(self, waypoint_array):
    
        rate = rospy.Rate(20)
    
        pose_msg = PoseStamped()
        
        pose_msg.pose.position.x = waypoint_array[0]
        pose_msg.pose.position.y = waypoint_array[1]
        pose_msg.pose.position.z = waypoint_array[2]

        # set_mode_guided = SetModeRequest()
        # set_mode_guided.custom_mode = 'GUIDED'
        
        # if (self.state_msg.mode == "GUIDED"):
        
            # rospy.loginfo("Guided mode already enabled")
            # return "001: Guided mode already enabled"
                          
        # else:
        
        #    if (self.set_mode_client.call(set_mode_guided).mode_sent == True):
            
        #        rospy.loginfo("Guided mode enabled")
                # return "001: Guided mode enabled"
                
        while (self.state_msg.armed == False and not rospy.is_shutdown()):
            
            msg = self.armer.control()
            rate.sleep()
            return msg
        
        self.pose_publisher.publish(pose_msg)
        
        return "001: publishing"           
        
class takeoff():

    def __init__(self):

        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.statustext_publisher = rospy.Publisher("/mavros/statustext/send", 
        						StatusText, queue_size = 10)
        						
        self.armer = arm()
        self.takeoff_flag = 0
        
    def state_callback(self, state_msg):
    
        self.state_msg = state_msg
    
    def control(self, altitude):
    
        rate = rospy.Rate(20)

        # set_mode_guided = SetModeRequest()
        # set_mode_guided.custom_mode = 'GUIDED'
        
        # if (self.state_msg.mode == "GUIDED"):
        
            # rospy.loginfo("Guided mode already enabled")
            # return "001: Guided mode already enabled"
                          
        # else:
        
        #    if (self.set_mode_client.call(set_mode_guided).mode_sent == True):
            
        #        rospy.loginfo("Guided mode enabled")
                # return "001: Guided mode enabled"
                
        while (self.state_msg.armed == False and not rospy.is_shutdown()):
            
            msg = self.armer.control()
            rate.sleep()
            # return msg
            
        rospy.sleep(1)       
            
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.min_pitch = 0
        takeoff_cmd.yaw = 0
        takeoff_cmd.latitude = 0
        takeoff_cmd.longitude = 0
        takeoff_cmd.altitude = altitude[0]
        
        self.takeoff_client.call(takeoff_cmd) 
        
        if (self.takeoff_client.call(takeoff_cmd).success == True):
            
            return "001: takeoff detected" 
        
        else:
        
            return "001: takeoff failed"
                              
class follow():

    def __init__(self):

        self.follow = "following"
        
    def control(self):
    
        # rospy.loginfo(self.follow)
        return "001: following"
        
class kill():

    def __init__(self):

        self.kill = "killing"
        
    def control(self):
    
        # rospy.loginfo(self.kill)
        return "001: killing"
        
class retrieve():

    def __init__(self):

        self.retrieve = "retrieving"
        
    def control(self):
    
        # rospy.loginfo(self.retrieve)
        return "001: retrieving"
        
class abort():

    def __init__(self):

        self.abort = "Aborting"
        
    def control(self):
    
        # rospy.loginfo(self.abort)   
        return "001: aborting"         
            
                 
                
if __name__ == "__main__":

    try:
        
        rospy.init_node("controller_node")
        
    except rospy.ROSInterruptException:

        pass




