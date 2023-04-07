#! /usr/bin/env python3

import rospy
import os
import time
from controller import *
from mavros_msgs.msg import StatusText, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, StreamRate, StreamRateRequest

time.sleep(0.5)
os.system("gnome-terminal -e 'bash -c \"roslaunch mavros apm.launch && exit; exec bash\" '")
time.sleep(6)
os.system("gnome-terminal -e 'bash -c \"rosrun mavros mavsys rate --all 10 && exit; exec bash\" '")
class main():

    def __init__(self, follower, killer, retriever, aborter, takeoffer, armer, poser):

        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.statustext_callback)
        rospy.Subscriber("/mavros/state", StatusText, self.state_callback)
        
        self.statustext_publisher = rospy.Publisher("/mavros/statustext/send", 
        						StatusText, queue_size = 10)    						
        self.stream_client = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        
        self.text = 'nil'

    def statustext_callback(self, statustext_msg):
    
        text = statustext_msg.text
        if (text.startswith("001")):
            self.text = text       

    def state_callback(self, state_msg):

        self.state = state_msg

    def main(self):
    
        rate = rospy.Rate(20)
                
        rospy.loginfo("waiting for MAVLink message")
        
        self.last_msg_time = rospy.Time.now()
        
        streamrate = StreamRateRequest()
        streamrate.stream_id = 0
        streamrate.message_rate = 10
        streamrate.on_off = True
        
    
        while (not rospy.is_shutdown()):
        
            if (self.text.startswith("001: Follow")):
                
                msg = follower.control()

            elif (self.text.startswith("001: Kill")):
                
                msg = killer.control()
                
            elif (self.text.startswith("001: RTL")):
                
                msg = retriever.control()
                
            elif (self.text.startswith("001: Abort")):
                
                msg = aborter.control()
                
            elif (self.text.startswith("001: Arm")):
            
                msg = armer.control()
                
            elif (self.text.startswith("001: Takeoff")):
                
                altitude = [float(self.text.split(',')[1])]
                msg = takeoffer.control(altitude)
                
            elif (self.text.startswith("001: Waypoint")):
                
                waypoint_array = [int(self.text.split(',')[1]), int(self.text.split(',')[2]), int(self.text.split(',')[3])]
                msg = takeoffer.control(waypoint_array)
                
            elif (self.text.startswith("001: Pose")):
                
                msg = poser.control()
                
            else:
            
                msg = "001: idle"
                
            if ((rospy.Time.now() - self.last_msg_time) > rospy.Duration(0.5)):
            
                statustext_msg = StatusText()

                statustext_msg.text = msg
                statustext_msg.severity = 6
                self.statustext_publisher.publish(statustext_msg)
            	
                self.last_msg_time = rospy.Time.now()
                        
            rate.sleep()        
    
if __name__ == "__main__":

    try:
        
        rospy.init_node("main_node")
        
        follower = follow()
        killer = kill()
        retriever = retrieve()
        aborter = abort()
        armer = arm()
        takeoffer = takeoff()
        waypointer = waypoint()
        poser = pose()
        
        main_obj = main(follower, killer, retriever, aborter, takeoffer, armer, poser)
        main_obj.main()

    except rospy.ROSInterruptException:

        pass




