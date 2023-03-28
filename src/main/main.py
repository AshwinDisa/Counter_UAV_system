#! /usr/bin/env python3

import rospy
from pymavlink import mavutil
from anti_drone.msg import command_msgs

receiver = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

receiver.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (receiver.target_system, 
                                                        receiver.target_component))

command_topic = rospy.Publisher('/command_topic', command_msgs,
                                                                queue_size = 10)                                                        

def main():

    rate = rospy.rate(5)

    while True:

        msg = receiver.recv_msg()
        command_msg = command_msgs()

        if msg and msg.get_type() == "STATUSTEXT":

            text = msg.text

            if text.startswith("Anti-Drone command: "):
            # if text.startswith(""):

                print(text)

                if text.startswith("Anti-Drone command: Follow"):

                    command_msg.command = '0'
                    command_topic.publish(command_msg)

        rate.sleep(5)      

if __name__ == "__main__":

    try:

        rospy.init_node("main_node")
        main()

    except rospy.ROSInterruptException:

        pass




