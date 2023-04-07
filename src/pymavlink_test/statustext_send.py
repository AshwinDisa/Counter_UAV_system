#! /usr/bin/env python3

import time
from pymavlink import mavutil

# Connect to FCU over serial port
sender = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for the heartbeat message to confirm the connection
sender.wait_heartbeat()

class main():

    def __init__(self):

        self.arm_flag = 0
        self.takeoff_flag = 0

    def number_to_string(self, argument):

        if (argument == '0'):
            text = "001: Follow".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
            
        elif (argument == '1'):
            text = "001: Kill".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

        elif (argument == '2'):
            text = "001: RTL".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

        elif (argument == '3'):
            text = "001: Abort".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

        elif (argument == '4'):

            if (self.takeoff_flag == 0):

                takeoff_alt = input("Enter Altitude: ") 

                takeoff_alt = '%02d' % int(takeoff_alt)
                
                tup = ('001: Takeoff', ' ,', takeoff_alt)
                text = ''.join(tup).encode('utf-8')
                sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
                self.takeoff_flag = 1

                time.sleep(1)
                text = "001: idle".encode('utf-8')
                sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        
            else:
                text = "001: idle".encode('utf-8')
                sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)


        elif (argument == '6'):

            waypoint_x = input("Enter waypoint x: ") 
            waypoint_y = input("Enter waypoint y: ") 
            waypoint_z = input("Enter waypoint z: ") 

            waypoint_x = '%02d' % int(waypoint_x)
            waypoint_y = '%02d' % int(waypoint_y)
            waypoint_z = '%02d' % int(waypoint_z)
                
            tup = ('001: Waypoint', ' ,', waypoint_x, ',', waypoint_y, ',', waypoint_z)
            text = ''.join(tup).encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

        elif (argument == '5'):

            text = "001: Pose".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

        elif (argument == '69'):

            if (self.arm_flag == 0):
                
                text = "001: Arm".encode('utf-8')
                sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
                self.arm_flag = 1

                time.sleep(2)
                text = "001: idle".encode('utf-8')
                sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        
        else:
            text = "001: idle".encode('utf-8')
            sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)


        print("----------------------")

        flag = 0
        while (flag == 0):
            
            # msg = sender.recv_msg()
            msg = sender.recv_match(type='STATUSTEXT', blocking=True)
            if (msg and msg.get_type() == "STATUSTEXT" and msg.text.startswith('001') or msg.text.startswith("('001")):

                text = msg.text
                flag = 0
                print("----------------------")
                print("Message Received", text)
                print("----------------------")

            time.sleep(0.1)

if __name__ == "__main__":

    start_flag = 0
    while (start_flag == 0):

        # msg = sender.recv_msg()
        msg = sender.recv_match(type='STATUSTEXT', blocking=True)
        if (msg and msg.get_type() == "STATUSTEXT" and msg.text.startswith('001') or msg.text.startswith("('001")):
        #if (msg and msg.get_type() == "STATUSTEXT"):
    
            print(msg.text)

            start_flag = 1

        time.sleep(0.1)

    main_obj = main()

    argument = input("----------------------\nPress 0 --- Follow\nPress 1 --- Kill\nPress 2 --- RTL\nPress 3 --- Abort\nPress 4 --- Takeoff\nPress 5 --- Current pose\nPress 6 --- Waypoint\nPress 69 --- Arm\nInput: ")
    main_obj.number_to_string(argument)