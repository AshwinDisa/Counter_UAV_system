#! /usr/bin/env python3

from pymavlink import mavutil

# Connect to FCU over serial port
sender = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for the heartbeat message to confirm the connection
sender.wait_heartbeat()

def number_to_string(argument):

    if (argument == '0'):
        text = "Anti-Drone command: Follow".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        
    elif (argument == '1'):
        text = "Anti-Drone command: Follow and Inspect".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
        
    elif (argument == '2'):
        text = "Anti-Drone command: Kill".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

    elif (argument == '3'):
        text = "Anti-Drone command: RTL".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)

    elif (argument == '69'):
        text = "Anti-Drone command: Arm".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
    
    else:
        text = "Anti-Drone command: Invalid Input".encode('utf-8')
        sender.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text)
            
if __name__ == "__main__":

    argument = input(" Press 0 -- Follow\n Press 1 -- Follow and Inspect\n Press 2 -- Kill\n Press 3 -- RTL\n Press 69 -- Arm\n Input: ")
    number_to_string(argument)

