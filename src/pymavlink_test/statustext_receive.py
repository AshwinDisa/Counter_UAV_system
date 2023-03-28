#! /usr/bin/env python3

from pymavlink import mavutil

receiver = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

receiver.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (
						receiver.target_system, receiver.target_component))

while True:

	msg = receiver.recv_msg()
	if msg and msg.get_type() == "STATUSTEXT":
		text = msg.text
		if text.startswith("Anti-Drone command: "):
		# if text.startswith(""):
			print(text)
			if text.startswith("Anti-Drone command: Arm"):
				receiver.mav.command_long_send(receiver.target_system, 
					receiver.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
						0, 1, 0, 0, 0, 0, 0, 0)