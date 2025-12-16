#!/usr/bin/env python3
import socket
import json
import rospy
from time import sleep
from std_msgs.msg import String
from dvl_a50_ros_driver.msg import DVL
from dvl_a50_ros_driver.msg import DVLBeam
from dvl_a50_ros_driver.msg import DVLEstimate
import select

def connect():
	global s, TCP_IP, TCP_PORT
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((TCP_IP, TCP_PORT))
		s.settimeout(1)

		# # Reset dead-reckoning estimate on connection
		# reset_cmd = '{"command":"reset_dead_reckoning"}'
		# s.sendall(reset_cmd.encode("utf-8"))
		# rospy.loginfo("Connected to DVL at and sent reset_dead_reckoning command.")
	except socket.error as err:
		rospy.logerr("No route to host, DVL might be booting? {}".format(err))
		sleep(1)
		connect()

oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()
estimate = DVLEstimate()

def getData():
	global oldJson, s
	raw_data = ""

	while not '\n' in raw_data:
		try:
			rec = s.recv(1) # Add timeout for that
			if len(rec) == 0:
				rospy.logerr("Socket closed by the DVL, reopening")
				connect()
				continue
		except socket.timeout as err:
			rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
			connect()
			continue
		raw_data = raw_data + str(rec.decode())
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('\n')
	oldJson = raw_data[1]
	raw_data = raw_data[0]
	return raw_data

def configure():
	"""
	Sends configuration commands to the DVL after connection.
	"""
	global s
	config = {
		"speed_of_sound": rospy.get_param("~speed_of_sound", 1480),
		"acoustic_enabled": rospy.get_param("~acoustic_enabled", True),
		"dark_mode_enabled": rospy.get_param("~dark_mode_enabled", False),
		"mounting_rotation_offset": rospy.get_param("~mounting_rotation_offset", 0),
		"range_mode": rospy.get_param("~range_mode", "auto"),
		"periodic_cycling_enabled": rospy.get_param("~periodic_cycling_enabled", True)
	}
	command_str = json.dumps({"command":"set_config", "parameters":config})

    try:
        rospy.loginfo("Sending configuration to DVL: {}".format(command_str))
        if not command_str.endswith('\n'):
            command_str = command_str + '\n'
        s.sendall(command_str.encode("utf-8"))
        
        # --- NEW CODE: Wait for response ---
        # We loop briefly to read the socket until we get the response
        # This handles cases where a velocity packet might arrive before the response
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0: # 2 second timeout
            raw_resp = getData() # Reuse your existing getData function!
            try:
                data = json.loads(raw_resp)
                if data.get("type") == "response":
                    if data.get("success"):
                        rospy.loginfo("Configuration Applied Successfully.")
                        return # Exit function, success!
                    else:
                        rospy.logerr(f"Configuration Failed: {data.get('message')}")
                        return
                # If we get velocity data here, we just ignore it or log it, 
                # but we keep looping to find the response.
            except ValueError:
                pass
        
        rospy.logwarn("Timed out waiting for configuration response.")
        # -----------------------------------

    except socket.error as err:
        rospy.logerr("Failed to send configuration to DVL: {}".format(err))

def command_callback(msg):
	"""
	Listens to /dvl/send_command topic.
	Expects a JSON string. 
	Available commands:
	- '{"command":"reset_dead_reckoning"}'
	- '{"command":"calibrate_gyro"}' (only when platform is stationary)
	- '{"command":"trigger_ping"}'
	- '{"command":"get_config"}'
	- '{"command":"set_config", "parameters":{
			"speed_of_sound":1480,
			"acoustic_enabled":true,
			"dark_mode_enabled":false,
			"mounting_rotation_offset":0,
			"range_mode":"auto",
			"periodic_cycling_enabled":true}}'
	"""
	global s
	command_str = msg.data

	# Check JSON validity
	try:
		json_check = json.loads(command_str)
		if "command" not in json_check:
			rospy.logerr("DVL command JSON must contain a 'command' field.")
			return
	except ValueError as e:
		rospy.logerr("Invalid JSON command received. Not sending to DVL.")
		return
	
	try:
		rospy.loginfo("Sending command to DVL: {}".format(command_str))
		if not command_str.endswith('\n'):
			command_str = command_str + '\n'
		s.sendall(command_str.encode("utf-8"))
	except socket.error as err:
		rospy.logerr("Failed to send command to DVL: {}".format(err))


def publisher():
	pub_raw = rospy.Publisher('dvl/json_data', String, queue_size=10)
	pub = rospy.Publisher('dvl/data', DVL, queue_size=10)
	pub_estimate = rospy.Publisher('/dvl/estimate', DVLEstimate, queue_size=10)

	rospy.Subscriber('dvl/send_command', String, command_callback)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
		data = json.loads(raw_data)

		# edit: the logic in the original version can't actually publish the raw data
		# we slightly change the if else statement so now
		# do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
		# do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

		if do_log_raw_data:
			rospy.loginfo(raw_data)
			pub_raw.publish(raw_data)

		# Get message type
		msg_type = data.get("type", "unknown")

		if msg_type == "velocity":
			theDVL.header.stamp = rospy.Time.now()
			theDVL.header.frame_id = "dvl_link"
			theDVL.time = data["time"]
			theDVL.velocity.x = data["vx"]
			theDVL.velocity.y = data["vy"]
			theDVL.velocity.z = data["vz"]
			theDVL.fom = data["fom"]
			theDVL.altitude = data["altitude"]
			theDVL.velocity_valid = data["velocity_valid"]
			theDVL.status = data["status"]
			theDVL.form = data["format"]

			beam0.id = data["transducers"][0]["id"]
			beam0.velocity = data["transducers"][0]["velocity"]
			beam0.distance = data["transducers"][0]["distance"]
			beam0.rssi = data["transducers"][0]["rssi"]
			beam0.nsd = data["transducers"][0]["nsd"]
			beam0.valid = data["transducers"][0]["beam_valid"]

			beam1.id = data["transducers"][1]["id"]
			beam1.velocity = data["transducers"][1]["velocity"]
			beam1.distance = data["transducers"][1]["distance"]
			beam1.rssi = data["transducers"][1]["rssi"]
			beam1.nsd = data["transducers"][1]["nsd"]
			beam1.valid = data["transducers"][1]["beam_valid"]

			beam2.id = data["transducers"][2]["id"]
			beam2.velocity = data["transducers"][2]["velocity"]
			beam2.distance = data["transducers"][2]["distance"]
			beam2.rssi = data["transducers"][2]["rssi"]
			beam2.nsd = data["transducers"][2]["nsd"]
			beam2.valid = data["transducers"][2]["beam_valid"]

			beam3.id = data["transducers"][3]["id"]
			beam3.velocity = data["transducers"][3]["velocity"]
			beam3.distance = data["transducers"][3]["distance"]
			beam3.rssi = data["transducers"][3]["rssi"]
			beam3.nsd = data["transducers"][3]["nsd"]
			beam3.valid = data["transducers"][3]["beam_valid"]

			theDVL.beams = [beam0, beam1, beam2, beam3]

			pub.publish(theDVL)
		
		elif msg_type == "position_local":
			estimate.header.stamp = rospy.Time.now()
			estimate.header.frame_id = "dvl_link"
			estimate.ts = data["ts"]
			estimate.x = data["x"]
			estimate.y = data["y"]
			estimate.z = data["z"]
			estimate.std = data["std"]
			estimate.roll = data["roll"]
			estimate.pitch = data["pitch"]
			estimate.yaw = data["yaw"]
			estimate.status = data["status"]

			pub_estimate.publish(estimate)
			# print(f"Dead-reckoning euclidean distance {(estimate.x**2 + estimate.y**2 + estimate.z**2)**0.5:.2f} m")

		elif msg_type == "response":
			# This line is prints to terminal and publishes to /rosout
			success = data.get("success", False)
			message = data.get("message", "No message")
			if success:
				rospy.loginfo("DVL COMMAND SUCCESS: {}".format(message))
			else:
				rospy.logerr("DVL COMMAND FAILED: {}".format(message))

		elif msg_type == "error":
			rospy.logerr("DVL ERROR: {}".format(data.get("message", "")))

		else:
			rospy.logwarn("Unknown DVL message type: {}".format(msg_type))

		rate.sleep()

if __name__ == '__main__':
	global s, TCP_IP, TCP_PORT, do_log_raw_data
	rospy.init_node('a50_pub', anonymous=False)
	TCP_IP = rospy.get_param("~ip", "10.42.0.186")
	TCP_PORT = rospy.get_param("~port", 16171)
	do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
	connect()
	configure()
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()
