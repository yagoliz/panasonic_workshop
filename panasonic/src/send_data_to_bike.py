#!/usr/bin/env python

# Regular libraries
import sys

# Regular Python libraries
import rospy
import time
# ROS libraries
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import serial
import utilClasses

# TerMITes data flow
# [X, Y, X]: [0, 1, 2]
# Temperature: 3
# Light: 4
# Humidity: 5
# Proximity: 6
# Pressure: 7
# Altitude: 8
# DewPoint: 9


# Import rospy and start publishers
rospy.init_node("bike_data")
mode = rospy.get_param("mode", "humidity")

# Change the mode of the e-bike
pub_mode = rospy.Publisher("/hackbike/command/set_mode", UInt8, queue_size=5)
state = UInt8()
state.data = 1

pub_mode.publish(state)

# Start data collecting through serial port
# Open serial port
ser = serial.Serial('/dev/termite', 115200)
# Send commands to terMITe to ouput data in CSV mode
# Stop sending data
ser.write(b'CMD')
time.sleep(2.0)
# Change mode to CSV
ser.write(b'CSV')
time.sleep(1.0)
# Start sending data
ser.write(b'EXT')
time.sleep(1.0)

# Start classes depending on chosen mode
if mode == "humidity":
  obj_class = utilClasses.Humidity()

elif mode == "accelerometer":
  obj_class = utilClasses.Accelerometer()

elif mode == "pedestrian":
  obj_class = utilClasses.PedestrianTensorFlow()

else:
  rospy.logerr("Invalid mode")
  sys.exit()

# Start main loop
ctrl_c = True
def shutdownhook():
  ctrl_c = False

rospy.on_shutdown(shutdownhook)

while ctrl_c:
  # Read data from serial
  data = ser.readline()
  data = data.split(',')

  if len(data) < 10:
    pass

  else:
    if mode == "humidity":
      input_data = float(data[5])

    elif mode == "accelerometer":
      # Vector with 3 components of acceleration
      input_data = [0, 0, 0]
      input_data[0] = float(data[0])
      input_data[1] = float(data[1])
      input_data[2] = float(data[2])

      print(input_data)

    elif mode == "pedestrian":
      input_data = None

    obj_class.main(input_data)

# Close the serial port
ser.close()
