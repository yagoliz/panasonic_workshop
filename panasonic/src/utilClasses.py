#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
import numpy as np

class Humidity():

  def __init__(self):
    self.pub = rospy.Publisher("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd = UInt16()
    self.min_humidity = 35.0
    self.max_humidity = 55.0
    self.max_assist_rate = 500
    self.min_assist_rate = 100

  def main(self, humidity):
    if humidity < self.min_humidity:
      humidity = self.min_humidity

    if humidity > self.max_humidity:
      humidity = self.max_humidity

    self.cmd.data = self.extrapolate(humidity)
    self.pub.publish(self.cmd)

  def extrapolate(self, humidity):
    res = self.max_assist_rate - (self.max_assist_rate-self.min_assist_rate)*(humidity-self.min_humidity)/(self.max_humidity-self.min_humidity)
    return res

class Accelerometer():

  def __init__(self):
    self.pub = rospy.Publisher("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd = Uint16()
    self.prev_accels = np.zeros(5,3)
    self.count = 0

  def main(self, accel_vector):

    if self.count < 5:
      self.prev_accels[count] = accel_vector
      count += 1

    else:
      self.prev_accels[:4] = self.prev_accels[1:5]
      self.prev_accels[4] = accel_vector

      self.evaluate_accels()

      self.pub.publish(self.cmd)

  def evaluate_accels():
    

class PedestrianTensorFlow():

  def __init__(self):
    self.sub = rospy.Subscriber("/number_pedestrians", UInt8, self.callback)
    self.pub = rospy.Publisher ("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd = UInt16()
    self.people_count = 0
    print("Values initialized")

  def callback(self, msg):
    self.people_count = msg.data

  def main(self, data):

    if self.people_count < 2:
      self.cmd.data = 500

    elif self.people_count >= 2 and self.people_count < 5:
      self.cmd.data = 250

    else:
      self.cmd.data = 100

    self.pub.publish(self.cmd)


if __name__=="__main__":

  rospy.init_node("fake_node")
  hum = Humidity()
  acc = Accelerometer()
  ped = PedestrianTensorFlow()

  rospy.spin()
