#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
import numpy as np

class Humidity():

  def __init__(self):
    self.pub_light = rospy.Publisher("/number_pedestrians", UInt8, queue_size=5)
    self.pub = rospy.Publisher("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd_light = UInt8()
    self.cmd = UInt16()
    self.min_humidity = 35.0
    self.max_humidity = 55.0
    self.max_assist_rate = 500
    self.min_assist_rate = 100
    self.prev = 200

    self.cmd_light.data = 0
    self.pub_light.publish(self.cmd_light)

  def main(self, humidity):
    if humidity < self.min_humidity:
      humidity = self.min_humidity

    if humidity > self.max_humidity:
      humidity = self.max_humidity

    self.cmd.data = self.extrapolate(humidity)

    if self.cmd.data != self.prev:
      self.pub.publish(self.cmd)
      self.pub_light.publish(self.cmd_light)
      self.prev = self.cmd.data

  def extrapolate(self, humidity):
    res = self.max_assist_rate - (self.max_assist_rate-self.min_assist_rate)*(humidity-self.min_humidity)/(self.max_humidity-self.min_humidity)

    if res > 400:
      res = 500
      self.cmd_light.data = 0
    elif res < 200:
      res = 200
      self.cmd_light.data = 2
    elif res >= 200 and res <= 400:
      res = 100
      self.cmd_light.data = 4
    else:
      res = 200
      self.cmd_light.data = 2

    return res

class Accelerometer():

  def __init__(self):
    self.pub_light = rospy.Publisher("/number_pedestrians", UInt8, queue_size=5)
    self.pub = rospy.Publisher("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd_light = UInt8()
    self.cmd = UInt16()
    self.prev_accels = np.zeros((5,3))
    self.count = 0
    self.prev = 200
    self.max_diff = np.array([3.0, 3.0, 7.0])

    self.cmd_light.data = 0
    self.pub_light.publish(self.cmd_light)

  def main(self, accel_vector):

    if self.count < 5:
      self.prev_accels[self.count] = accel_vector
      self.count += 1

    else:
      self.prev_accels[:4] = self.prev_accels[1:5]
      self.prev_accels[4] = accel_vector

      self.evaluate_accels()

      if self.cmd.data != self.prev:
        self.pub.publish(self.cmd)
        self.pub_light.publish(self.cmd_light)
        self.prev = self.cmd.data

  def evaluate_accels(self):
    max_acc = np.amax(self.prev_accels, axis=0)
    min_acc = np.amin(self.prev_accels, axis=0)

    diff_accels = max_acc - min_acc

    # Bumpy road
    if diff_accels[0] > self.max_diff[0] or diff_accels[1] > self.max_diff[1] or diff_accels[2] > self.max_diff[2]:
      self.cmd.data = 500
      self.cmd_light.data = 4

    # Non-bumpy road
    else:
      avg_accel = np.average(self.prev_accels, axis=0)

      if avg_accel[2] < 8.0:
        if avg_accel[1] > 2.0:
          self.cmd.data = 500
          self.cmd_light.data = 4
        elif avg_accel[1] < -2.0:
          self.cmd.data = 100
          self.cmd_light.data = 1
        else:
          self.cmd.data = 200
          self.cmd_light.data = 1

      else:
        self.cmd.data = 200
        self.cmd_light.data = 1

class PedestrianTensorFlow():

  def __init__(self):
    self.sub = rospy.Subscriber("/number_pedestrians", UInt8, self.callback)
    self.pub = rospy.Publisher ("/hackbike/command/target_assist_rate", UInt16, queue_size=1)
    self.cmd = UInt16()
    self.people_count = 0
    self.prev = 200
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

    if self.cmd.data != self.prev:
      self.pub.publish(self.cmd)
      self.prev = self.cmd.data


if __name__=="__main__":

  rospy.init_node("fake_node")
  hum = Humidity()
  acc = Accelerometer()
  ped = PedestrianTensorFlow()

  rospy.spin()
