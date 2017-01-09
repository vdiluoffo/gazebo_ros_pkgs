#!/usr/bin/env python
import rclpy
from sensor_msgs.msg import Range

import unittest

class TestRangePlugin(unittest.TestCase):

  def test_max_range(self):
#    msg = rospy.wait_for_message('/sonar2', Range)
     msg = rclpy_wait('/sonar2', Range)
    self.assertAlmostEqual(msg.range, msg.max_range)

  def test_inside_range(self):
#    msg = rospy.wait_for_message('/sonar', Range)
     msg = rclpy_wait('/sonar', Range)
    self.assertTrue(msg.range < 0.25 and msg.range > 0.22)

if __name__ == '__main__':
#  import rostest

  PKG_NAME = 'gazebo_plugins'
  TEST_NAME = PKG_NAME + 'range_test'
#  rospy.init_node(TEST_NAME)
#  rostest.rosrun(PKG_NAME, TEST_NAME, TestRangePlugin) # It seems that gtest should be used here with UnitTest
    rclpy_init(TEST_NAME)
