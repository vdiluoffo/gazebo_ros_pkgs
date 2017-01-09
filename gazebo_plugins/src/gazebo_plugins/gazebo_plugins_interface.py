#! /usr/bin/env python
# Wrappers around the services provided by gazebo_ros_factory plugin


####            ROS1 Code                       ####
#import roslib; roslib.load_manifest('gazebo_plugins')


#import rospy

import sys
from time import sleep

import rclpy

from std_msgs.msg import String

from gazebo_plugins.msg import GazeboModel
from gazebo_plugins.srv import SpawnModel
from gazebo_plugins.srv import DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def load_model(model_msg):
    print "waiting for service spawn_model"
#    rospy.wait_for_service('spawn_model') Not available in rclpy.c ( // TODO(jacquelinekay) Services.)
#    rclpy.wait_for_service('spawn_model')
    try:
#        spawn_model= rospy.ServiceProxy('spawn_model', SpawnModel)

#        spawn_model= rclpy.ServiceProxy('spawn_model', SpawnModel) Mot available in rclpy.c rclpy.ServiceProxy
        resp1 = spawn_model(model_msg)
        return resp1.success
#    except rospy.ServiceException, e:
    except rclpy.ServiceException, e:
        print "Service call failed: %s"%e




