#!/usr/bin/env python

import math
import numpy
import carla
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel


def carla_location_to_numpy_vector(carla_location):
    """
    as it is a right-handed coordinate system
    
    Convert a carla location to numpy vector.

    Considers the conversion from left-handed system (unreal) to right-handed system (ROS).

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """

    return numpy.array([
        carla_location.x,
        -carla_location.y,
        carla_location.z,
    ])

def RPY2quaternion(roll, pitch, yaw):
    """
    transform eluer to quaternion
    parameter:
        roll
        pitch
        yaw
    return (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

def ros_pose_to_carla_transform(ros_pose):
    """
    Convert a ROS pose a carla transform.
    """

    return carla.Transform(
        ros_point_to_carla_location(ros_pose.position),
        ros_quaternion_to_carla_rotation(ros_pose.orientation),
    )


def ros_point_to_carla_location(ros_point):
    ##########for tracker
    return carla.Location(ros_point.x, -ros_point.z, ros_point.y)
    #########################################################################################
    #return carla.Location(ros_point.x, -ros_point.y, ros_point.z)

def ros_quaternion_to_carla_rotation(ros_quaternion):
    roll, pitch, yaw = quat2euler([
        ros_quaternion.w,
        ros_quaternion.x,
        ros_quaternion.y,
        ros_quaternion.z,
    ])
    return RPY_to_carla_rotation(roll, pitch, yaw)

def RPY_to_carla_rotation(roll, pitch, yaw):
    #############
    return carla.Rotation(
        roll=math.degrees(roll),
        #pitch=-math.degrees(pitch), # if not right-handed 
        pitch=math.degrees(pitch),
        yaw=math.degrees(yaw),
        #yaw=-math.degrees(yaw),    # if not right-handed    
    )