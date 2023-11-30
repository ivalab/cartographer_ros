#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file cartographer_helper.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-29-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Transform
import tf2_ros

from scipy.spatial.transform import Rotation


class CartographerHelper:
    def __init__(self):
        rospy.init_node("Cartographer_helper_node", anonymous=False)

        # Subscribers and publishers.
        self._pose_pub = rospy.Publisher("/slam/pose", PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber("/tracked_pose", PoseStamped, self.__pose_callback)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_br = tf2_ros.TransformBroadcaster()

        # rospy.spin()

    def __pose_callback(self, msg):
        pose_out = PoseWithCovarianceStamped()
        pose_out.header = msg.header
        pose_out.pose.pose = msg.pose

        # assign covariance value
        xyz_sigma = 0.05  # meters
        rpy_sigma = np.deg2rad(0.5)  # rads
        for r in range(6):
            index = r * 6 + r
            val = xyz_sigma**2 if r < 3 else rpy_sigma**2
            pose_out.pose.covariance[index] = val

        # Publish pose
        self._pose_pub.publish(pose_out)

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            # Inverse tf
            Tom_msg = TransformStamped()
            try:
                Tcmo_msg = self._tf_buffer.lookup_transform(
                    "cartographer_map", "odom", rospy.Time(0), rospy.Duration(0.5)
                )
                Tom_msg.header.frame_id = "odom"
                Tom_msg.header.stamp = Tcmo_msg.header.stamp
                Tom_msg.child_frame_id = "slam_map"
                Tom_msg.transform = self.__invert_tf(Tcmo_msg.transform)
                self._tf_br.sendTransform(Tom_msg)
            except tf2_ros.TransformException as ex:
                rospy.logwarn(f"error: {ex}")
            rate.sleep()

    def __invert_tf(self, msg):
        mat_in = np.eye(4)
        mat_in[:3, :3] = Rotation.from_quat(
            [
                msg.rotation.x,
                msg.rotation.y,
                msg.rotation.z,
                msg.rotation.w,
            ]
        ).as_matrix()
        mat_in[:3, 3] = [
            msg.translation.x,
            msg.translation.y,
            msg.translation.z,
        ]
        mat_out = np.linalg.inv(mat_in)

        tf_out = Transform()
        tf_out.translation.x = mat_out[0, 3]
        tf_out.translation.y = mat_out[1, 3]
        tf_out.translation.z = mat_out[2, 3]
        quat = Rotation.from_matrix(mat_out[:3, :3]).as_quat()
        tf_out.rotation.x = quat[0]
        tf_out.rotation.y = quat[1]
        tf_out.rotation.z = quat[2]
        tf_out.rotation.w = quat[3]
        return tf_out


if __name__ == "__main__":
    try:
        ch = CartographerHelper()
        ch.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception Caught!!!")
    

