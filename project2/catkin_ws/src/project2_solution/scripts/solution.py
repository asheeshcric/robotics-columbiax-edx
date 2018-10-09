#!/usr/bin/env python
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg


def transform_coordinate_frame(T):
    new_frame = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    translation = tf.transformations.translation_from_matrix(T)
    new_frame.translation.x = translation[0]
    new_frame.translation.y = translation[1]
    new_frame.translation.z = translation[2]
    new_frame.rotation.x = q[0]
    new_frame.rotation.y = q[1]
    new_frame.rotation.z = q[2]
    new_frame.rotation.w = q[3]
    return new_frame


def transform_object():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    T1 = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)),
        tf.transformations.translation_matrix((0.0, 1.0, 1.0)))
    object_transform.transform = transform_coordinate_frame(T1)
    br.sendTransform(object_transform)


def transform_robot():
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    T2 = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))),
        tf.transformations.translation_matrix((0.0, -1.0, 0.0)))
    robot_transform.transform = transform_coordinate_frame(T2)
    br.sendTransform(robot_transform)


def transform_camera():
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    # camera_transform.transform.translation.x = 0.0
    # camera_transform.transform.translation.y = 0.1
    # camera_transform.transform.translation.z = 0.1
    # camera_quaternion = tf.transformations.quaternion_from_euler(0.0, -0.46, 0.0)
    # camera_transform.transform.rotation.x = camera_quaternion[0]
    # camera_transform.transform.rotation.y = camera_quaternion[1]
    # camera_transform.transform.rotation.z = camera_quaternion[2]
    # camera_transform.transform.rotation.w = camera_quaternion[3]
    
    # Translation matrix with displacement on y-axis and z-axis
    D3 = tf.transformations.translation_matrix([0.0, 0.1, 0.1])

    br.sendTransform(camera_transform)


def publish_transforms():
    transform_object()
    transform_robot()
    transform_camera()


if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)