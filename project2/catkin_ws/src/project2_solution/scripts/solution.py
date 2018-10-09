#!/usr/bin/env python
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

initial_loop = True

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


def matrix_multiplication(matrix, vector):
    vector.append(1)
    return [sum([vector[x]*matrix[n][x] for x in range(len(vector))]) for n in range(len(matrix))]


def between_angle(v1, v2):
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    return np.arccos(np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0))

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
    return T1


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
    return T2


def transform_camera(T1, T2):
    global initial_loop, angle, x_axis, origin2_camera, origin2_robot
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

    # We do not know the camera frame orientation so we don't rotate it at the first loop
    if initial_loop:
        initial_loop = False
        R3 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, 0))

        # Concatenated transformation matrix
        T3 = tf.transformations.concatenate_matrices(D3, R3)
        print("Camera View transformation = {}".format(T3))

        # Calculate the coordinated of origin from object_frame w.r.t camera_frame
        # Origin is the translation part from base_frame to object_frame
        origin2 = tf.transformations.translation_from_matrix(T1)   

        # Origin referenced to robot_frame
        origin2_robot = matrix_multiplication(tf.transformations.inverse_matrix(T2), origin2.tolist())
        origin2_robot = origin2_robot[:(len(origin2_robot) - 1)]

        # Origin2 referenced from camera_frame
        origin2_camera = matrix_multiplication(
            tf.transformations.inverse_matrix(T3), origin2_robot)
        origin2_camera = origin2_camera[:(len(origin2_camera) - 1)]


        # X-axis for camera frame viewed from its own frame
        x_axis = [1, 0, 0]

        # Angle difference between x-axis and the origin of object_frame
        angle = between_angle(x_axis, origin2_camera)
        print("Angle = {}".format(angle))

    # Vector through which x-axis has to rotate
    normal_vector = np.cross(x_axis, origin2_camera)

    R3 = tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(
                angle, normal_vector))

    T3 = tf.transformations.concatenate_matrices(D3, R3)

    camera_transform.transform = transform_coordinate_frame(T3)
    br.sendTransform(camera_transform)


def publish_transforms():
    T1 = transform_object()
    T2 = transform_robot()
    transform_camera(T1, T2)


if __name__ == '__main__':
    rospy.init_node('project2_solution')

    initial_loop = True

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)