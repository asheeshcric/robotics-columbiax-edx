#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF


def S_matrix(w):
    S = numpy.zeros((3, 3))
    S[0, 1] = -w[2]
    S[0, 2] = w[1]
    S[1, 0] = w[2]
    S[1, 2] = -w[0]
    S[2, 0] = -w[1]
    S[2, 1] = w[0]
    return S


# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    number_of_joints = len(joint_transforms)
    dq = numpy.zeros(number_of_joints)

    # -------------------- Fill in your code here ---------------------------

    end_effector_t_b = tf.transformations.inverse_matrix(b_T_ee_current)
    end_effector_t_ee = numpy.dot(end_effector_t_b, b_T_ee_desired)

    translation_b_t_ee = tf.transformations.translation_from_matrix(end_effector_t_ee)

    rotation_r_ee = end_effector_t_ee[:3, :3]

    angle, axis = rotation_from_matrix(end_effector_t_ee)

    rotation = numpy.dot(angle, axis)

    invert_rotation_ee_b = tf.transformations.inverse_matrix(rotation_r_ee)
    linear_gain = 3
    rotational_gain = 1.5
    delta_for_x = numpy.append(translation_b_t_ee * linear_gain, rotation * rotational_gain)

    prop_gain = 1
    dot_x = prop_gain * delta_for_x

    capitol_j = numpy.empty((6, 0))

    for joint_j in range(number_of_joints):
        b_t_joint = joint_transforms[joint_j]
        joint_t_b = tf.transformations.inverse_matrix(b_t_joint)
        joint_t_ee = numpy.dot(joint_t_b, b_T_ee_current)
        ee_t_joint = tf.transformations.inverse_matrix(joint_t_ee)

        ee_r_joint = ee_t_joint[:3, :3]

        j_t_ee = tf.transformations.translation_from_matrix(joint_t_ee)
        capitol_s = S_matrix(j_t_ee)

        vertical_joint = numpy.append(
            numpy.append(ee_r_joint, numpy.dot(-ee_r_joint, capitol_s), axis=1),
            numpy.append(numpy.zeros([3, 3]), ee_r_joint, axis=1),
            axis=0
        )
        capitol_j = numpy.column_stack((capitol_j, vertical_joint[:, 5]))

    joint_pin_vertical = numpy.linalg.pinv(capitol_j, rcond=1e-2)

    dq = numpy.dot(joint_pin_vertical, dot_x)

    if red_control is True:
        joint_pin_vertical = numpy.linalg.pinv(capitol_j, rcond=0)
        n_dq = numpy.dot(
            numpy.identity(7) - numpy.dot(joint_pin_vertical, capitol_j),
            numpy.array([q0_desired - q_current[0], 0, 0, 0, 0, 0, 0]))
        dq = numpy.dot(joint_pin_vertical, dot_x) + n_dq

    # ----------------------------------------------------------------------
    return dq


def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                   t.translation.y,
                                                   t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans, rot)
    return T


# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa - 1.0) * axis[0] * axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa - 1.0) * axis[0] * axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa - 1.0) * axis[1] * axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis


class CartesianControl(object):

    # Initialization
    def __init__(self):
        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        # Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        # This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()

    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms,
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms,
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)

    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0, 0, 1])
        x = numpy.array([1, 0, 0])
        dot = numpy.dot(z, axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map:
            self.x_current = T
            return
        for i in range(0, len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0],
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0],
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle,
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T)
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)


if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
