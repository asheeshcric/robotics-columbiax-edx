#!/usr/bin/env python

from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg


def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]
    return t


def convert_from_message(msg):
    R = tf.transformations.quaternion_matrix((msg.orientation.x,
                                              msg.orientation.y,
                                              msg.orientation.z,
                                              msg.orientation.w))
    T = tf.transformations.translation_matrix((msg.position.x,
                                               msg.position.y,
                                               msg.position.z))
    return numpy.dot(T, R)


def convert_from_trans_message(msg):
    R = tf.transformations.quaternion_matrix((msg.rotation.x,
                                              msg.rotation.y,
                                              msg.rotation.z,
                                              msg.rotation.w))
    T = tf.transformations.translation_matrix((msg.translation.x,
                                               msg.translation.y,
                                               msg.translation.z))
    return numpy.dot(T, R)


def fetch_vector(point1, point2):
    return numpy.subtract(point1, point2)


def fetch_unit_vector(point1, point2):
    v = fetch_vector(point1, point2)
    return v / numpy.linalg.norm(v)


def fetch_distance(point1, point2):
    return numpy.linalg.norm(fetch_vector(point1, point2))


def fetch_maximum_number_points(point1, point2, step):
    return max(numpy.ceil(numpy.true_divide(abs(fetch_vector(point1, point2)), step)))


def fetch_closest_point(tree, q):
    dists = [fetch_distance(q_pos, q) for i, q_pos in enumerate(d["pos_configuration_space"] for d in tree)]
    min_dist_index = dists.index(min(dists))
    nearest_point = tree[min_dist_index].get("pos_configuration_space")
    return min_dist_index, nearest_point


def fetch_point_at_dist(nearest_point, rand_point, factor):
    vector = fetch_unit_vector(rand_point, nearest_point)
    vector *= factor
    vector = numpy.add(vector, nearest_point)
    return vector


class MoveArm(object):

    def __init__(self):
        print
        "Motion Planning Initializing..."
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Some info and conventions about the robot that we hard-code in here
        # min and max joint values are not read in Python urdf, so we must hard-code them here
        self.num_joints = 7
        self.q_min = []
        self.q_max = []
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        self.q_min.append(-3.14159);
        self.q_max.append(3.14159)
        # How finely to sample each joint
        self.q_sample = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1]
        self.joint_names = ["lwr_arm_0_joint",
                            "lwr_arm_1_joint",
                            "lwr_arm_2_joint",
                            "lwr_arm_3_joint",
                            "lwr_arm_4_joint",
                            "lwr_arm_5_joint",
                            "lwr_arm_6_joint"]

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState,
                         self.joint_states_callback)

        # Subscribe to command for motion planning goal
        rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform,
                         self.move_arm_cb)

        # Publish trajectory command
        self.pub_trajectory = rospy.Publisher("/joint_trajectory", trajectory_msgs.msg.JointTrajectory,
                                              queue_size=1)

        # Initialize variables
        self.joint_state = sensor_msgs.msg.JointState()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)
        print
        "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',
                                                      moveit_msgs.srv.GetStateValidity)
        print
        "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "lwr_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        print
        "MoveIt! interface ready"

        # Options
        self.subsample_trajectory = True
        print
        "Initialization done."

    def get_joint_val(self, joint_state, name):
        if name not in joint_state.name:
            print
            "ERROR: joint name not found"
            return 0
        i = joint_state.name.index(name)
        return joint_state.position[i]

    def set_joint_val(self, joint_state, q, name):
        if name not in joint_state.name:
            print
            "ERROR: joint name not found"
        i = joint_state.name.index(name)
        joint_state.position[i] = q

    """ Given a complete joint_state data structure, this function finds the values for 
    our arm's set of joints in a particular order and returns a list q[] containing just 
    those values.
    """

    def q_from_joint_state(self, joint_state):
        q = []
        for i in range(0, self.num_joints):
            q.append(self.get_joint_val(joint_state, self.joint_names[i]))
        return q

    """ Given a list q[] of joint values and an already populated joint_state, this 
    function assumes that the passed in values are for a our arm's set of joints in 
    a particular order and edits the joint_state data structure to set the values 
    to the ones passed in.
    """

    def joint_state_from_q(self, joint_state, q):
        for i in range(0, self.num_joints):
            self.set_joint_val(joint_state, q[i], self.joint_names[i])

    """ This function will perform IK for a given transform T of the end-effector. It 
    returns a list q[] of 7 values, which are the result positions for the 7 joints of 
    the left arm, ordered from proximal to distal. If no IK solution is found, it 
    returns an empy list.
    """

    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "world_link"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = self.q_from_joint_state(res.solution.joint_state)
        return q

    """ This function checks if a set of joint angles q[] creates a valid state, or 
    one that is free of collisions. The values in q[] are assumed to be values for 
    the joints of the left arm, ordered from proximal to distal. 
    """

    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        current_joint_state = deepcopy(self.joint_state)
        current_joint_state.position = list(current_joint_state.position)
        self.joint_state_from_q(current_joint_state, q)
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = current_joint_state
        res = self.state_valid_service(req)
        return res.valid

    def make_discrete_path(self, nearest_point, target_point):
        step_weight = self.q_sample
        maximum_points = fetch_maximum_number_points(target_point, nearest_point, step_weight)

        m_vector = numpy.true_divide(fetch_vector(target_point, nearest_point), maximum_points - 1)
        b_vector = numpy.true_divide(numpy.subtract(numpy.multiply(nearest_point, maximum_points), target_point),
                                     maximum_points - 1)
        t_vector = [m_vector * i + b_vector for i in range(int(maximum_points) + 1)]
        return t_vector

    def is_free_collision_path(self, nearest_point, target_point):
        path = self.make_discrete_path(nearest_point, target_point)
        for row in path:
            if self.is_state_valid(row) is False:
                return False
        return True

    def fetch_random_point(self):
        return [random.uniform(self.q_min[i], self.q_max[i]) for i in range(self.num_joints)]

    def motion_plan(self, q_start, q_goal, q_min, q_max):
        rrt_node_object = {
            "pos_configuration_space": q_start,
            "par_node": -1
        }
        rrt_node_list = [rrt_node_object.copy()]

        max_nodes = 150
        max_time = 240
        start_time = rospy.get_rostime().secs
        current_time = rospy.get_rostime().secs

        while (len(rrt_node_list) < max_nodes) or ((current_time - start_time) < max_time):
            rand_point = self.fetch_random_point()

            minimum_dist_index, nearest_point = fetch_closest_point(rrt_node_list, rand_point)

            target_point = fetch_point_at_dist(nearest_point, rand_point, 0.5)

            if self.is_free_collision_path(nearest_point, target_point) is True:
                rrt_node_object.update({"pos_configuration_space": target_point})
                rrt_node_object.update({"par_node": minimum_dist_index})
                rrt_node_list.append(rrt_node_object.copy())

                if self.is_free_collision_path(target_point, q_goal) is True:
                    par_node = len(rrt_node_list) - 1
                    rrt_node_object.update({"par_node": par_node})
                    rrt_node_object.update({"pos_configuration_space": q_goal})
                    rrt_node_list.append(rrt_node_object.copy())
                    break
                else:
                    pass
            else:
                pass

            current_time = rospy.get_rostime().secs

        q_list = [q_goal]
        par_node = rrt_node_list[-1].get("par_node")

        while True:
            q_list.insert(0, rrt_node_list[par_node].get("pos_configuration_space"))
            if par_node <= 0:
                break
            else:
                par_node = rrt_node_list[par_node].get("par_node")

        copy_q_list = [q_list[0]]

        for i in range(len(q_list) - 2):
            if self.is_free_collision_path(q_list[i], q_list[i + 2]) is False:
                copy_q_list.append(q_list[i])

        copy_q_list.append(q_list[-1])
        q_list = copy_q_list

        return q_list

    def create_trajectory(self, q_list, v_list, a_list, t):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            point.velocities = list(v_list[i])
            point.accelerations = list(a_list[i])
            point.time_from_start = rospy.Duration(t[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def create_trajectory(self, q_list):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def project_plan(self, q_start, q_goal, q_min, q_max):
        q_list = self.motion_plan(q_start, q_goal, q_min, q_max)
        joint_trajectory = self.create_trajectory(q_list)
        return joint_trajectory

    def move_arm_cb(self, msg):
        T = convert_from_trans_message(msg)
        self.mutex.acquire()
        q_start = self.q_from_joint_state(self.joint_state)
        print
        "Solving IK"
        q_goal = self.IK(T)
        if len(q_goal) == 0:
            print
            "IK failed, aborting"
            self.mutex.release()
            return
        print
        "IK solved, planning"
        trajectory = self.project_plan(numpy.array(q_start), q_goal, self.q_min, self.q_max)
        if not trajectory.points:
            print
            "Motion plan failed, aborting"
        else:
            print
            "Trajectory received with " + str(len(trajectory.points)) + " points"
            self.execute(trajectory)
        self.mutex.release()

    def joint_states_callback(self, joint_state):
        self.mutex.acquire()
        self.joint_state = joint_state
        self.mutex.release()

    def execute(self, joint_trajectory):
        self.pub_trajectory.publish(joint_trajectory)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()
