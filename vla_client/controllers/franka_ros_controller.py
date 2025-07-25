import rospy
import tf2_ros
import actionlib
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
from sensor_msgs.msg import JointState
from franka_gripper.msg import GraspGoal, GraspAction, MoveAction, MoveGoal
from franka_msgs.msg import FrankaState, ErrorRecoveryAction, ErrorRecoveryGoal
from dynamic_reconfigure.client import Client

import numpy as np
import threading
from collections import deque
import transforms3d as t3d
import time

from .common import MAX_HEIGHT, POS_TOL, ROT_TOL, FORCE_LIMIT


class FrankaROSController:
    ROBOT_MODE_OTHER=0
    ROBOT_MODE_IDLE=1
    ROBOT_MODE_MOVE=2
    ROBOT_MODE_GUIDING=3
    ROBOT_MODE_REFLEX=4
    ROBOT_MODE_USER_STOPPED=5
    ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
    ROBOT_MODES_UNCONTROLLABLE = [ROBOT_MODE_GUIDING, ROBOT_MODE_USER_STOPPED, ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY]
    BASE_FRAME_ID = 'panda_link0'
    EEF_FRAME_ID = 'panda_EE'
    def __init__(self, time_mode, extented_finger):
        assert time_mode in ['logical', 'physical']
        self.time_mode = time_mode

        if extented_finger:
            self.REAL_EEF_TO_SIM_EEF = np.array([
                [1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.03],
                [0., 0., 0., 1.],
            ])
        else:
            self.REAL_EEF_TO_SIM_EEF = np.array([
                [1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.],
            ])

        # set controller parameters
        self.dynamic_param_client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")
        if self.time_mode == 'logical':
            self.dynamic_param_client.update_configuration({"filter_d_order": 1})
            self.dynamic_param_client.update_configuration({"filter_params": 0.005})
            self.dynamic_param_client.update_configuration({"translational_stiffness": 2000.0})
            self.dynamic_param_client.update_configuration({"translational_damping": 89.0})
            self.dynamic_param_client.update_configuration({"rotational_stiffness": 150.0})
            self.dynamic_param_client.update_configuration({"rotational_damping": 7.0})
            self.dynamic_param_client.update_configuration({"nullspace_stiffness": 0.2})
            self.dynamic_param_client.update_configuration({"joint1_nullspace_stiffness": 100.0})
            for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
                self.dynamic_param_client.update_configuration({"translational_clip_" + direction: 0.01})
                self.dynamic_param_client.update_configuration({"rotational_clip_" + direction: 0.05})
        elif self.time_mode == 'physical':
            self.dynamic_param_client.update_configuration({"filter_d_order": 3})
            self.dynamic_param_client.update_configuration({"filter_params": 0.004})
            self.dynamic_param_client.update_configuration({"translational_stiffness": 2000.0})
            self.dynamic_param_client.update_configuration({"translational_damping": 89.0})
            self.dynamic_param_client.update_configuration({"rotational_stiffness": 150.0})
            self.dynamic_param_client.update_configuration({"rotational_damping": 7.0})
            self.dynamic_param_client.update_configuration({"nullspace_stiffness": 0.2})
            self.dynamic_param_client.update_configuration({"joint1_nullspace_stiffness": 100.0})
            for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
                self.dynamic_param_client.update_configuration({"translational_clip_" + direction: 0.02})
                self.dynamic_param_client.update_configuration({"rotational_clip_" + direction: 0.05})

        # to notify the keep_available thread that the robot state has updated
        self.robot_mode = None
        self.robot_mode_ts = 0.
        self.abnormal = False
        self.busy = False
        self.reset_history()
        self.franka_state_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self._franka_state_callback)
        self.franka_gripper_state_subscriber = rospy.Subscriber('/franka_gripper/joint_states', JointState, self._franka_gripper_state_callback)
        self.franka_f_ext_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self._franka_f_ext_callback)
        self.eef_pose_publisher = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=0)
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_grasp_client.wait_for_server()
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper_move_client.wait_for_server()
        self.error_recovery_client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)
        self.error_recovery_client.wait_for_server()
        self._try_recover_from_error()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.gripper_status = None
        self.latest_franka_state = None
        self.external_force = None

        while self.latest_franka_state is None or self.external_force is None:
            time.sleep(0.1)

        self.thread_monitor_robot_state = threading.Thread(target=self.monitor_robot_state, daemon=True)
        self.thread_monitor_robot_state.start()

    def reset_history(self):
        self.history_waypoints = deque(maxlen=100)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # wait for the first transform
        time.sleep(0.3)

    def set_waypoint(self, pos, ori_mat):
        mat = np.eye(4)
        mat[:3, 3] = pos
        mat[:3, :3] = ori_mat
        mat = mat @ np.linalg.inv(self.REAL_EEF_TO_SIM_EEF)

        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = mat[:3, 3]
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = t3d.quaternions.mat2quat(mat[:3, :3])
        self.eef_pose_publisher.publish(msg)
        self.last_waypoint = (pos, ori_mat)

    def monitor_robot_state(self):
        rate = rospy.Rate(10)
        while True:
            rate.sleep()
            if self.external_force[2] > FORCE_LIMIT or self.robot_mode != self.ROBOT_MODE_MOVE:
                self.abnormal = True
                self.busy = True
                print(f'large external force or robot abnormal, trying to recover...')
                self._try_recover_from_error()
                print('moving up by 5cm...')
                target = (self.last_waypoint[0] + np.random.uniform(low=[-0.02, -0.02, 0.05], high=[0.02, 0.02, 0.05]), self.last_waypoint[1])
                self.set_waypoint(*target)
                try:
                    self._wait_for_reach(*target, pos_tol=0.01, rot_tol=10/180*np.pi, timeout=2, ignore_error=True)
                except TimeoutError as e:
                    pass
                print('finished moving up')
                self.reset_history()
                self.busy = False
                print(f'robot recovered')

    def _try_recover_from_error(self):
        rate = rospy.Rate(50)
        now = time.time()
        while True:
            rate.sleep()
            if self.robot_mode_ts < now:
                continue
            if self.robot_mode not in self.ROBOT_MODES_UNCONTROLLABLE and self.robot_mode != self.ROBOT_MODE_MOVE:
                break
            if self.robot_mode == self.ROBOT_MODE_MOVE:
                return
        goal = ErrorRecoveryGoal()
        self.error_recovery_client.send_goal(goal)
        # the error recovery may not response if the robot is not in error mode
        # and there is no reliable way to check if the robot is in error mode
        # so we just wait for a fixed time
        print('waiting for error recovery to finish...')
        self.error_recovery_client.wait_for_result()
        print('error recovery finished')

    def _franka_state_callback(self, msg):
        self.latest_qpos = np.array(msg.q)
        # if the robot changed from uncontrollable mode to controllable mode, the user may have manually moved the robot
        # so we clear the history waypoints to avoid unexpected behavior
        if self.robot_mode in self.ROBOT_MODES_UNCONTROLLABLE and msg.robot_mode not in self.ROBOT_MODES_UNCONTROLLABLE:
            self.reset_history()
        self.robot_mode = msg.robot_mode
        self.robot_mode_ts = time.time()
        self.latest_franka_state = msg

    def _franka_gripper_state_callback(self, msg):
        self.gripper_joint_positions = msg.position

    def _franka_f_ext_callback(self, msg):
        self.external_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def execute_waypoints(self, waypoints):
        # clip for safety
        waypoints = [
            ((*pos[:2], min(pos[2], MAX_HEIGHT)), ori_mat, gripper_action)
            for pos, ori_mat, gripper_action in waypoints
        ]

        if self.time_mode == 'logical':
            self._execute_waypoints_logical(waypoints)
        elif self.time_mode == 'physical':
            self._execute_waypoints_physical(waypoints)

    def _execute_waypoints_logical(self, waypoints):
        """
        This function assumes the given waypoints are near each other.
        If not, please use multiple calls to this function.
        """
        for waypoint in waypoints:
            # now gripper
            gripper_action = waypoint[2]
            desired_gripper_status = None
            if gripper_action == -1. and self.gripper_status != 'close':
                desired_gripper_status = 'close'
            elif gripper_action == 1. and self.gripper_status != 'open':
                desired_gripper_status = 'open'
            if desired_gripper_status is not None:
                try:
                    self._wait_for_reach(*waypoint[:2], pos_tol=POS_TOL, rot_tol=ROT_TOL, timeout=1)
                except TimeoutError as e:
                    print(f"control: {e}")
                self.move_gripper(desired_gripper_status)

            if self.abnormal:
                raise RuntimeError('robot abnormal')
            # print(f'ewp: {waypoint[0]} {waypoint[2]}')
            self.set_waypoint(*waypoint[:2])
            # intermediate waypoints can have larger tolerance
            try:
                self._wait_for_reach(*waypoint[:2], pos_tol=0.01, rot_tol=10/180*np.pi, timeout=0.1)
            except TimeoutError as e:
                pass

            if self.abnormal:
                raise RuntimeError('robot abnormal')

            self.history_waypoints.append((*waypoint[:2], 1 if self.gripper_status == 'open' else -1))

    def _execute_waypoints_physical(self, waypoints):
        if self.abnormal:
            raise RuntimeError('robot abnormal')
        first_ga = 0.
        for wp in waypoints:
            if wp[-1] != 0:
                first_ga = wp[-1]
                break
        if not (self.gripper_status == 'close' and first_ga == 1.):
            self.set_waypoint(*waypoints[-1][:2])

        for pos, ori_mat, gripper_action in waypoints:
            desired_gripper_status = None
            if gripper_action == -1. and self.gripper_status != 'close':
                desired_gripper_status = 'close'
            elif gripper_action == 1. and self.gripper_status != 'open':
                desired_gripper_status = 'open'
            if desired_gripper_status is not None:
                if self.abnormal:
                    raise RuntimeError('robot abnormal')
                try:
                    self._wait_for_reach(pos, ori_mat, pos_tol=0.01, rot_tol=10/180*np.pi, timeout=2, ignore_error=True)
                except TimeoutError:
                    pass
                self.move_gripper(desired_gripper_status)
                break

    def get_eef_pose(self, before=0.):
        if self.time_mode == 'logical':
            if len(self.history_waypoints) == 0:
                self.history_waypoints.append(self._get_eef_pose_from_ros(0.))
            idx = round(before / 0.1)
            if idx + 1 > len(self.history_waypoints):
                waypoint = self.history_waypoints[0]
            else:
                waypoint = self.history_waypoints[-(1+idx)]
        else:
            waypoint = self._get_eef_pose_from_ros(before)
        return np.array([*waypoint[0], *t3d.euler.mat2euler(waypoint[1]), waypoint[2]])

    def _get_eef_pose_from_ros(self, before):
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(self.BASE_FRAME_ID, self.EEF_FRAME_ID, rospy.Time.now() - rospy.Duration(before), rospy.Duration(0.5))
                break
            except tf2_ros.ExtrapolationException as e:
                continue

        real_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        real_ori_quat_sxyz = np.array([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
        real_pose_mat = np.eye(4)
        real_pose_mat[:3, 3] = real_pos
        real_pose_mat[:3, :3] = t3d.quaternions.quat2mat(real_ori_quat_sxyz)
        sim_pose_mat = real_pose_mat @ self.REAL_EEF_TO_SIM_EEF
        sim_pos = sim_pose_mat[:3, 3]
        return (sim_pos, sim_pose_mat[:3, :3], 1 if self.gripper_status == 'open' else -1)

    def get_qpos(self):
        return [*self.latest_qpos, *self.gripper_joint_positions]

    def get_eef_width(self):
        return sum(self.gripper_joint_positions)

    def move_gripper(self, status):
        if self.abnormal:
            raise RuntimeError('robot abnormal')
        if status == 'open':
            goal = MoveGoal(width=0.08, speed=0.1)
            self.gripper_move_client.send_goal(goal)
            self.gripper_move_client.wait_for_result(timeout=rospy.Duration(2))
        elif status == 'close':
            goal = GraspGoal(width=0.04, speed=0.2, force=30)
            goal.epsilon.inner = 0.04
            goal.epsilon.outer = 0.04
            self.gripper_grasp_client.send_goal(goal)
            self.gripper_grasp_client.wait_for_result(timeout=rospy.Duration(2))
        self.gripper_status = status

    def update_tf(self, frame_id, xyz, rpy):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.BASE_FRAME_ID
        msg.child_frame_id = frame_id
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = t3d.euler.euler2quat(*rpy)
        self.tf_broadcaster.sendTransform(msg)

    def wait_for_reach(self, pos_tol, rot_tol, timeout):
        self._wait_for_reach(*self.last_waypoint, pos_tol, rot_tol, timeout)

    def _wait_for_reach(self, pos, rot_mat, pos_tol, rot_tol, timeout, ignore_error=False):
        begin_time = time.time()
        while True:
            if self.abnormal and not ignore_error:
                raise RuntimeError('robot abnormal')
            latest_mat = np.array(self.latest_franka_state.O_T_EE).reshape((4,4)).T @ self.REAL_EEF_TO_SIM_EEF
            pos2 = latest_mat[:3, 3]
            rot_mat2 = latest_mat[:3, :3]
            pos_diff = np.linalg.norm(pos - pos2)
            rot_diff = abs(np.arccos((np.trace(rot_mat @ rot_mat2.T) - 1) / 2))
            if pos_diff < pos_tol and rot_diff < rot_tol:
                break
            spent_time = time.time() - begin_time
            if spent_time > timeout:
                raise TimeoutError(f"control timeout with pos_diff {pos_diff:.4f}m angle_diff {rot_diff / np.pi * 180:.2f}d spent_time {spent_time:.4f}s")
            time.sleep(0.1)
