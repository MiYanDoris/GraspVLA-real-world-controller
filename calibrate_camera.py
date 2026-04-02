import argparse
from collections import OrderedDict
import sys

sys.path.append('./vla_client')

import time

import transforms3d as t3d

from vla_client.controllers.franka_ros_controller import FrankaROSController
from vla_client.utils.cameras import CameraVisualizer, Camera

import rospy

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--front-camera", type=str, help="front camera serial number")
arg_parser.add_argument("--side-camera", type=str, help="side camera serial number")

if __name__ == "__main__":
    args = arg_parser.parse_args()

    import pyrealsense2

    context = pyrealsense2.context()
    cameras = context.query_devices()
    for i, dev in enumerate(cameras):
        print(f"Cevice {i}: {dev}")
    print('all cameras listed')

    rospy.init_node('test_camera', anonymous=True, disable_signals=True)

    ref_images = ["./res/front_ref.png", "./res/side_ref.png"]
    camera_visualizer = CameraVisualizer(OrderedDict({'front': Camera(args.front_camera), 'side': Camera(args.side_camera)}), ref_images=ref_images)

    robot_controller = FrankaROSController("logical", True)

    robot_controller.move_gripper('open')

    init_pos = [0.36307235, -0.00142398, 0.48912046]
    init_ori_mat = t3d.euler.euler2mat(-3.141592653589793, 0.0, 3.141592653589793 / 4)
    robot_controller.execute_waypoints([(init_pos, init_ori_mat, 1)])
    robot_controller.wait_for_reach(0.005, 5 / 180 * 3.141592653589793, timeout=2)

    time.sleep(2.0)
    print("Robot moved to reference state. Please correct your camera with Rviz window. ")
    input("press enter to quit...")
