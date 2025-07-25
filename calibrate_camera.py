import argparse
from collections import OrderedDict
import sys
sys.path.append('./vla_client')
import time
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

    time.sleep(2.0)
    print(f"Robot moved to reference state. Please correct your camera with Rviz window. ")
    input(f"press enter to quit...")

