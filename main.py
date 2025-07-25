import rospy
import sys
sys.path.append('./vla_client')
from vla_client.modes import GraspMode
import argparse

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--front-camera", type=str, help="front camera serial number")
arg_parser.add_argument("--side-camera", type=str, help="side camera serial number")
arg_parser.add_argument("--automatically_put_down", action='store_true', help="automatically put down the object after grasping")
arg_parser.add_argument("--extented_finger", action='store_true', help="use the extention of the finger")
arg_parser.add_argument("--server-ip", type=str)
arg_parser.add_argument("--server-port", type=int)

if __name__ == "__main__":
    args = arg_parser.parse_args()

    import pyrealsense2
    context = pyrealsense2.context()
    cameras = context.query_devices()
    for i, dev in enumerate(cameras):
        print(f"Cevice {i}: {dev}")
    print('all cameras listed')

    rospy.init_node('vla_controller', anonymous=True, disable_signals=True)
    mode = GraspMode(args)
    mode.run()

    rospy.signal_shutdown('done')
