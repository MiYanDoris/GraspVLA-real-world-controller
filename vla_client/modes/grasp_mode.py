from ..utils.timer import Timer
from ..utils import input_typed
from ..utils.cameras import Camera, CameraVisualizer

import pynput
from pynput import keyboard
import transforms3d as t3d
import controllers
import zmq
from collections import OrderedDict
import numpy as np
import io
import PIL.Image
import termios
import sys
import time
import threading


class GraspMode:
    def __init__(self, args):
        self.args = args
        self.front_camera = Camera(args.front_camera)
        self.side_camera = Camera(args.side_camera)
        self.camera_visualizer = CameraVisualizer(OrderedDict({'front': self.front_camera, 'side': self.side_camera}))

        self.robot_controller = controllers.FrankaROSController(args.extented_finger)

        self.robot_lock = threading.RLock()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{args.server_ip}:{args.server_port}")

    def run(self):
        while True:
            self._goto_init()
            self.robot_controller.reset_history()

            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            user_input = input_typed('Enter target object name (empty string for exit): ', str)
            if user_input == '':
                break
            instruction = f'pick up {user_input}'
            print(f"The instruction is: {instruction}")
            self._run_once(instruction)

    def _run_once(self, instruction):
        self.finished = False
        self.paused = False
        timer = Timer()
        with pynput.keyboard.Listener(on_press=self._on_keyboard_press, on_release=self._on_keyboard_release):
            while not self.finished:
                time.sleep(0.005)
                if self.paused:
                    time.sleep(0.1)
                    continue

                with self.robot_lock:
                    if self.paused:
                        continue
                    if self.robot_controller.busy:
                        continue
                    self.robot_controller.abnormal = False

                    step_info = {}

                    front_rgb = self.front_camera.get_frame()
                    side_rgb = self.side_camera.get_frame()

                    prev_eef_pose = self.robot_controller.get_eef_pose(0.3)
                    eef_pose = self.robot_controller.get_eef_pose()

                    message = {
                        "text": instruction,
                        "front_view_image": [self._compress_image(front_rgb)],
                        "side_view_image": [self._compress_image(side_rgb)],
                        "proprio_array": [prev_eef_pose, prev_eef_pose, prev_eef_pose, eef_pose],
                        "compressed": True,
                    }

                    with timer("request"):
                        response = self.socket.send_pyobj(message)
                        print('-' * 40)
                        print('observation sent...')
                        response = self.socket.recv_pyobj()
                        cost_time = timer.get_time('request')
                        print(f'current request costs {cost_time:.2f}s')

                    assert response["info"] == "success", "invalid response from server"
                    debug = response["debug"]
                    if "pose" in debug:
                        self.robot_controller.update_tf('grasp_pose', debug["pose"][0], debug["pose"][1])
                    if debug.get("bbox") is not None:
                        self.camera_visualizer.set_bbox('front', (debug["bbox"][0], (224, 224)))
                        self.camera_visualizer.set_bbox('side', (debug["bbox"][1], (224, 224)))

                    delta_actions = response["result"]
                    current_position = eef_pose[:3]
                    current_orientation_mat = t3d.euler.euler2mat(*eef_pose[3:6])
                    abs_actions = []
                    cnt = 0
                    for action in delta_actions:
                        assert action[6] in [-1., 1., 0.], "please quantize gripper actions"
                        if cnt % 2 == 1:
                            gripper_action = 'open' if action[6] > 0 else 'close'
                            print(f'delta translation: x={action[0]:.3f}, y={action[1]:.3f}, z={action[2]:.3f} {gripper_action}')
                        target_position = current_position + action[:3]
                        target_orientation_mat = t3d.euler.euler2mat(*action[3:6]) @ current_orientation_mat
                        abs_actions.append((target_position, target_orientation_mat, action[6]))
                        self.robot_controller.update_tf('target_' + str(cnt), abs_actions[cnt][0], t3d.euler.mat2euler(abs_actions[cnt][1]))
                        cnt = cnt + 1
                        current_position = target_position
                        current_orientation_mat = target_orientation_mat

                    step_info["actions"] = abs_actions
                    try:
                        self.execute_waypoints_and_wait(abs_actions, timeout=2)
                    except RuntimeError:
                        continue
        
        self.camera_visualizer.clear()

        # if automatically_put_down is set and the gripper is closed, 
        # this program will automatically descend 10cm before releasing the object
        # to prevent the object from failling and breaking.
        if self.args.automatically_put_down and self.robot_controller.gripper_status == "close":
            # put down
            eef_pose = self.robot_controller.get_eef_pose()
            current_position = eef_pose[:3]
            current_orientation_mat = t3d.euler.euler2mat(*eef_pose[3:6])
            try:
                self.execute_waypoints_and_wait([([*current_position[:2], current_position[2] - 0.10], current_orientation_mat, 0)], timeout=2)
            except RuntimeError:
                pass

    def execute_waypoints_and_wait(self, waypoints, timeout):
        self.robot_controller.execute_waypoints(waypoints)
        if timeout == 0:
            return
        try:
            self.robot_controller.wait_for_reach(0.005, 5/180*np.pi, timeout=timeout)
        except TimeoutError as e:
            print(f"controller timeout: {e}")

    def _on_keyboard_press(self, key):
        if hasattr(key, "char"):
            # pause/resume
            if key.char == 'p':
                print('pause...' if not self.paused else 'resume...')
                with self.robot_lock:
                    self.paused = not self.paused
                    print('paused' if self.paused else 'resumed')
            # quit
            elif key.char == 'q':
                print('finish trajectory')
                self.finished = True
            else:
                pass
        else:
            if key == keyboard.Key.space:
                with self.robot_lock:
                    self.paused = not self.paused
                print('pause' if self.paused else 'resume')
            elif key == keyboard.Key.esc:
                exit(1)

    def _on_keyboard_release(self, key):
        import sys
        if hasattr(key, "char") or key in [keyboard.Key.space]:
            sys.stdout.write('\b \b')
            sys.stdout.flush()

    def _goto_init(self):
        while True:
            try:
                self.robot_controller.move_gripper('open')
                break
            except RuntimeError:
                pass
        init_pos = [0.36307235, -0.00142398, 0.48912046]
        init_ori_mat = t3d.euler.euler2mat(-np.pi, 0., np.pi/4)
        while True:
            try:
                self.execute_waypoints_and_wait([(init_pos, init_ori_mat, 1)], timeout=2)
                break
            except RuntimeError:
                pass

    def _compress_image(self, image):
        bytes_io = io.BytesIO()
        PIL.Image.fromarray(image).save(bytes_io, format="JPEG")
        return bytes_io.getvalue()
