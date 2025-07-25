import cv2
from .realsense import RealSenseCamera
from collections import OrderedDict
import rospy
import threading
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import numpy as np


class Camera:
    def __init__(self, serial_number):
        self.camera = RealSenseCamera(serial_number, fps=30)
        self.k_real = self.camera.get_camera_intrinsics_matrix()
        print(f'camera intrinsics {self.k_real}')

    def get_frame(self):
        rgb, depth= self.camera.get_frames()
        rgb = self.crop_frame(rgb)
        return rgb
    
    def crop_frame(self, image):
        assert image.shape[0] == 480 and image.shape[1] == 640
        # center crop and resize
        size_y, size_x, _ = image.shape
        start_x = size_x//2 - size_y//2
        start_y = size_y//2 - size_y//2
        new_image = image[start_y : start_y + size_y, start_x : start_x + size_y, :]
        res = cv2.resize(new_image, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)

        return res


class CameraVisualizer:
    def __init__(self, cameras: OrderedDict, hz=10, size=(512, 512), ref_images=None):
        self.cameras = cameras
        self.cv_bridge = CvBridge()

        self.hz = hz
        self.size = size

        if ref_images is not None:
            assert len(ref_images) == len(cameras), 'Number of ref images must be equal to number of cameras'
            self.ref_images = []
            for ref_image_path in ref_images:
                ref_image = cv2.resize(cv2.imread(ref_image_path), size)
                robot_mask = np.sum(ref_image, axis=-1) > 0
                ref_image[robot_mask, 0] = 0
                ref_image[robot_mask, 1] = 255
                ref_image[robot_mask, 2] = 0
                self.ref_images.append(ref_image)
        else:
            self.ref_images = None

        self.publishers = {}
        self.bboxes = {}
        for name in cameras:
            self.publishers[name] = rospy.Publisher(f'/cameras/{name}', ImageMsg, queue_size=0)
            if self.ref_images:
                self.publishers[f"{name}_ref"] = rospy.Publisher(f'/cameras/{name}_ref', ImageMsg, queue_size=0)

        def fn():
            rate = rospy.Rate(self.hz)
            while not rospy.is_shutdown():
                self._step()
                rate.sleep()

        self.step_thread = threading.Thread(target=fn, daemon=True)
        self.step_thread.start()

    def _step(self):
        frames = []
        for idx, (name, camera) in enumerate(self.cameras.items()):
            try:
                color_frame = camera.get_frame()
                color_frame = cv2.cvtColor(cv2.resize(color_frame.copy(), self.size), cv2.COLOR_RGB2BGR)
            except Exception as e:
                print(f'Error getting frame from camera {name}: {e}')
                break

            if name in self.bboxes:
                color_frame = self.draw_bbox(color_frame, *self.bboxes[name])
            self.draw_center_cross(color_frame)

            if self.ref_images is not None:
                diff = (color_frame * 0.8 + self.ref_images[idx] * 0.2).astype('uint8')
                try:
                    self.publishers[name + '_ref'].publish(self.cv_bridge.cv2_to_imgmsg(diff, "bgr8"))
                except:
                    pass

            frames.append(color_frame)
            try:
                self.publishers[name].publish(self.cv_bridge.cv2_to_imgmsg(color_frame, "bgr8"))
            except:
                raise

    def draw_center_cross(self, frame):
        cross_length = 50
        cv2.line(frame, (frame.shape[0]//2-cross_length, frame.shape[1]//2), (frame.shape[0]//2+cross_length, frame.shape[1]//2), color=(0, 255, 0), thickness=1)
        cv2.line(frame, (frame.shape[0]//2, frame.shape[1]//2-cross_length), (frame.shape[0]//2, frame.shape[1]//2+cross_length), color=(0, 255, 0), thickness=1)

    def draw_bbox(self, frame, bbox, image_size):
        original_size = frame.shape[:2]
        # directly resize the bbox
        bbox = np.array(bbox)
        image_size = np.array(image_size)
        bbox = (bbox * np.array([*(original_size/image_size)]*2)).astype(int)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        return frame
    
    def set_bbox(self, camera_name, bbox):
        self.bboxes[camera_name] = bbox

    def clear(self):
        self.bboxes.clear()