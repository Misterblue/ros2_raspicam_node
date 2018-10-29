# Copyright 2018 Robert Adams
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import queue
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import picamera

class ROS2_raspicam_node(Node):

    def __init__(self):
        super().__init__('ros2_raspicam_node', namespace='raspicam')

        self.camera = picamera.PiCamera()
        time.sleep(1);  # let camera initialization complete

        self.initialize_publisher()
        self.set_camera_parameters()
        self.initialize_capture_queue()

    def destroy_node(self):
        # overlay Node function called when class is being stopped and camera needs closing
        # if hasattr(self, 'compressed_publisher') and self.compressed_publisher != None:
        #     # nothing to do
        if hasattr(self, 'camera') and self.camera != None:
            self.camera.close()
        super().destroy_node()

    def initialize_publisher(self):
        self.compressed_publisher = self.create_publisher(CompressedImage, 'raspicam_compressed')
        self.frame_num = 0
        
    def set_camera_parameters(self):
        # NOTE: WHEN PYTHON HAS ROS2 PARAMETERS, PARAMERTIZE ALL THE FOLLOWING

        # https://picamera.readthedocs.io/en/release-1.13/api_camera.html
        # off, auto, sunlight, cloudy, shade, trungsten, florescent, incandescent, flash, horizon
        self.camera.awb_mode = str(self.get_parameter_or('camera_awb_mode', 'auto'))
        self.get_parameter_set_if_set('camera_annotate_background',
                lambda xx: self.camera.annotate_background = xx, str())
        self.get_parameter_set_if_set('camera_annotate_foreground',
                lambda xx: self.camera.annotate_foreground = xx, str())
        self.get_parameter_set_if_set('camera_annotate_text',
                lambda xx: self.camera.annotate_text = xx, str())
        self.get_parameter_set_if_set('camera_annotate_text_size',
                lambda xx: self.camera.annotate_text_size = xx, int())
        # self.camera.annotate_background = picamera.Color('black')
        # self.camera.annotate_foreground = picamera.Color('yellow')
        # self.camera.annotate_text = '')
        # self.camera.annotate_text_size = 10  # 6..160, default 32
        # brightness: 0..100, default 50
        self.camera.brightness = int(self.get_parameter_or('camera_brightness', 55))
        # Contrast: -100..100, default 0
        self.camera.contrast = int(self.get_parameter_or('camera_contrast', 0))
        self.get_parameter_set_if_set('camera_exif_copyright',
                lambda xx: self.camera.exif_tage['IFDO.Copyright'] = xx, str())
        self.get_parameter_set_if_set('camera_exif_user_comment',
                lambda xx: self.camera.exif_tage['EXIF.UserComment'] = xx, str())
        # self.camera.exif_tags['IFD0.Copyright'] = 'Copyright 2018, Robert Adams';
        # self.camera.exif_tags['EXIF.UserComment'] = '';
        # Exposure compenstation: -25..25, default 0, one step = 1/6 F-stop
        self.camera.exposure_compensation = int(self.get_parameter_or('camera_exposure_compenstation', 0))
        # off, auto, night, backlight, spotlight, sports, snow, beach, antishake, fireworks
        self.camera.exposure_mode = str(self.get_parameter_or('camera_exposure_mode', 'auto))
        # the camera is upside down in initial setup
        self.camera.hflip = self.get_parameter_or('camera.hflip', True)
        self.camera.vflip = self.get_parameter_or('camera.vflip', True)
        # 'none', 'negative', 'solarize', 'sketch', 'denoise', 'emboss', 'oilpaint',
        # 'hatch', 'gpen', 'pastel', 'watercolor', 'film', 'blur', 'saturation',
        # 'colorswap', 'washedout', 'posterise', 'colorpoint', 'colorbalance', 'cartoon', 'deinterlace1',
        # 'deinterlace2'
        self.camera.image_effect = str(self.get_parameter_or('camera_image_effect', 'none'))
        # 'average' 'spot' 'backlit' 'matrix'
        self.camera.meter_mode = str(self.get_parameter_or('camera_meter_mode', 'backlit'))
        # 640/480, 800/600, 1280/720
        self.image_width = int(self.get_parameter_or('camera_image_width', 800))
        self.image_height = int(self.get_parameter_or('camera_image_height', 600))
        # self.camera.resolution = ( self.image_width, self.image_height )
        self.get_logger().debug('CAM: setting capture resolution = %s/%s'
                % (self.camera.resolution[0], self.camera.resolution[1]))
        # Saturation: -100..100, default 0
        self.camera.saturation = int(self.get_parameter_or('camera_saturation', 0))
        # Sharpness: -100..100, default 0
        self.camera.sharpness = int(self.get_parameter_or('camera_sharpness', 10))


    def initialize_capture_queue(self):
        # Create a queue and two threads to capture and then push the images to the topic
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()
        # self.capture_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to capture camera images and place in queue
        self.capture_event = threading.Event()
        self.capturer = threading.Thread(target=self.take_pictures, name='capturer')

        # thread to read queue and send them to the topic
        self.publisher_event = threading.Event()
        self.publisher = threading.Thread(target=self.publish_images, name='publisher')

        self.capturer.start()
        self.publisher.start()

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publisher') and self.publisher.is_alive():
            self.publisher.join()
        if hasattr(self, 'capturer') and self.capturer.is_alive():
            self.capturer.join()


    def take_pictures(self):
        # Take compressed images and put into the queue.
        # 'jpeg', 'rgb'
        try:
            for capture in self.camera.capture_continuous(self.write_capture(self), format='jpeg'):
                if self.capture_event.is_set():
                    break
                time.sleep(0.5)
                # The exit flag could have been set while in the sleep
                if self.capture_event.is_set():
                    break
        except:
            self.get_logger().error('CAM: exiting take_pictures because of exception')

    class write_capture():
        # Writer object that writes the passed data to the queue
        def __init__(self, pparent):
            self.parent = pparent

        def write(self, d):
            if not self.parent.capture_event.is_set():
                with self.parent.queue_lock:
                    msg = CompressedImage()
                    msg.data = d
                    msg.format = 'jpeg'
                    msg.header.frame_id = str(self.parent.frame_num)
                    self.parent.frame_num += 1
                    self.parent.get_logger().debug('CAM: capture frame. size=%s, frame=%s'
                            % (len(d), msg.header.frame_id) )
                    # msg.header.stamp = time.Time
                    self.parent.capture_queue.put(msg)

        def flush(self):
            return

    def publish_images(self):
        # Loop reading from capture queue and send to ROS topic
        while True:
            if self.publisher_event.is_set():
                break
            try:
                msg = self.capture_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.publisher_event.is_set():
                break
            if msg != None:
                self.get_logger().debug('CAM: sending frame. frame=%s'
                                    % (msg.header.frame_id) )
                self.compressed_publisher.publish(msg)

    def get_parameter_or(self, param, default):
        # Helper function to return value of a parameter or a default if not set
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            ret = default
        else:
            ret = param_desc.value
        return ret

def main(args=None):
    rclpy.init(args=args)

    camNode = ROS2_raspicam_node()

    try:
        rclpy.spin(camNode)
    except KeyboardInterrupt:
        camNode.get_logger().info('CAM: Keyboard interrupt')

    camNode.stop_workers()

    camNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
