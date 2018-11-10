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
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import picamera

class ROS2_raspicam_node(Node):

    def __init__(self):
        super().__init__('ros2_raspicam_node', namespace='raspicam')

        self.set_parameter_defaults( [
            ('compressed_image', Parameter.Type.BOOL, True),
            ('image_topic', Parameter.Type.STRING, 'raspicam_uncompressed'),
            ('compressed_image_topic', Parameter.Type.STRING, 'raspicam_compressed'),

            # off, auto, sunlight, cloudy, shade, trungsten, florescent, incandescent, flash, horizon
            ('camera_awb_mode', Parameter.Type.STRING, 'auto'),
            # ('camera_annotate_background', Parameter.Type.STRING, 'black'),
            # ('camera_annotate_foreground', Parameter.Type.STRING, 'yellow'),
            # ('camera_annotate_text', Parameter.Type.STRING, 'fun image'),
            # text size: 6..160, default 32
            # ('camera_annotate_text_size', Parameter.Type.INTEGER, 10),
            # brightness: 1..100, default 50
            ('camera_brightness', Parameter.Type.INTEGER, 55),
            # Contrast: -100..100, default 0
            ('camera_contrast', Parameter.Type.INTEGER, 0),
            # ('camera_exif_copyright', Parameter.Type.STRING, 'Copyrightt 2018 MY NAME'),
            # ('camera_user_comment', Parameter.Type.STRING, 'SOMETHING INFORMATIVE'),
            # Exposure compenstation: -25..25, default 0, one step = 1/6 F-stop
            ('camera_exposure_compenstation', Parameter.Type.INTEGER, 0),
            # off, auto, night, backlight, spotlight, sports, snow, beach, antishake, fireworks
            ('camera_exposure_mode', Parameter.Type.STRING, 'auto'),
            # the camera is upside down in initial setup
            ('camera_hflip', Parameter.Type.BOOL, True),
            ('camera_vflip', Parameter.Type.BOOL, True),
            # 'none', 'negative', 'solarize', 'sketch', 'denoise', 'emboss', 'oilpaint',
            # 'hatch', 'gpen', 'pastel', 'watercolor', 'film', 'blur', 'saturation',
            # 'colorswap', 'washedout', 'posterise', 'colorpoint', 'colorbalance', 'cartoon', 'deinterlace1',
            # 'deinterlace2'
            ('camera_image_effect', Parameter.Type.STRING, 'none'),
            # 'average' 'spot' 'backlit' 'matrix'
            ('camera_meter_mode', Parameter.Type.STRING, 'average'),
            # 640/480, 800/600, 1280/720
            ('camera_image_width', Parameter.Type.INTEGER, 640),
            ('camera_image_height', Parameter.Type.INTEGER, 480),
            # Saturation: -100..100, default 0
            ('camera_saturation', Parameter.Type.INTEGER, 0),
            # Sharpness: -100..100, default 0
            ('camera_sharpness', Parameter.Type.INTEGER, 10),
            ] )

        self.camera = picamera.PiCamera()
        time.sleep(1);  # let camera initialization complete

        self.initialize_publisher()
        self.set_camera_parameters()
        self.initialize_capture_queue()

    def destroy_node(self):
        # overlay Node function called when class is being stopped and camera needs closing
        # if hasattr(self, 'publisher') and self.publisher != None:
        #     # nothing to do
        if hasattr(self, 'camera') and self.camera != None:
            self.camera.close()
        super().destroy_node()

    def initialize_publisher(self):
        if self.get_parameter_value('compressed_image'):
            self.publisher = self.create_publisher(CompressedImage,
                                self.get_parameter_value('compressed_image_topic'))
        else:
            self.publisher = self.create_publisher(Image,
                                self.get_parameter_value('image_topic'))
        self.frame_num = 0
        
    def set_camera_parameters(self):
        # https://picamera.readthedocs.io/en/release-1.13/api_camera.html
        self.camera.awb_mode = self.get_parameter_value('camera_awb_mode')
        self.parameter_set_if_set('camera_annotate_background',
                lambda xx: setattr(self.camera, 'annotate_background', xx))
        self.parameter_set_if_set('camera_annotate_foreground',
                lambda xx: setattr(self.camera, 'annotate_foreground', xx))
        self.parameter_set_if_set('camera_annotate_text',
                lambda xx: setattr(self.camera, 'annotate_text', xx))
        self.parameter_set_if_set('camera_annotate_text_size',
                lambda xx: setattr(self.camera, 'annotate_text_size', xx))
        self.camera.brightness = self.get_parameter_value('camera_brightness')
        self.camera.contrast = self.get_parameter_value('camera_contrast')
        if self.has_parameter('camera_exif_copyright'):
            self.camera.exif_tage['IFDO.Copyright'] = self.get_parameter_value('camera_exif_copyright')
        if self.has_parameter('camera_exif_user_comment'):
            self.camera.exif_tage['EXIF.UserComment'] = self.get_parameter_value('camera_exif_user_comment')
        self.camera.exposure_compensation = self.get_parameter_value('camera_exposure_compenstation')
        self.camera.exposure_mode = self.get_parameter_value('camera_exposure_mode')
        self.camera.hflip = self.get_parameter_value('camera_hflip')
        self.camera.vflip = self.get_parameter_value('camera_vflip')
        self.camera.image_effect = self.get_parameter_value('camera_image_effect')
        self.camera.meter_mode = self.get_parameter_value('camera_meter_mode')
        self.image_width = self.get_parameter_value('camera_image_width')
        self.image_height = self.get_parameter_value('camera_image_height')
        self.camera.resolution = ( self.image_width, self.image_height )
        self.get_logger().debug('CAM: setting capture resolution = %s/%s'
                % (self.camera.resolution[0], self.camera.resolution[1]))
        self.camera.saturation = self.get_parameter_value('camera_saturation')
        self.camera.sharpness = self.get_parameter_value('camera_sharpness')

    def initialize_capture_queue(self):
        # Create a queue and two threads to capture and then push the images to the topic
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()
        # self.capture_queue = queue.SimpleQueue()  # introduced in Python 3.7

        # thread to capture camera images and place in queue
        self.capture_event = threading.Event()
        self.capturer_thread = threading.Thread(target=self.take_pictures, name='capturer')

        # thread to read queue and send them to the topic
        self.publisher_event = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_images, name='publisher')

        self.capturer_thread.start()
        self.publisher_thread.start()

    def stop_workers(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publisher_thread') and self.publisher_thread.is_alive():
            self.publisher_thread.join()
        if hasattr(self, 'capturer_thread') and self.capturer_thread.is_alive():
            self.capturer_thread.join()


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
                self.publisher.publish(msg)

    def get_parameter_or(self, param, default):
        # Helper function to return value of a parameter or a default if not set
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            ret = default
        else:
            ret = param_desc.value
        return ret

    def get_parameter_value(self, param):
        # Helper function to return value of a parameter
        ret = None
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            raise Exception('Fetch of parameter that does not exist: ' + param)
        else:
            ret = param_desc.value
        return ret

    def set_parameter_defaults(self, params):
        # If a parameter has not been set externally, set the value to a default.
        # Passed a list of "(parameterName, parameterType, defaultValue)" tuples.
        parameters_to_set = []
        for (pparam, ptype, pdefault) in params:
            if not self.has_parameter(pparam):
                parameters_to_set.append( Parameter(pparam, ptype, pdefault) )
        if len(parameters_to_set) > 0:
            self.set_parameters(parameters_to_set)

    def parameter_set_if_set(self, param, set_function):
        # If there is a parameter set, do set_function with the value
        if self.has_parameter(param):
            set_function(self.get_parameter_value(param))

    def has_parameter(self, param):
        # Return 'True' if a parameter by that name is specified
        param_desc = self.get_parameter(param)
        if param_desc.type_== Parameter.Type.NOT_SET:
            return False
        return True


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
