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

import sys

import rclpy
from rclpy.node import Node

import Adafruit_PCA9685

from ros2_adafruit_pwmhat_msgs.msg import PWMPinPulseLength, PWMPulseLength
from ros2_adafruit_pwmhat_msgs.msg import PWMPinAngle, PWMAngle

class ROS2_Adafruit_pwmhat_node(Node):
    # Node for driving Adafruit PWM hat for the Raspberry Pi 3
    # Subscribes to multiple topics to accept settings for named pins and for
    #    setting values in both raw pulse length and angle.

    def __init__(self):
        super().__init__('ros2_adafruit_pwmhat_node', namespace='pwmhatter')

        self.pwm = Adafruit_PCA9685.PCA9685()
        # frequency can be from 40 to 1000
        self.pwm_frequency = 100
        self.pwm.set_pwm_freq(self.pwm_frequency)

        # subscriptions for different message types (named, pins, angle)
        self.subs = []
        self.subs.append(self.create_subscription(
                    PWMPinAngle, 'pinAngle', self.pinAngle_callback))
        self.subs.append(self.create_subscription(
                    PWMAngle, 'angle', self.angle_callback))
        self.subs.append(self.create_subscription(
                    PWMPinPulseLength, 'pinPulseLength', self.pinPulseLength_callback))
        self.subs.append(self.create_subscription(
                    PWMPulseLength, 'pulseLength', self.pulseLength_callback))

        # THE FOLLOWING DATA WILL COME FROM PARAMETER FILES WHEN THAT WORKS FOR ROS2 AND PYTHON

        # parameters for each type of device that can be connected to a pin
        self.deviceTypes = {}
        self.deviceTypes['SG90'] = {
            'name': 'SG90',
            'range_degrees': [ -90, 90 ],
            # documentation says 1..2ms but experimentation says the range is a little larger
            'min_pulse_length': 0.6,    # pulse length for -90 degrees
            'max_pulse_length': 2.4     # pulse length for +90 degrees
            }

        # what's connected to each pin
        self.pins = []
        for ii in range(0,15):
            self.pins.append( { 'pin': ii, 'device_type': 'SG90' } )
        self.pins[8]['channel'] = 'tilt'
        self.pins[9]['channel'] = 'pan'

        # map channel names to pin numbers
        self.channels = {}
        for pinIndex in range(len(self.pins)):
            aPin = self.pins[pinIndex]
            if 'channel' in aPin:
                self.channels[aPin['channel']] = pinIndex

    def angle_callback(self, request):
        # set PWM based on channel name and angle
        self.get_logger().debug("PWMHat: chan/angle: chan=%s angle=%s"
                    % (request.chan, request.angle) )
        if request.chan in self.channels:
            pin = self.channels[request.chan]
            pulse_length = self.convert_angle_to_pulse_length(pin, request.angle_units, request.angle)
            self.set_pwm_by_pulse_length(pin, pulse_length)
        else:
            self.get_logger().error("PWMHAT: chan/angle: channel does not exist: chan=%s, angle=%s"
                    % (request.chan, request.angle) )
        return
        
    def pinAngle_callback(self, request):
        # set PWM based on pin and angle
        self.get_logger().debug("PWMHat: pin/angle: pin=%s angle=%s"
                    % (request.pin, request.angle) )
        if request.pin >= 0 and request.pin < len(self.pins):
            pulse_length = self.convert_angle_to_pulse_length(request.pin, request.angle_units, request.angle)
            self.set_pwm_by_pulse_length(request.pin, pulse_length)
        else:
            self.get_logger().error("PWMHAT: pin/angle: pin out of range: pin=%s, angle=%s"
                    % (request.pin, request.angle) )
        return

    def pulseLength_callback(self, request):
        # set PWM based on channel and pulse length
        self.get_logger().debug("PWMHat: chan/pulseLength: chan=%s pulse_length=%s"
                    % (request.chan, request.pulse_length) )
        if request.chan in self.channels:
            pin = self.channels[request.chan]
            self.set_pwm_by_pulse_length(pin, request.pulse_length)
        else:
            self.get_logger().error("PWMHAT: chan/pulseLength: channel does not exist: chan=%s, pulseLength=%s"
                    % (request.chan, request.pulse_length) )
        return

    def pinPulseLength_callback(self, request):
        # set PWM based on pin number and pulse length
        self.get_logger().debug("PWMHAT: pin/pulseLength: pin=%s pulse_length=%s"
                    % (request.pin, request.pulse_length) )
        if request.pin >= 0 and request.pin < len(self.pins):
            self.set_pwm_by_pulse_length(request.pin, request.pulse_length)
        else:
            self.get_logger().error("PWMHAT: pin/pulseLength: pin out of range: pin=%s, angle=%s"
                    % (request.pin, request.angle) )
        return

    def convert_angle_to_pulse_length(self, pin, units, angle):
        # given an angle and a pin description, return pulse length
        calc_pulse_length = 1.0
        conv_angle = angle
        if units == PWMAngle.RADIANS:
            conv_angle = angle * (180 / 3.14159265)

        pinInfo = self.pins[pin]
        deviceType = self.deviceTypes[pinInfo['device_type']]
        if conv_angle >= deviceType['range_degrees'][0] and conv_angle <= deviceType['range_degrees'][1]:
            # if angle is in range, adjust the range to be zero based
            conv_angle -= deviceType['range_degrees'][0]
            calc_pulse_length = (deviceType['max_pulse_length'] - deviceType['min_pulse_length']) / 180
            calc_pulse_length *= conv_angle
            calc_pulse_length += deviceType['min_pulse_length']
        else:
            self.get_logger().error("PWMHat: Pulse length calc: angle out of bounds: pin=%s, angle=%s"
                    % (pin, angle) )

        return calc_pulse_length

    def set_pwm_by_pulse_length(self, pin, pulse_length):
        # turn specified PWM channel on for the given pulse_length in microseconds
        pulse_scaler = 1000000              # microseconds
        pulse_scaler //= self.pwm_frequency # microseconds per cycle
        pulse_scaler //= 4096               # microseconds per PWM count
        conv_pulse_length = int((pulse_length * 1000) / pulse_scaler)
        conv_pin = int(pin)
        # pwm.set_pwm(channel, onTickTime, offTickTime) where on/off are within 0..4095
        self.get_logger().debug("set_pwm: pin=%s conv_pulse_length=%s" % (conv_pin, conv_pulse_length) )
        self.pwm.set_pwm(conv_pin, 0, conv_pulse_length)


def main(args=None):
    rclpy.init(args=args)

    pwmNode = ROS2_Adafruit_pwmhat_node()

    rclpy.spin(pwmNode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
