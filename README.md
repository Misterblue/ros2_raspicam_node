# ros2-adafruit-pwmhat-node

A Python ROS2 node for control of the [Adafruit 16-Channel PWM/Servo HAT](https://www.adafruit.com/product/2327)    .

When run on the Raspberry Pi 3 that has the PWMHAT, this node subscribes to the topics:

| Topic                     | Message format                              | Data                                                           |
|:------------------------- | ------------------------------------------- | -------------------------------------------------------------- |
| /pwmhatter/angle          | ros2_adafruit_pwmhat_msgs/PWMAngle          | chan: channelName<br>angle: angleNumber<br>angle-unit: DEGREES |
| /pwmhatter/pinAngle       | ros2_adafruit_pwmhat_msgs/PWMPinAngle       | pin: pinNumber<br>angle: angleNumber<br>angle-unit: DEGREES    |
| /pwmhatter/pulseLength    | ros2_adafruit_pwmhat_msgs/PWMPulseLength    | chan: channelName<br>pulse-length: pulseMS                     |
| /pwmhatter/pinPulseLength | ros2_adafruit_pwmhat_msgs/PWMPinPulseLength | pin: pinNumber<br>pulse-length: pulseMS                        |

The angle could be either degrees or radians and the '''angle-unit''' variable can be the constant "DEGREES" or "RADIANS" with the default being degrees if not specified.

The stepping motor parameters are hard coded for the SG90 stepping motor which has a documented PWM pulse width from 1ms to 2ms but experimentation has shown that the real range for 180 degree movement is from 0.6ms to 2.4ms. The angle range parameters are for -90 degrees to +90 degrees. Other stepping motor definitions can be added and someday there will be an external parameter file (see note at bottom about ROS2, parameters, and Python).

### Building

The node is built using ROS2 on Raspbian. I have built the latest ROS2 sources on the Raspberry Pi 3 using a script at [ROS2OnPiTools](https://github.com/Misterblue/ROS2OnPiTools) that fetches and builds the latest ROS2 on Raspbian. Once ROS2 is available, presuming we're accessing the console on the Pi, the build instructions are:

```
source /opt/ros2/setup.bash    # set ROS2 paths into environment variables
cd
git clone https://github.com/Misterblue/ros2_adafruit_pwmhat_node.git
cd ros2_adafruit_pwmhat_node
colcon build --symlink-install
```

Since the code is only Python (none of that C++ stuff), this relies on some Adafruit supplied libraries to talk to the PWM Hat.

```
cd
sudo apt-get install -y python-smbus i2c-tools
sudo apt-get install -y build-essential python-dev
git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git
cd Adafruit_Python_PCA9685
sudo python3 setup.py install
```

Note the "python3" above -- ROS2 uses Python3.

### Running

The above build instructions build this package in its own directory so running requires setting up the environment for both ROS2 and this package:

```
source /opt/ros2/setup.bash        # set ROS2 paths into environment variables
cd $HOME/ros2_adafruit_pwmhat_node
source install/local-setup.bash    # set local package links into environment variables
ros2 run ros2_adafruit_pwmhat_node service
```

The namespace used in the topics (the "pwmhatter" listed in the 'topic's above) can be changed by adding parameters at the end of the "run" invocation line: "__ns:=namespace".

Once the node is running, example messages:

```
ros2 topic pub -1 /pwmhatter/pulseLength ros2_adafruit_pwmhat_msgs/PWMPulseLength "{
 'chan': 'tilt', 'pulse_length': 1.5 }"
 ros2 topic pub -1 /pwmhatter/pinAngle ros2_adafruit_pwmhat_msgs/PWMPinAngle "{ 'pin'
: 8, 'angle': -10 }"
```

### Notes

Some parameters are hard coded. As of October 6, 2018, all of the parameter API is not available to Python. Hopefully, the next ROS2 release (Dec 2018) will add those APIs.

If you look at the code in "ros2_adafruit_pwmhat_node" you will see that the compiled in parameters are for the SG90 stepping motor. Others will be added someday.
