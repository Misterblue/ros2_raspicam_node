# ros2-raspicam_node

A Python ROS2 node for generating images from the Raspberry Pi camera. This uses the wonderful [PiCamera] to drive the camera while this node pacakage takes the JPEG images and sends them out over a ROS2 topic.

| Topic                         | Message Format                   | Data                                                       |
| ----------------------------- | -------------------------------- | ---------------------------------------------------------- |
| /raspicam/raspicam_compressed | sensor_msgs.msg.CompressedImage_ | data: image<br>format: "jpeg"<br>header.frame_id: frameNum |

Since this project is being worked on, it does not output a raw image, output camera info, or accept parameters. That will happen in a later step.

Internally, this starts two threads one of which captures an image from the camera which it puts into a FIFO queue while the other thread reads from the queue and publishes the image to the topic.

### Building

The node is built using ROS2 on Raspbian. I have built the latest ROS2 sources on the Raspberry Pi 3 using a script at [ROS2OnPiTools] that fetches and builds the latest ROS2 on Raspbian. Once ROS2 is available, presuming we're accessing the console on the Pi, the build instructions are:

```
source /opt/ros2/setup.bash    # set ROS2 paths into environment variables
cd
git clone https://github.com/Misterblue/ros2_raspicam_node.git
cd ros2_raspicam_node
colcon build --symlink-install
```

Since the code is only Python (none of that C++ stuff), this relies on some Adafruit supplied libraries to talk to the camera:

```
cd
sudo apt-get install python3-picamera
```

Also, be sure to enable the camera using the Raspberry Pi configuration program.

### Running

The above build instructions build this package in its own directory so running requires setting up the environment for both ROS2 and this package:

```
source /opt/ros2/setup.bash        # set ROS2 paths into environment variables
cd $HOME/ros2_raspicam_node
source install/local-setup.bash    # set local package links into environment variables
ros2 run ros2_raspicam_node service
```

As of November 1, 2018, building Python only modules with ```colcon``` doesn't add the path to the install directory to AMENT_PREFIX_PATH so the above ```ros2 run``` command will say "program not found". This should be fixed in a later release but, as of now, you will have to manually add the path to the search path.

### Notes

This program uses parameters to set the camera statistics but, as of November 1,2018, this requires
building from the latest ROS2 sources as the 'bouncy' release didn't yet have parameters for
Python programs. Hopefully that will be all fixed in the December 2018 release.

[PiCamera]: https://picamera.readthedocs.io/en/release-1.13/

[ROS2OnPiTools]: https://github.com/Misterblue/ROS2OnPiTools


