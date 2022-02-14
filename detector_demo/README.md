# ME134 Detector Demo
See scripts folder for relevant ROS nodes:
 * blob_detector.py : detects green circle in camera output
 * template_detector.py: detects template in camera output
 * display.py: displays detector output overlayed over camera output

Run the detector demo as follows:
1. Copy this folder to ros/packages
2. Run "catkin_make" in ros directory to build messages
3. Run "roslaunch detector_demo template_detector.launch"
