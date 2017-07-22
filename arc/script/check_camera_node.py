#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import sys
import datetime
from sensor_msgs.msg import PointCloud2

tmp = datetime.datetime.now()

pkg_name = "realsense_camera"
launch_name = "sr300_nodelet_rgbd.launch"
pkg_path = rospkg.RosPack().get_path(pkg_name)
full_path = pkg_path + "/launch/" + launch_name

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

rsCamera = roslaunch.parent.ROSLaunchParent(uuid, [full_path])

def callback(data):
  localtm = datetime.datetime.now()
  sys.stdout.write("Realsense is alive : "+str(localtm.strftime("%Y-%m-%d %H:%M:%S"))+'\r')
  global tmp
  tmp = localtm

def launch_realsense():
  rospy.logwarn("Trying restart realsense_camera")
  try:
    rsCamera.shutdown() # does a exist in the current namespace
  except AttributeError:
    rospy.logwarn("rsCamera not defined")

  rsCamera.start()
  # Wait Re-launch
  while (datetime.datetime.now() - tmp).total_seconds() > 5:
    sys.stdout.write("Wait..."+str((datetime.datetime.now() - tmp).total_seconds())+'\r')

def listener():
  rospy.init_node('check_camera_node', anonymous=True, disable_signals=True)
  # launch_init()

  rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback)

  while(1) :
    if (datetime.datetime.now() - tmp).total_seconds() > 5 :
      rospy.logwarn("Loss /camera/depth_registered/points")
      launch_realsense()
    rospy.sleep(0.1)
  # rospy.spin()

if __name__ == '__main__':
  listener()