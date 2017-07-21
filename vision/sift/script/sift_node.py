#!/usr/bin/env python
import roslib
roslib.load_manifest('sift')
import os
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Int32

import rospkg

import numpy as np
from matplotlib import pyplot as plt
import time

from sift.srv import *

# from __future__ import print_function

class image_converter:

  def __init__(self):
    print("Initialize...")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.s = rospy.Service('sift_server', sift, self.handle_get_file_name)
    self.rospack = rospkg.RosPack()
    print("OK")

  def handle_get_file_name(self, req):
    print("Request : "+str(req.fileName))
    ################## SIFT Process ###############
    pol, rect = self.sift_process(req.fileName)
    resp = siftResponse()
    resp.points = pol
    resp.xmin = rect[0]
    resp.xmax = rect[1]
    resp.ymin = rect[2]
    resp.ymax = rect[3]
    return resp

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def sift_process(self, compare):
    polygon = Polygon()
    xmin = Int32(); ymin = Int32(); xmax = Int32(); ymax = Int32()
    tmpArr = list()
    MIN_MATCH_COUNT = 10

    img2 = self.cv_image          # queryImage
    filePath = self.rospack.get_path('sift')+'/training items/'+compare.lower()+'/'+compare.lower()+'_top_01.png'
    if os.path.isfile(filePath) :
      img1 = cv2.imread(filePath, 0) # trainImage
    else :
      print("Can not read compare image '"+compare.lower()+"_top_01.png'")
      return

    h,w = img1.shape
    img1 = cv2.resize(img1,((int)(w / 3), (int)(h / 3)),interpolation=cv2.INTER_CUBIC)
    min_x, min_y, max_x, max_y = 0, 0, 0, 0
    h,w = img1.shape
    
    # Initiatotete SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
      if m.distance < 0.7*n.distance:
        good.append(m)

    #print (len(good))


    if len(good)>MIN_MATCH_COUNT:
      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

      M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
      matchesMask = mask.ravel().tolist()

      h,w = img1.shape

      pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
      dst = cv2.perspectiveTransform(pts,M)
      print(dst)
      # bufferXmin = dst[0][0][0]; bufferYmin = dst[0][0][1]
      # bufferXmax = dst[0][0][0]; bufferYmax = dst[0][0][1]
      bufferX = []; bufferY = []
      for i in range(len(dst)) :
        point = Point32()
        # xmin = Int32(); ymin = Int32(); xmax = Int32(); ymax = Int32()
        point.x = dst[i][0][0]
        point.y = dst[i][0][1]
        point.z = 0
        polygon.points.append(point)
        print(str(i)+" : Append point ("+str(point.x)+", "+str(point.y)+")")
        bufferX.append(point.x); bufferY.append(point.y)
      
      bufferX.sort(); bufferY.sort()
      xmin.data = bufferX[0]; xmax.data = bufferX[-1]
      ymin.data = bufferY[0]; ymax.data = bufferY[-1]

      img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
      print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
      matchesMask = None
      point = Point32()
      point.x = -1
      point.y = -1
      point.z = -1
      xmin.data = -1; ymin.data = -1; xmax.data = -1; ymax.data = -1
      polygon.points = [point]


    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                      singlePointColor = None,
                      matchesMask = matchesMask, # draw only inliers
                      flags = 2)

    img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
    cv2.imwrite('result_Hinged_Ruled_Index_Cards.png', img3)
    plt.imshow(img3, 'gray'),plt.show(block=False)

    return polygon, (xmin, xmax, ymin, ymax)
    #'''

def main(args):
  ic = image_converter()
  rospy.init_node('sift_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)