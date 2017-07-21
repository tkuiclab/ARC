import numpy as np
import cv2
from matplotlib import pyplot as plt
import time
import sys
import os

tStart = time.time()
MIN_MATCH_COUNT = 10
max_good = 0

path = './training items/' + sys.argv[1]
img2 = cv2.imread(sys.argv[2], 0) # unknow
#h,w = img2.shape
#img2 = cv2.resize(img2, ((int)(w / 4), (int)(h / 4)),interpolation=cv2.INTER_CUBIC)

for dirname, dirnames, filenames in os.walk(path):
  for filename in filenames:
    print(os.path.join(dirname, filename))

    img1 = cv2.imread(path + '/' + str.lower(filename),0) # trainImage

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
    print (len(good))
    if len(good) > max_good : 
      max_good = len(good)
      path2 = path + '/' + str.lower(filename)

# match
print (max_good)

if max_good > MIN_MATCH_COUNT:
  img1 = cv2.imread(path2, 0)
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

  src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
  dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

  M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
  matchesMask = mask.ravel().tolist()

  h,w = img1.shape

  pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
  dst = cv2.perspectiveTransform(pts,M)
  print(dst)

  img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

  draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

  img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
  cv2.imwrite('result_' + sys.argv[1] + '.png', img3)

  tEnd = time.time()	#end clock
  print "It cost %f sec" % (tEnd - tStart)

  plt.imshow(img3, 'gray'),plt.show()

else:
  print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
  matchesMask = None



