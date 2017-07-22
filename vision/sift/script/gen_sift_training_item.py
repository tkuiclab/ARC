#!/usr/bin/env python
import numpy as np
import cv2
import math
import os
import glob
import rospkg

# code by FengFeng

MIN_MATCH_COUNT = 10


arc_item_path = rospkg.RosPack().get_path('arc') + '/Training items'
sift_item_path = rospkg.RosPack().get_path('sift') + '/training items'

# if not os.path.isdir('./training items') :
# 	os.mkdir('training items')

if not os.path.isdir(sift_item_path) :
	os.mkdir(sift_item_path)


print('arc_item_path = ' + arc_item_path)
print ('sift_item_path = ' + sift_item_path)

for dirname, dirnames, filenames in os.walk(arc_item_path): #os.walk('./Training items'):
	for filename in filenames:
		if os.path.splitext(filename)[-1] == '.png' :
			#print(os.path.join(dirname, filename))

			img_src = cv2.imread(os.path.join(dirname, filename))
						
			w, h, c = img_src.shape

			img_src=cv2.resize(img_src,((int)(h / 4), (int)(w / 4)),interpolation=cv2.INTER_CUBIC)

			w, h, c = img_src.shape
			min_x, min_y, max_x, max_y = 0, 0, 0, 0

			hsv_src = cv2.cvtColor(img_src, cv2.COLOR_BGR2HSV)
			w, h, c = hsv_src.shape
			min_x, min_y, max_x, max_y = 0, 0, 0, 0

			for _h in range(h) :
				for _w in range(w) :
					if (hsv_src[_w, _h, 1] > 0.3*255) and (hsv_src[_w, _h, 2] > 0.3*255) :
						if min_x == 0 :   min_x = _w
						elif min_x > _w : min_x = _w
						if min_y == 0 :   min_y = _h
						elif min_y > _h : min_y = _h
						if max_x == 0 :   max_x = _w
						elif max_x < _w : max_x = _w
						if max_y == 0 :   max_y = _h
						elif max_y < _h : max_y = _h
		
			Roi2 = img_src[min_x:max_x, min_y:max_y]


			last_dir_name = os.path.basename(os.path.normpath(dirname))

			sift_item_dir = sift_item_path + '/'+ str.lower(last_dir_name)
			#sift_item_dir = str.lower(sift_item_dir)

			
			if not os.path.isdir(sift_item_dir) :
				os.mkdir(sift_item_dir)
				print 'mkdir  ' + sift_item_dir

			

			print('Create ' + sift_item_dir +'/' + str.lower(filename))
			cv2.imwrite(sift_item_dir +'/' + str.lower(filename), Roi2)

