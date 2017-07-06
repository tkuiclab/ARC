#! /usr/bin/env python
# down vote
# Kuba Ober and Leyu Wang's answer works great. Here, is a python implementation of the same algorithm.

import numpy as np
import math


def rotation_matrix(A,B):
# a and b are in the form of numpy array

   ax = A[0]
   ay = A[1]
   az = A[2]

   bx = B[0]
   by = B[1]
   bz = B[2]

   au = A/(np.sqrt(ax*ax + ay*ay + az*az))
   bu = B/(np.sqrt(bx*bx + by*by + bz*bz))

   R=np.array([[bu[0]*au[0], bu[0]*au[1], bu[0]*au[2]], [bu[1]*au[0], bu[1]*au[1], bu[1]*au[2]], [bu[2]*au[0], bu[2]*au[1], bu[2]*au[2]] ])


   return(R)


if __name__ == '__main__':
    cam_norm = (0, 0, 1)
    #obj_norm = (0.0935308, 0.591839, 0.800025)
    obj_normal=(-0.118330,-0.642154,0.757393)
    R = rotation_matrix(cam_norm, obj_normal)


    print(R)