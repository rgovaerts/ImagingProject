# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <codecell>

"""
Created on Sat Nov 09 16:50:30 2013

@author: Remy
"""

from numpy import *
from math import sqrt

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

"""
Estimation of transformation parameters (rotation and translation)
http://nghiaho.com/?page_id=671
http://nghiaho.com/uploads/code/rigid_transform_3D.py_
last consulted 8/11/2013
"""

def rigid_transform_3D(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # total points
    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))
    # dot is matrix multiplication for array
    H = dot(AA.T,BB)
    U, S, Vt = linalg.svd(H)
    R = dot(Vt,U.T)
    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = dot(Vt,U.T)
    t = -dot(R,centroid_A) + centroid_B
    return R, t
    
import pylab
from scipy.spatial import KDTree
def ICP_3D(a,b, error):
    """
    Returns A transformed so that it matches B as closely as posible
    
    The algorithm steps are :
    
        1.Associate points by the nearest neighbor criteria (for each point in one point cloud find the closest point in the second point cloud).
        2.Estimate transformation parameters (rotation and translation) using a mean square cost function (the transform would align best each point to its match found in the previous step).
        3.Transform the points using the estimated parameters.
        4.Iterate (re-associate the points and so on).
        
    http://en.wikipedia.org/wiki/Iterative_closest_point
    http://en.wikipedia.org/wiki/Point_Set_Registration#Iterative_closest_point
    """
    n = a.shape[0] #number of points
    B = b
    tree1 = KDTree(a)
    tree2 = KDTree(b)
    # 1.Associate points by the nearest neighbor criteria
    dist,indices = tree1.query(b, k=1, eps=0, p=2, distance_upper_bound=inf)
    A = zeros_like(a)
    j = 0
    for i in indices:
        A[j] = a[i]
        j+=1
    # 2.Estimate transformation parameters (rotation and translation)
    ret_R, ret_t = rigid_transform_3D(A, b)
    # 3.Transform the points using the estimated parameters.
    A2 = dot(ret_R,A.T) + tile(ret_t, (n, 1)).T
    A2 = A2.T
    # Find the square error (MSE)
    err = A2 - B
    mse = sum(np.square(err))/n
    print mse
    if mse > error:
        # 4.Iterate
        A2 = ICP_3D(A2, B, error)
        return A2
    else:
        return A2
        
import os
import numpy as np
from matplotlib import pyplot



n = 100
points = random.rand(n,3)
points[:,0] = np.linspace(-1, 1, n)
points[:,1] = 0
points[:,2] = np.square(np.sin(np.linspace(0, np.pi, n))) / 3

angle1 = random.rand() % 0.2
angle2 = random.rand() % 0.2
angle3 = random.rand() % 0.2
#X rot matrix
rotX = np.zeros([3,3])
rotX[2,2] = math.cos(angle1)
rotX[1,1] = math.cos(angle1)
rotX[1,2] = -math.sin(angle1)
rotX[2,1] = math.sin(angle1)
rotX[0,0] = 1
#Y rot matrix
rotY = np.zeros([3,3])
rotY[0,0] = math.cos(angle2)
rotY[2,2] = math.cos(angle2)
rotY[2,0] = -math.sin(angle2)
rotY[0,2] = math.sin(angle2)
rotY[1,1] = 1
#Z rot matrix
rotZ = np.zeros([3,3])
rotZ[0,0] = math.cos(angle3)
rotZ[1,1] = math.cos(angle3)
rotZ[0,1] = -math.sin(angle3)
rotZ[1,0] = math.sin(angle3)
rotZ[2,2] = 1
#total rotation matrix
rot = dot(rotX,dot(rotY,rotZ))
#translation matrix
tran = random.rand(1,3) / 3

#translated & rotated set of points
new_points = dot(rot,points.T)  + tile(tran, (points.shape[0], 1)).T
new_points = new_points.T
error = 1
res = ICP_3D(points,new_points, error)
x = points.T[0]
y = points.T[1]
z = points.T[2]
res_x = res.T[0]
res_y = res.T[1]
res_z = res.T[2]
x2 = new_points.T[0]
y2 = new_points.T[1]
z2 = new_points.T[2]


from mayavi.mlab import *
points3d(x, y, z, color=(0, 0, 1), scale_factor=.02 )
points3d(res_x, res_y, res_z, color=(0, 1, 0), scale_factor=.02 )
points3d(x2, y2, z2, color=(1, 0, 0), scale_factor=.02)
show()

# <codecell>

