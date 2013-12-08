"""
Created on 7 dec. 2013

@author: Remy
"""

import numpy as np
import math as math
from scipy.spatial import KDTree
from mayavi.mlab import *

class icp3d(object):
    """ icp class """

    def __init__(self, source_point_cloud, target_point_cloud, err, maxIt, subsam, subsam_perc, distBound):
        self.source_points = source_point_cloud
        self.target_points = target_point_cloud
        self.error = err
        self.maxIter = maxIt
        self.subsampling = subsam
        if subsam == True:
            self.subsample_percentage = subsam_perc
        else:
            self.subsample_percentage = 1
        self.distanceBound = distBound
            
    def startICP(self):
        self.res = self.iterate(self.source_points, self.target_points, 0)
        
    def iterate(self, source, target, iternum):
        """
        Returns A transformed so that it matches B as closely as possible
        
        The algorithm steps are :
        
            1.Associate points by the nearest neighbor criteria (for each point in one point cloud find the closest point in the second point cloud).
            2.Estimate transformation parameters (rotation and translation) using a mean square cost function (the transform would align best each point to its match found in the previous step).
            3.Transform the points using the estimated parameters.
            4.Iterate (re-associate the points and so on).
            
        http://en.wikipedia.org/wiki/Iterative_closest_point
        http://en.wikipedia.org/wiki/Point_Set_Registration#Iterative_closest_point
        """
        n = source.shape[0] #number of points
        
        # random subsampling % of the point cloud
        source_sample = np.zeros((int(n * self.subsample_percentage),3))
        target_sample = np.zeros((int(n * self.subsample_percentage),3))
        for i in range(int(n * self.subsample_percentage)):
            ind = np.random.randint(0,n)
            source_sample[i] = source[ind]
            target_sample[i] = target[ind]
    
        tree1 = KDTree(source_sample)
        #tree2 = KDTree(target_sample)
        # 1.Associate points by the nearest neighbor criteria
        dist,indices = tree1.query(target_sample, k=1, eps=0, p=2, distance_upper_bound= self.distanceBound)   
    
        NNcount = 0
        for i in range(np.shape(indices)[0]):
            if indices[i] != np.shape(source_sample)[0]:
                NNcount += 1
        NNsource = np.zeros((NNcount,3))
        NNtarget = np.zeros((NNcount,3))
        j = 0
        for i in range(np.shape(indices)[0]):
            if indices[i] != np.shape(source_sample)[0]:
                NNsource[j] = source_sample[indices[i]]
                NNtarget[j] = target_sample[i]
                j +=1
        # 2.Estimate transformation parameters (rotation and translation)
        ret_R, ret_t = self.rigid_transform_3D(NNsource.T ,NNtarget.T)
        # 3.Transform the points using the estimated parameters.
        A2 = np.dot(ret_R,source.T) + np.tile(ret_t, (n, 1)).T
        A2 = A2.T
        # Find the square error (MSE)
        mse = np.sum(np.square(A2 - target))/n
        print mse
        if mse < np.square(self.error) or iternum >= self.maxIter:
            print "stop"
            return A2
        else:
            # 4.Iterate
            iternum += 1
            A2 = self.iterate(A2, target, iternum)
            return A2
        
    def rigid_transform_3D(self, A, B):
        """
        returns R, t the rotation and translation matrix between A and B using singular value decomposition
        
        Estimation of transformation parameters (rotation and translation)
        http://nghiaho.com/?page_id=671
        http://nghiaho.com/uploads/code/rigid_transform_3D.py_
        last consulted 8/11/2013
        """
        assert len(A) == len(B)
        N = A.shape[1]; # total points
        centroid_A = np.mean(A, axis=1)
        centroid_B = np.mean(B, axis=1)
        # center the points
        AA = A - np.tile(centroid_A, (N, 1)).T
        BB = B - np.tile(centroid_B, (N, 1)).T
        # dot is matrix multiplication for array
        H = np.dot(AA,BB.T)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt,U.T)
        # special reflection case
        if np.linalg.det(R) < 0:
            print "Reflection detected"
            Vt[2,:] *= -1
            R = np.dot(Vt,U.T)
        t = -np.dot(R,centroid_A) + centroid_B
        return R, t
    
    def view3d(self):
        points3d(self.source_points.T[0], self.source_points.T[1], self.source_points.T[2], color=(0, 0, 1), scale_factor=.01 )
        points3d(self.res.T[0], self.res.T[1], self.res.T[2], color=(0, 1, 0), scale_factor=.01 )
        points3d(self.target_points.T[0], self.target_points.T[1], self.target_points.T[2], color=(1, 0, 0), scale_factor=.01)
        show()
"""        
if __name__ == "__main__":
    #we give ourselves a cloud point
    numPoints = 5000
    points = np.zeros((numPoints,3))
    points[:,0] = np.linspace(-1, 1, numPoints) #np.exp(np.linspace(-1, 1, numPoints))  #np.exp(np.linspace(-1, 1, numPoints))
    points[:,1] = np.random.rand(numPoints) + np.square(np.linspace(-1, 1, numPoints)) / 2 #random.rand(numPoints) #np.linspace(-1, 1, numPoints)
    points[:,2] = 0.75*np.sin(np.linspace(0, 20*np.pi, numPoints)) / 10 + np.square(np.linspace(-1, 1, numPoints)) / 2
    
    #noise = random.rand(numPoints,3) / 75
    #points = points + noise
    
    #random angles for rotation matrix
    angle1 = np.random.rand() / 12
    angle2 = np.random.rand() / 12
    angle3 = np.random.rand() / 12
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
    rot = np.dot(rotX,np.dot(rotY,rotZ))
    #translation matrix
    tran = [0.05,0.1,0.1]#random.rand(1,3) / 5
    
    #translated & rotated set of points
    new_points = np.dot(rot,points.T)  + np.tile(tran, (points.shape[0], 1)).T
    new_points = new_points.T

    #new_points = new_points + noise
    
    test = icp3d(points, new_points, 0.05, 100, True, 0.05, 0.1)
    test.startICP()
    test.view3d()       
"""                
        
