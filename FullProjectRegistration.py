__author__ = 'remy'

from vtk import *
from scipy import spatial
import math
import numpy as np
import time

class registrationAlgorithm():
    def __init__(self, tol, rem, noi, outl):
        self.demo = True
        self.tolerance = tol
        self.bool_remove = rem
        self.bool_noise = noi
        self.bool_outliers = outl
        self.vFullPointCloudPolyData = vtk.vtkPolyData()
        self.running = False
        self.actor = vtk.vtkActor()

    # ============ create points ==============

    def genPointCloud(self,i, sX, sY):

        count = sX * sY

        x = np.tile(np.linspace(-3.5 + i, 1.5 + i, sX) , sY)
        y = np.linspace(-3.5 + i, 1.5 + i, count)
        z = 5*np.sin(np.sqrt(np.square(x)+np.square(y)))/np.sqrt(np.square(x)+np.square(y))

        PointCloud = np.zeros((count,3))
        PointCloud[:,0] = x
        PointCloud[:,1] = y
        PointCloud[:,2] = z

        return  PointCloud

    # ============ rot+tran of points ==============

    def induceRandomShift(self,pointCloud):
        #random angles for rotation matrix
        angle1 = np.random.rand() / 2
        angle2 = np.random.rand() / 2
        angle3 = np.random.rand() / 2
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
        tran = np.random.rand(1,3) + 2

        #translated & rotated set of points
        shiftedPoints = np.dot(rot,pointCloud.T)  + np.tile(tran, (pointCloud.shape[0], 1)).T
        shiftedPoints = shiftedPoints.T
        return shiftedPoints


    # ============ Noise and outliers==============

    def addNoise(self,pointCloud):
        count = np.shape(pointCloud)[0]
        noise = np.random.rand(count,3) / 2
        return pointCloud + noise

    def addOutliers(self,pointCloud):
        count = np.shape(pointCloud)[0]
        outliers = np.zeros((count,3))
        noise = np.random.rand(count,3) / 2
        for i in range(count):
            if noise[i,0]<0.002:
                outliers[i,2] = np.random.rand() * 10
        return pointCloud + outliers


    # ============ change to vtkPolyData ==============

    def change2VTKPolyData(self,pointCloud):
        count = np.shape(pointCloud)[0]
        vPoints = vtk.vtkPoints()
        vVertices = vtk.vtkCellArray()

        for index in range(count):
            id = vPoints.InsertNextPoint(pointCloud[index])
            vVertices.InsertNextCell(1)
            vVertices.InsertCellPoint(id)

        vPolyData = vtk.vtkPolyData()
        vPolyData.SetPoints(vPoints)
        vPolyData.SetVerts(vVertices)
        if vtk.VTK_MAJOR_VERSION <= 5:
            vPolyData.Update()

        return vPolyData

    # ============ run ICP ==============

    def runICP(self,source, target):
        icp = vtk.vtkIterativeClosestPointTransform()
        icp.SetSource(source)
        icp.SetTarget(target)
        icp.GetLandmarkTransform().SetModeToRigidBody()
        #icp.DebugOn()
        icp.SetMaximumNumberOfIterations(20)
        icp.StartByMatchingCentroidsOn()
        icp.Modified()
        icp.Update()

        icpTransformFilter = vtk.vtkTransformPolyDataFilter()
        if vtk.VTK_MAJOR_VERSION <= 5:
            icpTransformFilter.SetInput(source)
        else:
            icpTransformFilter.SetInputData(target)

        icpTransformFilter.SetTransform(icp)
        icpTransformFilter.Update()

        VTKtransformedSource = icpTransformFilter.GetOutput()

        return  VTKtransformedSource

    # ============ remove outliers ==============


    def removeOutliers(self,vPointCloud):
    # compute the median (less outlier sensitive than the mean!) of all the distances of the nearest
    # neighbor for each point of the point cloud and reject the points with distances over the 90% of the population (should
    # hopefully do the trick)
    # this is very slow though ... -_- and only removes single point outliers

        count = vPointCloud.GetNumberOfPoints()
        points = np.zeros((count,3))
        for index in range(count):
            vPointCloud.GetPoints().GetPoint(index, points[index])
        distance = np.zeros(np.shape(points)[0])
        for i in range(np.shape(points)[0]-1):
            slicedCopy = np.append(points[:i], points[i+1:], axis=0)
            sourceTree = spatial.cKDTree(slicedCopy)
            distance[i] = sourceTree.query(points[i])[0]
        return points[distance < np.median(distance) + np.median(distance) * 0.5]

    # ============ incremental registration ==============

    def startIncrementalRegistration(self, renderWindow):

        self.running = True
        self.vFullPointCloudPolyData.Reset()
        sizeX = 50
        sizeY = 50

        startingPointCloud = self.genPointCloud(0,sizeX,sizeX)

        #vFullPointCloudPolyData = vtk.vtkPolyData()

        vSourcePolyData = self.change2VTKPolyData(startingPointCloud)

        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(self.vFullPointCloudPolyData)
        else:
            mapper.SetInputData(self.vFullPointCloudPolyData)

        mapper.ScalarVisibilityOff()

        renWin = renderWindow
        ren = renderWindow.GetRenderers().GetFirstRenderer()
        ren.AddActor(self.actor)
        ren.ResetCamera()

        for i in range(0, 30, 1):
            if self.demo:
                target = self.genPointCloud(i*0.51,sizeX,sizeY)
            else:
                """
                target = GET INPUT DATA
                """
            if self.bool_noise :
                target = self.addNoise(target)
            if self.bool_outliers :
                target = self.addOutliers(target)
            vTargetPolyData = self.change2VTKPolyData(target)
            vTransformedSourcePolyData = self.runICP(vTargetPolyData, vSourcePolyData)

            if self.bool_remove :
                remOutlierPointCloud = self.change2VTKPolyData(self.removeOutliers(vTransformedSourcePolyData))
            else :
                remOutlierPointCloud = vTransformedSourcePolyData

            appendPoly = vtk.vtkAppendPolyData()
            appendPoly.AddInput(remOutlierPointCloud)
            appendPoly.AddInput(self.vFullPointCloudPolyData)
            appendPoly.Update()
            self.vFullPointCloudPolyData = appendPoly.GetOutput()

            """
            #this is the part that slow the algorithm down as the point cloud grows but it is necessary
            #this could to be sent to the GPU
            cleanFullPolyData = vtk.vtkCleanPolyData()
            cleanFullPolyData.SetInput(vFullPointCloudPolyData)
            cleanFullPolyData.SetTolerance(tolerance)
            cleanFullPolyData.Update()

            self.vFullPointCloudPolyData = cleanFullPolyData.GetOutput()
            """
            vSourcePolyData = vTargetPolyData

            if vtk.VTK_MAJOR_VERSION <= 5:
                mapper.SetInput(self.vFullPointCloudPolyData)
            else:
                mapper.SetInputData(self.vFullPointCloudPolyData)

            mapper.ScalarVisibilityOff()

            self.actor.SetMapper(mapper)
            self.actor.GetProperty().SetColor(1, 1, 1)

            ren.RemoveAllViewProps()
            ren.AddActor(self.actor)
            ren.GetActiveCamera().SetPosition(10, -20, 20)
            ren.ResetCamera()
            renWin.Render()

        self.running = False

    def laplacianSurface(self, renderWindow):
        # ============ Surface reconstruction ==============
        cleanFullPolyData = vtk.vtkCleanPolyData()
        cleanFullPolyData.SetInput(self.vFullPointCloudPolyData)
        cleanFullPolyData.SetTolerance(self.tolerance)
        cleanFullPolyData.Update()

        # Construct the surface and create isosurface.
        surf = vtk.vtkSurfaceReconstructionFilter()
        surf.SetInputConnection(cleanFullPolyData.GetOutputPort())
        #surf.SetInput(self.vFullPointCloudPolyData)

        cf = vtk.vtkContourFilter()
        cf.SetInputConnection(surf.GetOutputPort())

        # Sometimes the contouring algorithm can create a volume whose gradient
        # vector and ordering of polygon (using the right hand rule) are
        # inconsistent. vtkReverseSense cures this problem.
        reverse = vtk.vtkReverseSense()
        reverse.SetInputConnection(cf.GetOutputPort())
        reverse.ReverseCellsOn()
        reverse.ReverseNormalsOn()

        smooth_loop = vtk.vtkLoopSubdivisionFilter()
        smooth_loop.SetNumberOfSubdivisions(3)
        smooth_loop.SetInputConnection(reverse.GetOutputPort())

        surfaceMapper = vtk.vtkPolyDataMapper()
        surfaceMapper.SetInputConnection(smooth_loop.GetOutputPort())
        surfaceMapper.ScalarVisibilityOff()

        surfaceActor = vtk.vtkActor()
        surfaceActor.SetMapper(surfaceMapper)
        surfaceActor.GetProperty().SetDiffuseColor(1.0000, 0.3882, 0.2784)
        surfaceActor.GetProperty().SetSpecularColor(1, 1, 1)
        surfaceActor.GetProperty().SetSpecular(.4)
        surfaceActor.GetProperty().SetSpecularPower(50)


        renWin = renderWindow
        ren = renderWindow.GetRenderers().GetFirstRenderer()#vtk.vtkRenderer()
        ren.RemoveAllViewProps()
        ren.AddActor(surfaceActor)
        ren.ResetCamera()
        renWin.Render()

    def delaunaySurface(self, renderWindow, alpha):
    # ========== Delaunay ==========
        cleanFullPolyData = vtk.vtkCleanPolyData()
        cleanFullPolyData.SetInput(self.vFullPointCloudPolyData)
        cleanFullPolyData.SetTolerance(self.tolerance)
        cleanFullPolyData.Update()

        delny = vtk.vtkDelaunay3D()
        delny.SetInputConnection(cleanFullPolyData.GetOutputPort())
        delny.SetTolerance(self.tolerance)
        delny.SetAlpha(alpha)
        delny.Update()

        delaunaySurfaceMapper = vtk.vtkDataSetMapper()
        delaunaySurfaceMapper.SetInputConnection(delny.GetOutputPort())

        delaunayActor = vtk.vtkActor()
        delaunayActor.SetMapper(delaunaySurfaceMapper)
        delaunayActor.GetProperty().SetColor(1, 1, 1)

        renWin = renderWindow
        ren = renderWindow.GetRenderers().GetFirstRenderer()#vtk.vtkRenderer()
        ren.RemoveAllViewProps()
        ren.AddActor(delaunayActor)
        ren.ResetCamera()
        renWin.Render()

    def subsamplePointCloud(self, renderWindow):

        cleanFullPolyData = vtk.vtkCleanPolyData()
        cleanFullPolyData.SetInput(self.vFullPointCloudPolyData)
        cleanFullPolyData.SetTolerance(self.tolerance)
        cleanFullPolyData.Update()

        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
                mapper.SetInput(cleanFullPolyData.GetOutput())
        else:
            mapper.SetInputData(cleanFullPolyData.GetOutput())

        mapper.ScalarVisibilityOff()

        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetColor(1, 1, 1)

        renWin = renderWindow
        ren = renderWindow.GetRenderers().GetFirstRenderer()#vtk.vtkRenderer()
        ren.RemoveAllViewProps()
        ren.AddActor(self.actor)
        ren.ResetCamera()
        renWin.Render()

    def save2PLY(self, filename):

        plyWriter = vtk.vtkPLYWriter()
        plyWriter.SetFileName(str(filename))
        if vtk.VTK_MAJOR_VERSION <= 5:
            plyWriter.SetInput(self.vFullPointCloudPolyData)
        else:
            plyWriter.SetInputData(self.vFullPointCloudPolyData)
        plyWriter.Write();

