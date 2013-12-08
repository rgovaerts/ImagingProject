"""
Created on 7 dec. 2013

@author: Remy
"""

import os
import numpy as np
os.environ['ETS_TOOLKIT'] = 'qt4'
from pyface.qt import QtGui, QtCore
from traits.api import HasTraits, Instance, on_trait_change, \
    Int, Dict
from traitsui.api import View, Item
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor
from icpClass import icp3d
import math

from PySide.QtCore import *
from PySide.QtGui import *



################################################################################
#The actual visualization
class Visualization(HasTraits):
    scene = Instance(MlabSceneModel, ())

    @on_trait_change('scene.activated')
    def update_plot(self):
        # This function is called when the view is opened. We don't
        # populate the scene when the view is not yet open, as some
        # VTK features require a GLContext.

        # We can do normal mlab calls on the embedded scene.
        """
        numPoints = 5000
        points = np.zeros((numPoints,3))
        points[:,0] = np.linspace(-1, 1, numPoints) #np.exp(np.linspace(-1, 1, numPoints))  #np.exp(np.linspace(-1, 1, numPoints))
        points[:,1] = np.random.rand(numPoints) + np.square(np.linspace(-1, 1, numPoints)) / 2 #random.rand(numPoints) #np.linspace(-1, 1, numPoints)
        points[:,2] = 0.75*np.sin(np.linspace(0, 15*np.pi, numPoints)) / 10 + np.square(np.linspace(-1, 1, numPoints)) / 2
        x = points.T[0]
        y = points.T[1]
        z = points.T[2]
        
        xnew = newpoints.T[0]
        ynew = newpoints.T[1]
        znew = newpoints.T[2]
        
        xres = respoints.T[0]
        yres = respoints.T[1]
        zres = respoints.T[2]
        
        self.scene.mlab.points3d(x, y, z, color=(0, 0, 1), scale_factor=.01 )
        self.scene.mlab.points3d(xnew, ynew, znew, color=(0, 1, 0), scale_factor=.01 )
        self.scene.mlab.points3d(xres, yres, zres, color=(1, 0, 0), scale_factor=.01 )
        """

    # the layout of the dialog is created
    view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
                     height=250, width=300, show_label=False),
                resizable=True # We need this to resize with the parent widget
                )

        
################################################################################
# The QWidget containing the visualization, this is pure PyQt4 code.
class MayaviQWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        layout = QtGui.QVBoxLayout(self)
        self.visualization = Visualization()

        # If you want to debug, beware that you need to remove the Qt
        # input hook.
        #QtCore.pyqtRemoveInputHook()
        #import pdb ; pdb.set_trace()
        #QtCore.pyqtRestoreInputHook()

        # The edit_traits call will generate the widget to embed.
        self.ui = self.visualization.edit_traits(parent=self, kind='subpanel').control
        layout.addWidget(self.ui)
        self.ui.setParent(self)


def startICP():
    
    from mayavi import mlab
    mlab.clf()

    #we give ourselves a cloud point
    numPoints = npoint_spinbox.value()
    points = np.zeros((numPoints,3))
    points[:,0] = np.linspace(-1, 1, numPoints) #np.exp(np.linspace(-1, 1, numPoints))  #np.exp(np.linspace(-1, 1, numPoints))
    points[:,1] = np.random.rand(numPoints) + np.square(np.linspace(-1, 1, numPoints)) / 2 #random.rand(numPoints) #np.linspace(-1, 1, numPoints)
    points[:,2] = 0.75*np.sin(np.linspace(0, 20*np.pi, numPoints)) / 10 + np.square(np.linspace(-1, 1, numPoints)) / 2
    """
    noise = random.rand(numPoints,3) / 75
    points = points + noise
    """
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
    tran = np.random.rand(1,3) / 8
    
    #translated & rotated set of points
    newpoints = np.dot(rot,points.T)  + np.tile(tran, (points.shape[0], 1)).T
    newpoints = newpoints.T
    """
    new_points = new_points + noise
    """
    
    x = points.T[0]
    y = points.T[1]
    z = points.T[2]
    
    xnew = newpoints.T[0]
    ynew = newpoints.T[1]
    znew = newpoints.T[2]

    mlab.points3d(x, y, z, color=(0, 0, 1), scale_factor=.02 )
    mlab.points3d(xnew, ynew, znew, color=(0, 1, 0), scale_factor=.02 )
    
    error = error_spinbox.value()
    max_iteration = iteration_spinbox.value()
    distanceBound = dist_bound_spinbox.value()
    subampling = subsampling_checkbox.isChecked()
    subsampling_perc = subsampling_spinbox.value()
    
    test = icp3d(points, newpoints, error, max_iteration, subampling, subsampling_perc, distanceBound)
    test.startICP()
    respoints = test.res
    
    xres = respoints.T[0]
    yres = respoints.T[1]
    zres = respoints.T[2]
    

    
    
    mlab.points3d(xres, yres, zres, color=(1, 0, 0), scale_factor=.02 )
    
    

if __name__ == "__main__":
    app = QtGui.QApplication.instance()
    container = QtGui.QWidget()
    container.setWindowTitle("Embedding Mayavi in a PyQt4 Application")
    # define a "complex" layout to test the behaviour
    #grid_layout = QtGui.QGridLayout(container)
    horiz_box_layout = QtGui.QHBoxLayout(container)
    vert_box_layout = QtGui.QVBoxLayout(container)
    horiz_box_layout.addLayout(vert_box_layout)
    
    #Widgets to set parameters
    label_npoint = QtGui.QLabel(container)
    label_npoint.setText("Number of points")
    label_npoint.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(label_npoint)
    
    npoint_spinbox = QtGui.QSpinBox(container)
    npoint_spinbox.setMinimum(1)
    npoint_spinbox.setMaximum(10000)
    npoint_spinbox.setValue(500)
    npoint_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(npoint_spinbox)
    
    label_iter = QtGui.QLabel(container)
    label_iter.setText("Number of iterations")
    label_iter.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(label_iter)
    
    iteration_spinbox = QtGui.QSpinBox(container)
    iteration_spinbox.setMinimum(1)
    iteration_spinbox.setMaximum(200)
    iteration_spinbox.setValue(50)
    iteration_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(iteration_spinbox)
    
    label_error = QtGui.QLabel(container)
    label_error.setText("Error")
    label_error.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(label_error)
    
    error_spinbox = QtGui.QDoubleSpinBox(container)
    error_spinbox.setDecimals(4)
    error_spinbox.setMinimum(0.0001)
    error_spinbox.setSingleStep(0.001)
    error_spinbox.setValue(0.05)
    error_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(error_spinbox)
    
    label_dist_bound = QtGui.QLabel(container)
    label_dist_bound.setText("Distance bound for nearest neighbor search")
    label_dist_bound.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(label_dist_bound)
    
    dist_bound_spinbox = QtGui.QDoubleSpinBox(container)
    dist_bound_spinbox.setMinimum(0.01)
    dist_bound_spinbox.setSingleStep(0.01)
    dist_bound_spinbox.setValue(0.1)
    dist_bound_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(dist_bound_spinbox)
    
    subsampling_checkbox = QtGui.QCheckBox(container)
    subsampling_checkbox.setText("Sub-sampling")
    vert_box_layout.addWidget(subsampling_checkbox)
    
    subsampling_spinbox = QtGui.QDoubleSpinBox(container)
    subsampling_spinbox.setMinimum(0.001)
    subsampling_spinbox.setMaximum(1.00)
    subsampling_spinbox.setSingleStep(0.1)
    subsampling_spinbox.setValue(1.0)
    subsampling_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
    vert_box_layout.addWidget(subsampling_spinbox)
    
    start_button = QtGui.QPushButton(container)
    start_button.setText("Start ICP")
    vert_box_layout.addWidget(start_button)
    
    
    
    # Connect the button to the function
    start_button.clicked.connect(startICP)

    
    mayavi_widget = MayaviQWidget(container)
    
    horiz_box_layout.addWidget(mayavi_widget)
    
    
    #Widget to visualize 3d data, uses mayavi
    #mayavi_widget = MayaviQWidget(container)
    """
    for i in range(3):
        for j in range(3):
            if (i==1) and (j==1):continue
            label = QtGui.QLabel(container)
            label.setText("Your QWidget at (%d, %d)" % (i,j))
            label.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
            layout.addWidget(label, i, j)
            label_list.append(label)
    mayavi_widget = MayaviQWidget(container)

    layout.addWidget(mayavi_widget, 1, 1)
    """
    
    container.show()
    window = QtGui.QMainWindow()
    window.setWindowTitle("ICP")
    window.setCentralWidget(container)
    window.show()

    # Start the main event loop.
    app.exec_()
