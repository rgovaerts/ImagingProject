from pyface.qt import QtCore, QtGui
from pyface.qt.QtGui import QApplication
import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import sys

from FullProjectRegistration import registrationAlgorithm


class Ui_MainWindow(object):
    def __init__(self):
        self.myRegistration = registrationAlgorithm(0.015, False, False, False)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")



        self.centralWidget = QtGui.QWidget(MainWindow)
        horiz_box_layout = QtGui.QHBoxLayout(self.centralWidget)
        vert_box_layout = QtGui.QVBoxLayout(self.centralWidget)
        horiz_box_layout.addLayout(vert_box_layout)
        self.vtkWidget = QVTKRenderWindowInteractor(self.centralWidget)
        horiz_box_layout.addWidget(self.vtkWidget)

        label_data_path = QtGui.QLabel(self.centralWidget)
        label_data_path.setText("Data : ")
        label_data_path.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(label_data_path)

        button_Data = QtGui.QPushButton(self.centralWidget)
        button_Data.setText("Input Data")
        vert_box_layout.addWidget(button_Data)

        self.demo_data = QtGui.QCheckBox(self.centralWidget)
        self.demo_data.setText("Use Demo data")
        self.demo_data.setCheckState(QtCore.Qt.Checked)
        vert_box_layout.addWidget(self.demo_data)

        self.noise_checkBox = QtGui.QCheckBox(self.centralWidget)
        self.noise_checkBox.setText("Add noise")
        vert_box_layout.addWidget(self.noise_checkBox)

        self.outlier_checkBox = QtGui.QCheckBox(self.centralWidget)
        self.outlier_checkBox.setText("Add Outliers")
        vert_box_layout.addWidget(self.outlier_checkBox)

        self.outlier_removal = QtGui.QCheckBox(self.centralWidget)
        self.outlier_removal.setText("Remove Outliers")
        vert_box_layout.addWidget(self.outlier_removal)

        label_outlier_removal = QtGui.QLabel(self.centralWidget)
        label_outlier_removal.setWordWrap(True);
        label_outlier_removal.setText("Remove Outliers : removes point if distance to nearest neighbor is above "
                                      "given % of median (this can slow down the framerate")
        vert_box_layout.addWidget(label_outlier_removal)

        self.outlier_removal_spinbox = QtGui.QSpinBox(self.centralWidget)
        self.outlier_removal_spinbox.setMinimum(0)
        self.outlier_removal_spinbox.setMaximum(100)
        self.outlier_removal_spinbox.setSingleStep(1)
        self.outlier_removal_spinbox.setValue(50)
        self.outlier_removal_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(self.outlier_removal_spinbox)



        label_surface_reconstruction = QtGui.QLabel(self.centralWidget)
        label_surface_reconstruction.setText("Surface reconstruction : ")
        label_surface_reconstruction.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(label_surface_reconstruction)

        label_tolerance = QtGui.QLabel(self.centralWidget)
        label_tolerance.setText("Subsampling tolerance for surface reconstruction : ")
        label_tolerance.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(label_tolerance)

        self.tolerance_spinbox = QtGui.QDoubleSpinBox(self.centralWidget)
        self.tolerance_spinbox.setDecimals(3)
        self.tolerance_spinbox.setMinimum(0.001)
        self.tolerance_spinbox.setSingleStep(0.001)
        self.tolerance_spinbox.setValue(0.015)
        self.tolerance_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(self.tolerance_spinbox)

        self.button_Laplacian = QtGui.QPushButton(self.centralWidget)
        self.button_Laplacian.setText("Laplacian")
        vert_box_layout.addWidget(self.button_Laplacian)



        self.button_Delaunay = QtGui.QPushButton(self.centralWidget)
        self.button_Delaunay.setText("Delaunay")
        vert_box_layout.addWidget(self.button_Delaunay)

        label_delaunay_paramters = QtGui.QLabel(self.centralWidget)
        label_delaunay_paramters.setText("Delaunay alpha parameter : ")
        label_delaunay_paramters.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(label_delaunay_paramters)

        self.alpha_value_spinbox = QtGui.QDoubleSpinBox(self.centralWidget)
        self.alpha_value_spinbox.setDecimals(3)
        self.alpha_value_spinbox.setMinimum(0.001)
        self.alpha_value_spinbox.setSingleStep(0.001)
        self.alpha_value_spinbox.setValue(0.415)
        self.alpha_value_spinbox.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(self.alpha_value_spinbox)

        self.button_subsample = QtGui.QPushButton(self.centralWidget)
        self.button_subsample.setText("View Subsampled Point Cloud")
        vert_box_layout.addWidget(self.button_subsample)

        self.button_subsample.clicked.connect(self.viewSubsample)

        label_rt_registration = QtGui.QLabel(self.centralWidget)
        label_rt_registration.setText("Real time registration : ")
        label_rt_registration.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        vert_box_layout.addWidget(label_rt_registration)

        self.button_start = QtGui.QPushButton(self.centralWidget)
        self.button_start.setText("Start")
        vert_box_layout.addWidget(self.button_start)

        self.button_start.clicked.connect(self.startIncrementalRegistration)

        button_stop = QtGui.QPushButton(self.centralWidget)
        button_stop.setText("Stop")
        vert_box_layout.addWidget(button_stop)

        button_clear = QtGui.QPushButton(self.centralWidget)
        button_clear.setText("Clear")
        vert_box_layout.addWidget(button_clear)

        button_clear.clicked.connect(self.clearRenderWindow)

        self.save_name = QtGui.QLineEdit("untitled")
        vert_box_layout.addWidget(self.save_name)

        button_save = QtGui.QPushButton(self.centralWidget)
        button_save.setText("Save")
        vert_box_layout.addWidget(button_save)

        button_save.clicked.connect(self.Save2PLY)

        MainWindow.setCentralWidget(self.centralWidget)

    def startIncrementalRegistration(self):
        self.button_start.setEnabled(False)

        self.myRegistration.demo = self.demo_data.checkState() == QtCore.Qt.Checked
        self.myRegistration.tolerance = self.tolerance_spinbox.value()
        self.myRegistration.bool_remove = self.outlier_removal.checkState() == QtCore.Qt.Checked
        self.myRegistration.bool_noise = self.noise_checkBox.checkState() == QtCore.Qt.Checked
        self.myRegistration.bool_outliers = self.outlier_checkBox.checkState() == QtCore.Qt.Checked
        self.myRegistration.startIncrementalRegistration(self.vtkWidget.GetRenderWindow())

        self.button_start.setEnabled(True)
        self.button_Laplacian.clicked.connect(self.viewLaplacianSurface)
        self.button_Delaunay.clicked.connect(self.viewDelaunaySurface)

    def clearRenderWindow(self):
        self.vtkWidget.GetRenderWindow().GetRenderers().GetFirstRenderer().RemoveAllViewProps()
        self.myRegistration.vFullPointCloudPolyData.Reset()
    def viewLaplacianSurface(self):
        self.myRegistration.tolerance = self.tolerance_spinbox.value()
        self.myRegistration.laplacianSurface(self.vtkWidget.GetRenderWindow())
    def viewDelaunaySurface(self):
        self.myRegistration.tolerance = self.tolerance_spinbox.value()
        self.myRegistration.delaunaySurface(self.vtkWidget.GetRenderWindow(), self.alpha_value_spinbox.value())
    def viewSubsample(self):
        self.myRegistration.tolerance = self.tolerance_spinbox.value()
        self.myRegistration.subsamplePointCloud(self.vtkWidget.GetRenderWindow())
    def Save2PLY(self):
        self.myRegistration.save2PLY(self.save_name.text())

 
class SimpleView(QtGui.QMainWindow):
 
    def __init__(self, parent = None):
        QtGui.QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ren = vtk.vtkRenderer()
        self.ui.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.ui.vtkWidget.GetRenderWindow().GetInteractor()



if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = SimpleView()
    window.show()
    window.iren.Initialize() # Need this line to actually show the render inside Qt

    sys.exit(app.exec_())

