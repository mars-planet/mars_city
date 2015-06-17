import sys
import numpy as np
from PyQt4 import QtCore, QtGui
from habitat import Ui_MainWindow
from random import randint
from PyTango import DeviceProxy


class HabitatMonitor(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.actionAddDevice.triggered.connect(self.add_device)
        self.ui.actionCreate_Branch.triggered.connect(self.create_branch)
        self.dataSourcesTreeItem = QtGui.QTreeWidgetItem(self.ui.treeWidget)
        self.dataSourcesTreeItem.setText(0, "Data Sources")
        with open("devices.txt", "r+") as f:
            devices = f.read().strip().split("\n")
            for i in devices:
                self.update_tree(self.dataSourcesTreeItem, i)
        self.ui.treeWidget.connect(self.ui.treeWidget, 
            QtCore.SIGNAL('itemClicked(QTreeWidgetItem*, int)'),
            self.onClick)
        self.ui.addBranchDevices.clicked.connect(self.finalize_branch)
        self.ui.devicesListView.hide()
        self.ui.mainGraphicsView.hide()
        self.ui.addBranchDevices.hide()


    def finalize_branch(self):
        model = self.ui.devicesListView.model()
        for i in range(model.rowCount()):
            item = model.item(i)

            if item.checkState() == QtCore.Qt.Checked:
                self.update_tree(self.treeBranch, item.text())
        self.ui.devicesListView.hide()
        self.ui.addBranchDevices.hide()


    def create_branch(self):
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog', 
            'Enter Branch Name:')
        text = str(text)
        if ok:
            self.treeBranch = QtGui.QTreeWidgetItem(self.ui.treeWidget)
            self.treeBranch.setText(0, text)
            with open("devices.txt", "r") as f:
                devices = f.read().strip().split("\n")
            model = QtGui.QStandardItemModel()
            for i in devices:                   
                item = QtGui.QStandardItem(i)
                check = QtCore.Qt.Unchecked
                item.setCheckState(check)
                item.setCheckable(True)
                model.appendRow(item)
            self.ui.devicesListView.setModel(model)
            self.ui.devicesListView.show()
            self.ui.addBranchDevices.show()
            
            
    def update_tree(self, source, value):
        tempDataSource = QtGui.QTreeWidgetItem(source)
        tempDataSource.setText(0, value)


    def update_plot(self):
        try:
            print "updating plot"
            temp = self.proxy.temperature
            if len(self.data) >= 60:
                self.data.pop(0)
            self.data.append(temp)
            self.curve.setData(self.data[:])
        finally:
            QtCore.QTimer.singleShot(300, self.update_plot)

    def onClick(self, item, column):
        self.itemText = item.text(column)
        if self.itemText == "Data Sources":
            return
        self.ui.mainGraphicsView.show()

        import pyqtgraph as pg
        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000,600)
        self.win.setWindowTitle('pyqtgraph example: Plotting')
        pg.setConfigOptions(antialias=True)
        self.p6 = self.win.addPlot(title="Updating plot")
        self.curve = self.p6.plot(pen='y')
        self.data = []
        self.ptr = 0
        self.proxy = DeviceProxy(str(self.itemText))
        self.update_plot()

    def add_device(self):
        print "Add new device"
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog', 
            'Enter Device Address:')
        text = str(text)
        with open("devices.txt", "r+") as f:
            devices = f.read().strip().split("\n")
            if text in devices:
                return
        if ok:
            try:
                self.proxy = DeviceProxy(text)
                print self.proxy.ping()
                msgBox = QtGui.QMessageBox()
                msgBox.setText('Device added successfully')
                msgBox.addButton(QtGui.QPushButton('Ok'), 
                    QtGui.QMessageBox.YesRole)
                ret = msgBox.exec_()
                self.update_tree(self.dataSourcesTreeItem, text)
                with open("devices.txt", "a+") as f:
                    f.write(text + "\n")
                dev_attrs = self.proxy.get_attribute_list()
            except:
                print "Exceptions"
                QtGui.QErrorMessage(self).showMessage(
                    "Incorrect Device Address")
                

        else:
            QtGui.QMessageBox.critical(self, "Warning",
                            "Device not added")
        


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = HabitatMonitor()
    myapp.show()
    sys.exit(app.exec_())