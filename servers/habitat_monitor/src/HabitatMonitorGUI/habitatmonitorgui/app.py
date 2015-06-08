import sys
from PyQt4 import QtCore, QtGui
from habitat import Ui_MainWindow

from PyTango import DeviceProxy


class HabitatMonitor(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.addDeviceAction.triggered.connect(self.add_device)

    def add_device(self):
        print "Add new device"
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog', 
            'Enter Device Address:')
        text = str(text)
        
        if ok:
            try:
                self.proxy = DeviceProxy(text)
                print self.proxy.ping()
                msgBox = QtGui.QMessageBox()
                msgBox.setText('Device added successfully')
                msgBox.addButton(QtGui.QPushButton('Ok'), 
                    QtGui.QMessageBox.YesRole)
                ret = msgBox.exec_()
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