# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'habitat.ui'
#
# Created: Wed Jun 17 23:24:21 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(797, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 161, 561))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.treeWidget = QtGui.QTreeWidget(self.verticalLayoutWidget)
        self.treeWidget.setObjectName(_fromUtf8("treeWidget"))
        self.verticalLayout.addWidget(self.treeWidget)
        self.mainGraphicsView = QtGui.QGraphicsView(self.centralwidget)
        self.mainGraphicsView.setGeometry(QtCore.QRect(160, 0, 641, 551))
        self.mainGraphicsView.setObjectName(_fromUtf8("mainGraphicsView"))
        self.verticalLayoutWidget_2 = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(179, 60, 281, 281))
        self.verticalLayoutWidget_2.setObjectName(_fromUtf8("verticalLayoutWidget_2"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.devicesListView = QtGui.QListView(self.verticalLayoutWidget_2)
        self.devicesListView.setObjectName(_fromUtf8("devicesListView"))
        self.verticalLayout_2.addWidget(self.devicesListView)
        self.addBranchDevices = QtGui.QPushButton(self.verticalLayoutWidget_2)
        self.addBranchDevices.setObjectName(_fromUtf8("addBranchDevices"))
        self.verticalLayout_2.addWidget(self.addBranchDevices)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 797, 23))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.actionAddDevice = QtGui.QAction(MainWindow)
        self.actionAddDevice.setObjectName(_fromUtf8("actionAddDevice"))
        self.actionCreate_Branch = QtGui.QAction(MainWindow)
        self.actionCreate_Branch.setObjectName(_fromUtf8("actionCreate_Branch"))
        self.menuFile.addAction(self.actionAddDevice)
        self.menuFile.addAction(self.actionCreate_Branch)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Habitat Monitor", None))
        self.treeWidget.headerItem().setText(0, _translate("MainWindow", "Data Source", None))
        self.addBranchDevices.setText(_translate("MainWindow", "Add Devices", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.actionAddDevice.setText(_translate("MainWindow", "Add Device", None))
        self.actionCreate_Branch.setText(_translate("MainWindow", "Create Branch", None))

