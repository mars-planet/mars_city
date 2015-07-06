import sys
import numpy as np
import threading
import re
from pymongo import MongoClient
from PyQt4 import QtCore, QtGui
from habitat import Ui_MainWindow
from PyTango import DeviceProxy


class HabitatMonitor(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        client = MongoClient('localhost', 27017)
        self.db = client.habitatdb
        self.threads = []
        self.nodeTimers = []
        self.isModified = False
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.tabWidget.currentChanged.connect(self.tab_changed)
        self.ui.actionAddDevice.triggered.connect(self.add_device)
        self.ui.actionCreate_Branch.triggered.connect(self.create_branch)
        self.ui.actionModify_Summary.triggered.connect(self.modify_summary)
        self.ui.actionModify_Summary.setEnabled(False)
        self.ui.actionDelete_Node.triggered.connect(self.delete_node)
        self.ui.actionDelete_Node.setEnabled(False)
        self.dataSourcesTreeItem = QtGui.QTreeWidgetItem(self.ui.treeWidget)
        self.dataSourcesTreeItem.setText(0, "Data Sources")
        nodes = self.db.nodes
        node = nodes[0]
        for i in nodes.find({'type': 'leaf'}):
            device = i['name']
            proxy = DeviceProxy(device)
            deviceFlag = 1
            while deviceFlag:
                try:
                    proxy.ping()
                    deviceFlag = 0
                    t = threading.Thread(target=self.aggregate_data,args=([device]))
                    t.start()
                    self.update_tree(self.dataSourcesTreeItem, device)
                except Exception as ex:
                    QtGui.QMessageBox.critical(self, "Warning",
                            "Start the device server " + device)

        for i in nodes.find({'type': 'branch'}):
            device = i['name']
            treeBranch = QtGui.QTreeWidgetItem(self.ui.treeWidget)
            treeBranch.setText(0, device)
            for j in i['children']:
                self.update_tree(treeBranch, j)
            t = threading.Thread(target=self.aggregate_branch_data,args=([device]))
            t.start()
        self.ui.treeWidget.connect(self.ui.treeWidget, 
            QtCore.SIGNAL('itemClicked(QTreeWidgetItem*, int)'),
            self.onClick)
        self.ui.addBranchDevices.clicked.connect(self.finalize_branch)
        self.ui.functionButton.clicked.connect(self.add_summary)
        self.ui.graphButton.clicked.connect(self.show_graph)
        self.ui.graphButton.hide()
        self.ui.devicesListView.hide()
        self.ui.mainGraphicsView.hide()
        self.ui.addBranchDevices.hide()
        self.ui.groupBox.hide()
        self.ui.tabWidget.hide()
        self.ui.listWidget.hide()
        self.ui.listWidget_2.hide()


    def delete_node(self):
        nodes = self.db.nodes
        nodes.remove({'name': self.modifiedNode})
        nodes.update({'children': self.modifiedNode},
            {'$pull': {'children': self.modifiedNode}},
            upsert=False, multi=True)


    def modify_summary(self):
        nodes = self.db.nodes
        node = nodes.find_one({'name': self.modifiedNode})
        if node['type'] == 'branch':
            self.ui.timeLabel.hide()
            self.ui.timeLineEdit.hide()
            self.ui.minutesLabel.hide()
        elif node['type'] == 'leaf':
            self.ui.timeLabel.show()
            self.ui.timeLineEdit.show()
            self.ui.minutesLabel.show()
        self.ui.functionButton.setText('Modify Summary')
        self.isModified = True
        self.ui.tabWidget.hide()
        self.ui.graphButton.hide()
        self.ui.groupBox.show()


    def show_graph(self):
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

    def tab_changed(self):
        pass


    def fetch_data(self, devName):
        proxy = DeviceProxy(devName)
        temp = proxy.temperature
        db = self.db
        nodes = db.nodes
        node = nodes.find_one({'name': devName})
        max_len = node['max_len']
        if len(list(node['data'])) >= max_len:
            nodes.update({'name': devName}, {'$pop': {'data': -1}})
        nodes.update({'name': devName}, {'$push': {'data': temp}})
        node = nodes.find_one({'name': devName})
        summary_data = self.find_summary(list(node['data']), node['function'])
        nodes.update({'name': devName}, {'$set': {'summary_data': summary_data}})
        threading.Timer(3, self.fetch_data, [devName]).start()


    def fetch_branch_data(self, branchName):
        nodes = self.db.nodes
        node = nodes.find_one({'name': branchName})
        childNodes = node['children']
        raw_data = {}
        for i in childNodes:
            child = nodes.find_one({'name': i})
            childSummary = child['summary_data']
            raw_data[i] = childSummary
        summary_data = self.find_summary(raw_data.values(), node['function'])
        updated = nodes.update({'name': branchName}, 
            {'$set': {'data': raw_data, 'summary_data': summary_data}})
        threading.Timer(3, self.fetch_branch_data, [branchName]).start()


    def aggregate_data(self, devName):
        timer = threading.Timer(3, self.fetch_data, [devName])
        timer.start()


    def aggregate_branch_data(self, branchName):
        timer = threading.Timer(3, self.fetch_branch_data, [branchName])
        timer.start()


    def add_summary(self):
        summaryTime = ""
        max_len = 0
        children = ""
        nodes = self.db.nodes
        for radioButton in self.ui.verticalLayoutWidget_3.findChildren(QtGui.QRadioButton):
            if radioButton.isChecked():
                summary = str(radioButton.text())
                break

        pattern = re.compile("^[0-9][0-9]:[0-9][0-9]:[0-9][0-9]$")

        if self.isModified == True:
            modNode = nodes.find_one({'name': self.modifiedNode})
            if modNode['type'] == "leaf":
                timeField = str(self.ui.timeLineEdit.text())
                if len(timeField) == 0:
                    QtGui.QErrorMessage(self).showMessage("Time Field is required")
                    return
                if not pattern.match(timeField):
                    QtGui.QErrorMessage(self).showMessage(
                        "Please enter time in the correct format -- hh:mm:ss")
                    return
                l = timeField.split(":")
                summaryTime = int(l[0]) * 3600 + int(l[1]) * 60 + int(l[2])
                max_len = (summaryTime * 60) / 2
                nodes.update({'name': self.modifiedNode}, 
                    {'$set': {'max_len': max_len, 'function': summary}})
            elif modNode['type'] == "branch":
                nodes.update({'name': self.modifiedNode}, 
                    {'$set': {'function': summary}})
            self.isModified = False
            self.ui.groupBox.hide()
            return

        sourceType = self.sourceType
        self.ui.minButton.setChecked(True)
        if sourceType == "leaf":
            timeField = str(self.ui.timeLineEdit.text())
            if len(timeField) == 0:
                QtGui.QErrorMessage(self).showMessage("Time Field is required")
                return
            if not pattern.match(timeField):
                QtGui.QErrorMessage(self).showMessage(
                    "Please enter time in the correct format -- hh:mm:ss")
                return
            l = timeField.split(":")
            summaryTime = int(l[0]) * 3600 + int(l[1]) * 60 + int(l[2])
            max_len = (summaryTime * 60) / 2
            nodeName = self.devName
        elif sourceType == "branch":
            children = self.branchChildren
            nodeName = self.branchName
        node = {'name': nodeName,
                'type': sourceType,
                'function': summary,
                'time': summaryTime,
                'children': children,
                'max_len': max_len,
                'data': [],
                'summary_data': 0.0
                }
        node_id = nodes.insert_one(node).inserted_id
        if sourceType == "leaf":
            t = threading.Thread(target=self.aggregate_data,args=([self.devName]))
            t.start()

        if sourceType == "branch":
            t = threading.Thread(target=self.aggregate_branch_data,
                args=([self.branchName]))
            t.start()
        self.ui.groupBox.hide()


    def finalize_branch(self):
        model = self.ui.devicesListView.model()
        self.branchChildren = []
        for i in range(model.rowCount()):
            item = model.item(i)
            if item.checkState() == QtCore.Qt.Checked:
                self.branchChildren.append(str(item.text()))
                self.update_tree(self.treeBranch, item.text())
        self.ui.timeLabel.hide()
        self.ui.timeLineEdit.hide()
        self.ui.minutesLabel.hide()
        self.ui.devicesListView.hide()
        self.ui.addBranchDevices.hide()
        self.ui.groupBox.show()


    def create_branch(self):
        self.ui.tabWidget.hide()
        self.ui.graphButton.hide()
        self.ui.functionButton.setText("Add Summary")
        self.sourceType = "branch"
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog', 
            'Enter Branch Name:')
        text = str(text)
        self.branchName = str(text)
        if ok:
            self.treeBranch = QtGui.QTreeWidgetItem(self.ui.treeWidget)
            self.treeBranch.setText(0, text)
            nodes = self.db.nodes
            model = QtGui.QStandardItemModel()
            for i in nodes.find():
                device = i['name']
                item = QtGui.QStandardItem(device)
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
            temp = self.proxy.temperature
            if len(self.data) >= 60:
                self.data.pop(0)
            self.data.append(temp)
            self.curve.setData(self.data[:])
        finally:
            QtCore.QTimer.singleShot(300, self.update_plot)


    def find_summary(self, data, function):
        if function == "Minimum":
            return min(data)
        elif function == "Maximum":
            return max(data)
        elif function == "Average":
            return float(sum(data)) / len(data)


    def update_nodedata(self):
        node = self.db.nodes.find_one({'name': self.currentNode})
        value = node['data'][-1]
        self.ui.attributeValue.setText(str(value))
        self.ui.summaryValue.setText(str(node['summary_data']))


    def update_branchdata(self):
        try:
            nodes = self.db.nodes
            node = nodes.find_one({'name': self.branchNode})
            c = 0
            data = node['data']
            self.ui.listWidget.clear()
            self.ui.listWidget_2.clear()
            keys = data.keys()
            values = map(str, data.values())
            self.ui.listWidget.addItems(keys)
            self.ui.listWidget_2.addItems(values)
            self.ui.summaryLabel.setText(node["function"] + " Value: ")
            self.ui.summaryValue.setText(str(node['summary_data']))
        except AttributeError:
            pass


    def onClick(self, item, column):
        for i in self.nodeTimers:
            i.stop()
        self.ui.graphButton.show()
        currentNode = str(item.text(column))
        if currentNode == "Data Sources":
            self.ui.actionModify_Summary.setEnabled(False)
            self.ui.actionDelete_Node.setEnabled(False)
            return
        self.ui.actionModify_Summary.setEnabled(True)
        self.ui.actionDelete_Node.setEnabled(True)
        self.modifiedNode = currentNode
        self.isModified = False
        nodes = self.db.nodes
        node = nodes.find_one({'name': currentNode})
        if node['type'] == "leaf":
            self.ui.listWidget.hide()
            self.ui.listWidget_2.hide()
            self.currentNode = currentNode
            self.itemText = currentNode
            self.ui.attributeName.setText("Temperature: ")
            self.ui.summaryLabel.setText(node['function'] + "Temperature:")
            self.ui.tabWidget.show()
            timer = QtCore.QTimer()
            self.nodeTimers.append(timer)
            timer.timeout.connect(self.update_nodedata)
            timer.start(5000)

        else:
            self.branchNode = node['name']
            self.ui.tabWidget.show()
            self.ui.listWidget.show()
            self.ui.listWidget_2.show()
            timer = QtCore.QTimer()
            self.nodeTimers.append(timer)
            timer.timeout.connect(self.update_branchdata)
            timer.start(5000)

    def add_device(self):
        self.ui.tabWidget.hide()
        self.ui.graphButton.hide()
        self.ui.functionButton.setText("Add Summary")
        print "Add new device"
        devName, ok = QtGui.QInputDialog.getText(self, 'Input Dialog', 
            'Enter Device Address:')
        devName = str(devName)
        self.devName = devName
        nodes = self.db.nodes
        if nodes.find_one({'name': devName}) != None:
            return
        if ok:
            try:
                self.sourceType = "leaf"
                self.proxy = DeviceProxy(devName)
                msgBox = QtGui.QMessageBox()
                msgBox.setText('Device added successfully')
                msgBox.addButton(QtGui.QPushButton('Ok'), 
                    QtGui.QMessageBox.YesRole)
                ret = msgBox.exec_()
                self.update_tree(self.dataSourcesTreeItem, devName)
                dev_attrs = self.proxy.get_attribute_list()
                self.ui.groupBox.show()
                self.ui.timeLabel.show()
                self.ui.timeLineEdit.show()
                self.ui.minutesLabel.show()

            except Exception as ex:
                print ex
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