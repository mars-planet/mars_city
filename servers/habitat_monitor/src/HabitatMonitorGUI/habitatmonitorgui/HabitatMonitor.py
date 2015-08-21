import sys
import numpy as np
import threading
import re
import pyqtgraph as pg
import json
from pymongo import MongoClient
from PyQt4 import QtCore, QtGui
from habitat import Ui_MainWindow
from PyTango import DeviceProxy
from datetime import datetime as dt


class HabitatMonitor(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        client = MongoClient('localhost', 27017)
        self.db = client.habitatdb
        self.threads = []
        self.nodeTimers = []
        self.isModified = False
        self.addingSummary = False
        self.summaryNode = None
        self.graphTimer = QtCore.QTimer()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.actionAddDevice.triggered.connect(self.add_device)
        self.ui.actionCreate_Branch.triggered.connect(self.create_branch)
        self.ui.actionModify_Summary.triggered.connect(self.modify_summary)
        self.ui.actionModify_Summary.setEnabled(False)
        self.ui.actionAdd_Summary.setEnabled(False)
        self.ui.actionDelete_Summary.setEnabled(False)
        self.ui.actionDelete_Node.triggered.connect(self.delete_node)
        self.ui.actionDelete_Summary.triggered.connect(self.delete_summary)
        self.ui.actionAdd_Summary.triggered.connect(self.add_another_summary)
        self.ui.actionDelete_Node.setEnabled(False)
        self.dataSourcesTreeItem = QtGui.QTreeWidgetItem(self.ui.treeWidget)
        self.dataSourcesTreeItem.setText(0, "Data Sources")
        nodes = self.db.nodes
        self.checkedLeaves = []
        self.populate_startup_nodes()
        self.ui.treeWidget.connect(self.ui.treeWidget,
                                   QtCore.SIGNAL(
                                       'itemClicked(QTreeWidgetItem*, int)'),
                                   self.onClick)
        self.ui.addBranchDevices.clicked.connect(self.finalize_branch)
        self.ui.functionButton.clicked.connect(self.add_summary)
        self.ui.comboBox.currentIndexChanged[str].connect(
            self.combo_index_changed)
        self.ui.childrenBox.currentIndexChanged[str].connect(
            self.branch_children_changed)
        self.ui.summaryCB.currentIndexChanged[str].connect(
            self.summarycb_index_changed)
        self.ui.tabWidget.currentChanged.connect(self.current_tab_changed)
        self.ui.devicesListView.hide()
        self.ui.mainGraphicsView.hide()
        self.ui.addBranchDevices.hide()
        self.ui.groupBox.hide()
        self.ui.comboBox.hide()
        self.ui.attrLabel.hide()
        self.ui.tabWidget.hide()
        self.ui.listWidget.hide()
        self.ui.listWidget_2.hide()
        self.ui.summaryNameLE.hide()
        self.ui.summaryCB.hide()
        self.ui.summaryLW1.hide()
        self.ui.summaryLW2.hide()
        self.ui.hour_label.hide()
        self.ui.minutes_label.hide()
        self.ui.hour_value.hide()
        self.ui.minutes_value.hide()

    def populate_startup_nodes(self):
        nodes = self.db.nodes
        print dt.now(), ":", 'populating leaves'
        for i in nodes.find({'type': 'leaf'}):
            device = i['name'] + " - " + i['attr']
            if device in self.checkedLeaves:
                continue
            proxy = DeviceProxy(i['name'])
            deviceFlag = 1
            while deviceFlag:
                try:
                    proxy.ping()
                    deviceFlag = 0
                    t = threading.Thread(target=self.aggregate_data,
                                         args=([device]))
                    t.start()
                    self.update_tree(self.dataSourcesTreeItem, i['name'],
                                     i['attr'])
                    self.checkedLeaves.append(device)
                except Exception as ex:
                    QtGui.QMessageBox.critical(self, "Warning",
                                               "Start the device server " + device)
        print dt.now(), ":", 'populating branches'
        for i in nodes.find({'type': 'branch'}):
            device = i['name']
            treeBranch = QtGui.QTreeWidgetItem(self.ui.treeWidget)
            treeBranch.setText(0, device)
            for j in i['children']:
                if '-' in j:
                    temp = j.split(' - ')
                    self.update_tree(treeBranch, temp[0], temp[1])
                else:
                    self.update_tree(treeBranch, j, "")
            t = threading.Thread(
                target=self.aggregate_branch_data, args=([device]))
            t.start()

    def delete_summary(self):
        nodes = self.db.nodes
        node = nodes.find_one({'name': self.modifiedNode, 'attr': ''})
        children_no = len(node['summary_children'].keys())
        if children_no <= 1:
            QtGui.QMessageBox.critical(self, "Warning",
                                       "Number of summaries cannot be zero")
            return
        message = "Are you sure you want to delete summary %s?" % str(
            self.ui.summaryCB.currentText())
        reply = QtGui.QMessageBox.question(self, 'Message', message,
                                           QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:
            print dt.now(), ":", 'User aggrees to deleting summary'
            summary_name = str(self.ui.summaryCB.currentText())
            self.ui.summaryCB.removeItem(self.ui.summaryCB.currentIndex())
            print dt.now()
            summary_children = node['summary_children']
            del summary_children[summary_name]
            nodes.update({'name': self.modifiedNode, 'attr': ''},
                         {'$set': {'summary_children': summary_children}})
            self.ui.summaryCB.setCurrentIndex(0)
            self.update_branchdata()

    def current_tab_changed(self):
        if self.parentNode == None:
            self.ui.actionAdd_Summary.setEnabled(True)
            nodes = self.db.nodes
            node = nodes.find_one({'name': self.modifiedNode, 'attr': ''})
        else:
            self.ui.actionAdd_Summary.setEnabled(False)

    def combo_index_changed(self, text):
        self.summaryAttr = str(text)

    def branch_children_changed(self, text):
        print text
        self.init_graph()

    def summarycb_index_changed(self, text):
        self.summary_value = str(text)
        node = self.db.nodes.find_one({'name': self.currentNode, 'attr': ""})
        print "summary index changed ---", text
        print "summary index changed ---", node['summary_children']
        if self.summary_value in node['summary_children'].keys():
            print "summary index changed --- summary found"
            self.update_branchdata()

    def build_tree(self):
        nodes = self.db.nodes
        for i in nodes.find({'type': 'leaf'}):
            device = i['name']
            self.update_tree(self.dataSourcesTreeItem, device, i['attr'])

        for i in nodes.find({'type': 'branch'}):
            device = i['name']
            treeBranch = QtGui.QTreeWidgetItem(self.ui.treeWidget)
            treeBranch.setText(0, device)
            for j in i['children']:
                if '-' in j:
                    temp = j.split(' - ')
                    self.update_tree(treeBranch, temp[0], temp[1])
                else:
                    self.update_tree(treeBranch, j, "")

    def delete_node(self):
        message = "Are you sure you want to delete the node?"
        reply = QtGui.QMessageBox.question(self, 'Message', message,
                                           QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:
            nodes = self.db.nodes
            if self.parentNode == "Data Sources" or self.parentNode == None:
                mdAttr = ""
                mdNode = self.modifiedNode
                if '-' in self.modifiedNode:
                    mdNode = self.modifiedNode.split(' - ')[0]
                    mdAttr = self.modifiedNode.split(' - ')[1]
                nodes.remove({'name': mdNode, 'attr': mdAttr})
                nodes.update({'children': self.modifiedNode},
                             {'$pull': {'children': self.modifiedNode}},
                             upsert=False, multi=True)
            else:
                nodes.update(
                    {'name': self.parentNode}, {'$pull': {'children': self.modifiedNode}})
            self.ui.treeWidget.clear()
            self.dataSourcesTreeItem = QtGui.QTreeWidgetItem(
                self.ui.treeWidget)
            self.dataSourcesTreeItem.setText(0, "Data Sources")
            self.build_tree()
            root = self.ui.treeWidget.invisibleRootItem()
            try:
                item = root.child(0)
                item = item.child(0)
            except Exception as ex:
                print ex
                item = root.child(0)
            self.ui.treeWidget.setCurrentItem(item)

    def add_another_summary(self):
        currentNode = self.modifiedNode
        self.summaryNode = currentNode
        self.addingSummary = True
        branchChildren = self.db.nodes.find_one({'name': currentNode,
                                                 'attr': ""})['data'].keys()
        self.ui.tabWidget.hide()
        self.ui.functionButton.setText("Add Summary")
        model = QtGui.QStandardItemModel()
        for i in branchChildren:
            item = QtGui.QStandardItem(i)
            check = QtCore.Qt.Unchecked
            item.setCheckState(check)
            item.setCheckable(True)
            model.appendRow(item)
        self.ui.devicesListView.setModel(model)
        self.ui.devicesListView.show()
        self.ui.summaryNameLE.show()
        self.ui.groupBox.show()
        self.ui.addBranchDevices.hide()
        self.ui.timeLabel.hide()
        self.ui.timeLineEdit.hide()
        self.ui.minutesLabel.hide()

    def modify_summary(self):
        nodes = self.db.nodes
        flag = 1
        if '-' in self.modifiedNode:
            nodeType = 'leaf'
        else:
            nodeType = 'branch'
        if nodeType == 'branch':
            self.ui.timeLabel.hide()
            self.ui.timeLineEdit.hide()
            self.ui.minutesLabel.hide()
        elif nodeType == 'leaf':
            self.ui.timeLabel.show()
            self.ui.timeLineEdit.show()
            self.ui.minutesLabel.show()
        self.ui.functionButton.setText('Modify Summary')
        self.isModified = True
        self.ui.tabWidget.hide()
        self.ui.groupBox.show()

    def update_plot(self):
        print "updating_plot:  current node --", self.currentNode
        if '-' in self.currentNode:
            print "updating_plot:  proxy---", self.proxy
            params = json.dumps({'period': self.data_period})
            data = json.loads(self.proxy.get_data(params))
            data = data[self.attr]
            data1 = [i[1] for i in data]
            latest_data = data[len(data) - 1][0].split('T')[1].split(':')
            print "latest_data:", latest_data
            lh = latest_data[0]
            lm = latest_data[1]
            self.data = data1
            dataval = [":".join(i[0].split('T')[1].split(':')[2:])
                       for i in data]
            data_hour = dataval[len(dataval) - 1]
            values = range(1, len(dataval)+1)
            xdata = zip(values, dataval)
            self.ui.graphicsView.getAxis('bottom').setTicks([xdata])
            self.curve.setData(self.data)
            # temp = self.proxy[self.attr].value
        else:
            node = self.db.nodes.find_one(
                {'name': self.currentNode, 'attr': ""})
            try:
                temp = node['data'][str(self.ui.childrenBox.currentText())]
            except KeyError as kex:
                print "Exception"
                print kex
                return
            except TypeError as tex:
                return
            try:
                if len(self.data) >= self.total_graph_values:
                    self.data.pop(0)
                    self.branch_axis_counter += 1
                    self.curve.setPos(self.branch_axis_counter, 0)
                self.data.append(temp)
                if len(self.branch_time_array) >= self.total_graph_values:
                    self.branch_time_array.pop(0)
                ct = dt.now().time()
                lh = ct.hour
                lm = ct.minute
                self.branch_time_array.append(
                    str(ct.second) + ":" + str(ct.microsecond))
            except Exception as ex:
                print ex
            values = range(
                self.branch_axis_counter, self.branch_axis_counter + len(self.data) + 1)
            xdata = zip(values, self.branch_time_array)
            print "----update_plot----"
            print "values:", values
            print "branch_time_array:", self.branch_time_array
            print "branch_axis_counter:", self.branch_axis_counter
            self.ui.graphicsView.getAxis('bottom').setTicks([xdata])
            self.curve.setData(self.data[:])
        self.ui.hour_value.setText(str(lh))
        self.ui.minutes_value.setText(str(lm))

    def init_graph(self):
        temp = str(self.itemText).split(' - ')
        self.ui.hour_label.show()
        self.ui.hour_value.show()
        self.ui.minutes_label.show()
        self.ui.minutes_value.show()
        if len(temp) > 1:
            self.attr = temp[1]
            with open('graph_config', 'r') as fin:
                graph_nodes = json.loads(fin.read())
                current_graph_node = graph_nodes[str(self.itemText)]
                total_graph_values = int(
                    current_graph_node['total_graph_values'])
                graph_updation_time = int(
                    current_graph_node['graph_updation_time'])
                print "----init_graph----"
                print "total_graph_values:", total_graph_values
                print "graph_updation_time:", graph_updation_time
        elif len(temp) == 1:
            self.attr = str(self.ui.childrenBox.currentText())
            with open('graph_config', 'r') as fin:
                graph_nodes = json.loads(fin.read())
                current_graph_node = graph_nodes[str(self.itemText)]
                self.total_graph_values = int(
                    current_graph_node['total_graph_values'])
                graph_updation_time = int(
                    current_graph_node['graph_updation_time'])
                print "----init_graph----"
                print "total_graph_values:", self.total_graph_values
                print "graph_updation_time:", graph_updation_time
            self.branch_time_array = []
            self.branch_axis_counter = 0
        pg.setConfigOptions(antialias=True)
        self.ui.graphicsView.clear()
        self.curve = self.ui.graphicsView.plot(pen='y')
        try:
            self.l.scene().removeItem(self.l)
        except AttributeError:
            pass
        self.l = pg.LegendItem((100, 60), offset=(70, 30))
        self.l.setParentItem(self.ui.graphicsView.graphicsItem())
        self.l.addItem(self.curve, self.attr)
        self.data = []
        self.ptr = 0
        if len(temp) > 1:
            self.proxy = DeviceProxy(temp[0])
            params = json.dumps({'period': 1})
            data = json.loads(self.proxy.get_data(params))
            data = data[self.attr]
            data_len = len(data)
            self.data_period = total_graph_values / data_len
        else:
            self.proxy = ''
        self.graphTimer.stop()
        self.graphTimer.timeout.connect(self.update_plot)
        self.graphTimer.start(graph_updation_time)

    def fetch_data(self, devName):
        temp = devName.split(" - ")
        dName = temp[0]
        dAttr = temp[1]
        proxy = DeviceProxy(dName)
        nodes = self.db.nodes
        node = nodes.find_one({'name': dName, 'attr': dAttr})
        # print node
        temp = proxy[dAttr].value
        if node != None:
            try:
                max_len = node['max_len']
                if len(list(node['data'])) >= max_len:
                    nodes.update(
                        {'name': dName, 'attr': dAttr}, {'$pop': {'data': -1}})
                nodes.update(
                    {'name': dName, 'attr': dAttr}, {'$push': {'data': temp}})
                node = nodes.find_one({'name': dName, 'attr': dAttr})
                summary_data = self.find_summary(
                    list(node['data']), node['function'])
                nodes.update(
                    {'name': dName, 'attr': dAttr}, {'$set': {'summary_data': summary_data}})
                threading.Timer(3, self.fetch_data, [devName]).start()
            except Exception as ex:
                print ex
                print node
        else:
            print devName, "deleted"

    def fetch_branch_data(self, branchName):
        nodes = self.db.nodes
        node = nodes.find_one({'name': branchName, 'attr': ""})
        if node != None:
            childNodes = node['children']
            raw_data = {}
            for i in childNodes:
                if '-' in i:
                    temp = i.split(' - ')
                    chName = temp[0]
                    chAttr = temp[1]
                else:
                    chName = i
                    chAttr = ""
                child = nodes.find_one({'name': chName, 'attr': chAttr})
                childSummary = child['summary_data']
                raw_data[i] = childSummary
            summary_data = self.find_summary(
                raw_data.values(), node['function'])
            updated = nodes.update({'name': branchName, 'attr': ''},
                                   {'$set': {'data': raw_data, 'summary_data': summary_data}})
            threading.Timer(3, self.fetch_branch_data, [branchName]).start()
        else:
            print branchName, "deleted"

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
        attr = ""
        summary_children = {}
        nodes = self.db.nodes
        for radioButton in self.ui.verticalLayoutWidget_3.findChildren(
                QtGui.QRadioButton):
            if radioButton.isChecked():
                summary = str(radioButton.text())
                break

        pattern = re.compile("^[0-9][0-9]:[0-9][0-9]:[0-9][0-9]$")

        if self.isModified == True:
            if '-' in self.modifiedNode:
                nodeType = 'leaf'
            else:
                nodeType = 'branch'
            if nodeType == "leaf":
                timeField = str(self.ui.timeLineEdit.text())
                if len(timeField) == 0:
                    QtGui.QErrorMessage(self).showMessage(
                        "Time Field is required")
                    return
                if not pattern.match(timeField):
                    QtGui.QErrorMessage(self).showMessage(
                        "Please enter time in the correct format -- hh:mm:ss")
                    return
                self.ui.treeWidget.setEnabled(True)
                l = timeField.split(":")
                summaryTime = int(l[0]) * 3600 + int(l[1]) * 60 + int(l[2])
                max_len = (summaryTime * 60) / 2
                temp = self.modifiedNode.split(' - ')
                nodes.update({'name': temp[0], 'attr': temp[1]},
                             {'$set': {'max_len': max_len, 'function': summary}})
            elif nodeType == "branch":
                nodes.update({'name': self.modifiedNode},
                             {'$set': {'function': summary}})
            self.isModified = False
            self.ui.groupBox.hide()
            self.ui.comboBox.hide()
            self.ui.attrLabel.hide()
            return

        if self.addingSummary == True:
            print "Adding summary"
            node = nodes.find_one({'name': self.summaryNode, 'attr': ""})
            summaryName = str(self.ui.summaryNameLE.text())
            if summaryName == "":
                QtGui.QErrorMessage(self).showMessage(
                    "Summary name cannot be empty. Please enter summary name")
                return
            children = []
            model = self.ui.devicesListView.model()
            for i in range(model.rowCount()):
                item = model.item(i)
                itemText = str(item.text())
                if item.checkState() == QtCore.Qt.Checked:
                    children.append(itemText)
            if len(children) == 0:
                QtGui.QErrorMessage(self).showMessage(
                    "Select at least one child for summary!")
                return
            summary_children = node['summary_children']
            summary_children[summaryName] = [children, summary]
            print summary_children
            nodes.update({'name': self.summaryNode, 'attr': ""},
                         {'$set': {'summary_children': summary_children}})
            self.addingSummary = False
            self.ui.groupBox.hide()
            self.ui.comboBox.hide()
            self.ui.attrLabel.hide()
            self.ui.devicesListView.hide()
            self.ui.summaryNameLE.hide()
            return

        sourceType = self.sourceType
        print "logging:", sourceType
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
            attr = self.summaryAttr
            summaryTime = int(l[0]) * 3600 + int(l[1]) * 60 + int(l[2])
            max_len = (summaryTime * 60) / 2
            nodeName = self.devName
            summary_data = 0.0
        elif sourceType == "branch":
            summaryName = str(self.ui.summaryNameLE.text())
            if summaryName == "":
                QtGui.QErrorMessage(self).showMessage(
                    "Summary name cannot be empty. Please enter summary name")
                return
            children = []
            model = self.ui.devicesListView.model()
            for i in range(model.rowCount()):
                item = model.item(i)
                itemText = str(item.text())
                if item.checkState() == QtCore.Qt.Checked:
                    children.append(itemText)
            if len(children) == 0:
                QtGui.QErrorMessage(self).showMessage(
                    "Select at least one child for summary!")
                return
            summary_data = {}
            summary_children[summaryName] = [children, summary]
            children = self.branchChildren
            nodeName = self.branchName
        self.ui.treeWidget.setEnabled(True)
        node = {'name': nodeName,
                'type': sourceType,
                'attr': attr,
                'function': summary,
                'time': summaryTime,
                'children': children,
                'max_len': max_len,
                'data': [],
                'summary_data': summary_data,
                'summary_children': summary_children
                }

        print node
        node_id = nodes.insert_one(node).inserted_id
        print "inserted node:", nodes.find_one({'_id': node_id})
        if sourceType == "leaf":
            ok = None
            while(not ok):
                total_graph_values, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                                    'Enter Total values to be shown in graph:')
                total_graph_values = str(total_graph_values)
            ok = None
            while(not ok):
                graph_updation_time, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                                     'Enter Graph updation time in microseconds:')
                graph_updation_time = str(graph_updation_time)
            with open('graph_config', 'r') as fin:
                try:
                    gc = json.loads(fin.read())
                except ValueError as ex:
                    gc = {}
            with open('graph_config', 'w') as fout:
                dev_name = node['name'] + ' - ' + node['attr']
                print "dev_name:", dev_name
                print "total_graph_values:", total_graph_values
                print "graph_updation_time:", graph_updation_time
                config_dict = {
                    'total_graph_values': total_graph_values, 'graph_updation_time': graph_updation_time}
                gc[dev_name] = config_dict
                fout.write(json.dumps(gc))

            self.update_tree(self.dataSourcesTreeItem, self.devName, attr)
            t = threading.Thread(
                target=self.aggregate_data, args=([self.devName + " - " + attr]))
            t.start()

        if sourceType == "branch":
            ok = None
            while(not ok):
                total_graph_values, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                                    'Enter Total values to be shown in graph:')
                total_graph_values = str(total_graph_values)
            ok = None
            while(not ok):
                graph_updation_time, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                                     'Enter Graph updation time in microseconds:')
                graph_updation_time = str(graph_updation_time)
            with open('graph_config', 'r') as fin:
                try:
                    gc = json.loads(fin.read())
                except ValueError as ex:
                    gc = {}
            with open('graph_config', 'w') as fout:
                branch_name = node['name']
                print "branch_name:", branch_name
                print "total_graph_values:", total_graph_values
                print "graph_updation_time:", graph_updation_time
                config_dict = {
                    'total_graph_values': total_graph_values, 'graph_updation_time': graph_updation_time}
                gc[branch_name] = config_dict
                fout.write(json.dumps(gc))

            t = threading.Thread(target=self.aggregate_branch_data,
                                 args=([self.branchName]))
            t.start()
        self.ui.groupBox.hide()
        self.ui.comboBox.hide()
        self.ui.summaryCB.hide()
        self.ui.summaryLW1.hide()
        self.ui.summaryLW2.hide()
        self.ui.devicesListView.hide()
        self.ui.summaryNameLE.hide()
        self.ui.attrLabel.hide()

    def finalize_branch(self):
        model = self.ui.devicesListView.model()
        self.branchChildren = []
        nodes = self.db.nodes
        for i in range(model.rowCount()):
            item = model.item(i)
            itemText = str(item.text())
            if '-' in itemText:
                attr = itemText.split(" - ")[1]
                devName = itemText.split(" - ")[0]
            else:
                devName = itemText
                attr = ""
            if item.checkState() == QtCore.Qt.Checked:
                self.branchChildren.append(itemText)
                self.update_tree(self.treeBranch, devName, attr)
        self.ui.treeWidget.setCurrentItem(self.treeBranch)
        self.treeBranch.setSelected(True)
        self.ui.timeLabel.hide()
        self.ui.timeLineEdit.hide()
        self.ui.minutesLabel.hide()

        model = QtGui.QStandardItemModel()
        for i in self.branchChildren:
            item = QtGui.QStandardItem(i)
            check = QtCore.Qt.Unchecked
            item.setCheckState(check)
            item.setCheckable(True)
            model.appendRow(item)
        self.ui.devicesListView.setModel(model)
        self.ui.summaryNameLE.show()
        self.ui.addBranchDevices.hide()
        self.ui.groupBox.show()

    def create_branch(self):
        self.ui.tabWidget.hide()
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
                if i['type'] == 'leaf':
                    device = i['name'] + " - " + i['attr']
                elif i['type'] == 'branch':
                    device = i['name']
                item = QtGui.QStandardItem(device)
                check = QtCore.Qt.Unchecked
                item.setCheckState(check)
                item.setCheckable(True)
                model.appendRow(item)
            self.ui.devicesListView.setModel(model)
            self.ui.devicesListView.show()
            self.ui.addBranchDevices.show()
            self.ui.treeWidget.setEnabled(False)

    def update_tree(self, source, value, attr):
        tempDataSource = QtGui.QTreeWidgetItem(source)
        nodes = self.db.nodes
        ch = nodes.find_one({'name': value, 'attr': attr})
        if ch['type'] == 'leaf':
            tempDataSource.setText(0, value + " - " + attr)
        elif ch['type'] == 'branch':
            tempDataSource.setText(0, value)
        self.ui.treeWidget.setCurrentItem(tempDataSource)
        tempDataSource.setSelected(True)

    def find_summary(self, data, function):
        if function == "Minimum":
            return min(data)
        elif function == "Maximum":
            return max(data)
        elif function == "Average":
            try:
                value = float(sum(data)) / len(data)
            except ZeroDivisionError:
                value = 0
            return value

    def update_leafdata(self):
        temp = self.currentNode.split(' - ')
        node = self.db.nodes.find_one({'name': temp[0], 'attr': temp[1]})
        if node != None:
            if len(node['data']) != 0:
                value = node['data'][-1]
                self.ui.attributeValue.setText(str(value))
                self.ui.summaryValue.setText(str(node['summary_data']))

    def update_branchdata(self):
        try:
            nodes = self.db.nodes
            node = nodes.find_one({'name': self.branchNode, 'attr': ''})
            summary_name = str(self.ui.summaryCB.currentText())
            print "update_branchdata --- summary name:", summary_name
            if node != None:
                c = 0
                data = node['data']
                print "update_branchdata ---", node['summary_children']
                try:
                    summary_children = node[
                        'summary_children'][summary_name][0]
                except KeyError:
                    return
                try:
                    summ_data = [data[i] for i in summary_children]
                except KeyError as e:
                    summary_children = node['summary_children']
                    summary_children[summary_name][0].remove(e.args[0])
                    nodes.update({'name': self.branchNode, 'attr': ''},
                                 {'$set': {'summary_children': summary_children}})
                    summary_children = summary_children[summary_name][0]
                    summ_data = [data[i] for i in summary_children]
                except TypeError as e:
                    return
                summ_data = self.find_summary(
                    summ_data, node['summary_children'][summary_name][1])
                self.ui.listWidget.clear()
                self.ui.listWidget_2.clear()
                self.ui.summaryLW1.clear()
                self.ui.summaryLW2.clear()
                keys = data.keys()
                values = map(str, data.values())
                self.ui.summaryLW1.addItems(summary_children)
                self.ui.summaryLW2.addItem(str(summ_data))
                self.ui.listWidget.addItems(keys)
                self.ui.listWidget_2.addItems(values)
                self.ui.summaryLabel.setText(node["function"] + " Value: ")
                self.ui.summaryValue.setText(str(node['summary_data']))
        except AttributeError:
            pass

    def onClick(self, item, column):
        for i in self.nodeTimers:
            i.stop()
        try:
            print item.text(0)
            print item.parent().text(0)
            self.parentNode = str(item.parent().text(column))
        except Exception as ex:
            self.parentNode = None
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
        if '-' in currentNode:
            temp = currentNode.split(' - ')
            node = nodes.find_one({'name': temp[0], 'attr': temp[1]})
        else:
            node = nodes.find_one({'name': currentNode, 'attr': ''})

        self.currentNode = currentNode
        self.itemText = currentNode
        self.currentNodeType = node['type']
        if node['type'] == "leaf":
            self.ui.actionDelete_Summary.setEnabled(False)
            self.ui.childrenBox.hide()
            # self.ui.selectChildLabel.hide()
            self.ui.listWidget.hide()
            self.ui.listWidget_2.hide()
            self.ui.summaryCB.hide()
            self.ui.summaryLW1.hide()
            self.ui.summaryLW2.hide()
            self.init_graph()
            self.ui.attributeName.setText(node['attr'].capitalize())
            self.ui.summaryLabel.setText(node['function'] +
                                         node['attr'].capitalize() + ": ")
            self.ui.tabWidget.show()
            timer = QtCore.QTimer()
            self.nodeTimers.append(timer)
            timer.timeout.connect(self.update_leafdata)
            timer.start(5000)
            self.update_leafdata()
        else:
            self.ui.actionModify_Summary.setEnabled(False)
            children_no = len(node['summary_children'].keys())
            self.ui.actionDelete_Summary.setEnabled(True)
            summaryCB = self.ui.summaryCB
            summaryCB.clear()
            summaryCB.addItems(node['summary_children'].keys())
            summaryCB.show()
            summaryLW1 = self.ui.summaryLW1
            summaryLW2 = self.ui.summaryLW2
            self.branchNode = node['name']
            self.ui.childrenBox.show()
            # self.ui.selectChildLabel.show()
            self.ui.childrenBox.clear()
            print node['data']
            try:
                self.ui.childrenBox.addItems(node['data'].keys())
            except:
                pass
            self.ui.tabWidget.show()
            self.ui.listWidget.show()
            self.ui.listWidget_2.show()
            summaryLW1.clear()
            summaryLW2.clear()
            summaryLW1.show()
            summaryLW2.show()
            timer = QtCore.QTimer()
            self.nodeTimers.append(timer)
            timer.timeout.connect(self.update_branchdata)
            timer.start(5000)
            self.update_branchdata()

    def add_device(self):
        self.ui.tabWidget.hide()
        self.ui.functionButton.setText("Add Summary")
        print "Add new device"
        devName, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                 'Enter Device Address:')
        devName = str(devName)
        self.devName = devName
        nodes = self.db.nodes
        if ok:
            try:
                self.sourceType = "leaf"
                self.proxy = DeviceProxy(devName)
                msgBox = QtGui.QMessageBox()
                msgBox.setText('Device added successfully')
                msgBox.addButton(QtGui.QPushButton('Ok'),
                                 QtGui.QMessageBox.YesRole)
                ret = msgBox.exec_()
                dev_attrs = self.proxy.get_attribute_list()
                self.ui.comboBox.clear()
                for i in dev_attrs:
                    flag = 0
                    if nodes.find_one({'name': devName, 'attr': i}) != None:
                        flag = 1
                    if i == "State" or i == "Status" or flag == 1:
                        continue
                    self.ui.comboBox.addItem(i)
                self.ui.groupBox.show()
                self.ui.comboBox.show()
                self.ui.attrLabel.show()
                self.ui.timeLabel.show()
                self.ui.timeLineEdit.show()
                self.ui.minutesLabel.show()
                self.ui.treeWidget.setEnabled(False)
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
