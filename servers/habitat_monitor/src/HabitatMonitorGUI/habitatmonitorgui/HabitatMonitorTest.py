import sys
import unittest
from PyQt4.QtGui import QApplication
from PyQt4.QtTest import QTest
from PyQt4.QtCore import Qt
import HabitatMonitor

class HabitatMonitorTest(unittest.TestCase):
    '''Test the margarita mixer GUI'''
    def setUp(self):
        '''Create the GUI'''
        self.app = QApplication(sys.argv)
        self.gui = HabitatMonitor.HabitatMonitor()

    def setFormToZero(self):
        '''Set all ingredients to zero in preparation for setting just one
        to a nonzero value.
        '''
        self.gui.ui.timeLineEdit.setText("0.0")
        self.gui.ui.summaryNameLE.setText("0.0")       

    def test_defaults(self):
        '''Test the GUI in its default state'''
        
        self.setFormToZero()

        self.assertEqual(self.gui.ui.timeLineEdit.text(), "0.0")
        self.assertEqual(self.gui.ui.summaryNameLE.text(), "0.0")

if __name__ == "__main__":
    unittest.main()