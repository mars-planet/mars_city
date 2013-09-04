from __future__ import division, print_function

import matplotlib as mp
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure

mp.use('WXAgg')

import wx


class CanvasPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        wx.Panel.__init__(self, *args, **kwargs)
        #self.size = (800, 50)
        self.figure = Figure()
        self.figure.set_size_inches((8, 1))
        self.figure.set_dpi(80)
        #self.axes = self.figure.add_subplot(111)
        self.canvas = FigureCanvas(self, -1, self.figure)
        self.axes = self.figure.add_subplot(111)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.sizer)
        self.Fit()

    def draw(self, data):
        data_x, data_y = data
        self.axes.plot(data_x, data_y)
        self.canvas.draw()
        self.canvas.Refresh()
