from __future__ import division, print_function

import matplotlib

matplotlib.use('WXAgg')

from wx.lib.plot import PlotCanvas, PolyLine, PlotGraphics


class PlotCanvas(PlotCanvas):
    def __init__(self, *args, **kwargs):
        super(PlotCanvas, self).__init__(*args, **kwargs)
        self.gc = PlotGraphics([], 'Line Graph', 'X Axis', 'Y Axis')

    def set_data(self, data=None):
        self.data = data

    def draw(self):
        self.Clear()
        line = PolyLine(self.data, legend='', colour='blue', width=1)
        self.gc = PlotGraphics([line], 'Line Graph', 'X Axis', 'Y Axis')
        self.Draw(self.gc)
