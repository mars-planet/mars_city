# -*- coding: utf-8 -*-
# icons' license http://icons8.com/license/
from __future__ import division, print_function

from collections import deque
import os

import wx
import wxmplot

import utils


class DetailsFrame(wx.Frame):
    def __init__(self, parent, name, address, plot_width=600, plot_height=150,
                 air_flow_len=1000, acc_magn_len=1000, ecg_v1_len=1000,
                 heart_rate_len=1000, o2_len=1000, temperature_len=1000,
                 capture_file_name='details_frame.png', sleep_time=5000,
                 *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, parent, *args, **kwds)

        self.parent = parent
        self.name = name
        self.address = address
        dir_name = os.path.dirname(capture_file_name)
        base_name = os.path.basename(capture_file_name).split('.')
        self.capture_file_name = '{}_{}.{}'.format(base_name[0], self.name,
                                                   base_name[1])
        self.capture_file_name = os.path.join(dir_name, self.capture_file_name)
        self.air_flow = deque([0] * air_flow_len, maxlen=air_flow_len)
        self.air_flow_plt = wxmplot.PlotPanel(
                                    self, output_title='Air Flow',
                                    size=(plot_width, plot_height),
                                    messenger=None)
        self.acc_magn = deque([0] * acc_magn_len, maxlen=acc_magn_len)
        self.acc_magn_plt = wxmplot.PlotPanel(
                                        self, output_title='Body Acceleration',
                                        size=(plot_width, plot_height),
                                        messenger=None)
        self.ecg_v1 = deque([0] * ecg_v1_len, maxlen=ecg_v1_len)
        self.ecg_v1_plt = wxmplot.PlotPanel(
                                        self, output_title='ECG',
                                        size=(plot_width, plot_height),
                                        messenger=None)
        self.heart_rate = deque([0] * heart_rate_len, maxlen=heart_rate_len)
        self.heart_rate_plt = wxmplot.PlotPanel(
                                        self, output_title='Heart Rate',
                                        size=(plot_width, plot_height),
                                        messenger=None)
        self.o2 = deque([0] * o2_len, maxlen=o2_len)
        self.o2_plt = wxmplot.PlotPanel(
                                    self, output_title='SPO2',
                                    size=(plot_width, plot_height),
                                    messenger=None)
        self.temperature = deque([0] * temperature_len, maxlen=temperature_len)
        self.temperature_plt = wxmplot.PlotPanel(
                                            self, output_title='Temperature',
                                            size=(plot_width, plot_height),
                                            messenger=None)
        # draw the plots once so they set up the labels
        kwargs = {'air_flow': self.air_flow, 'acc_magn': self.acc_magn,
                  'ecg_v1': self.ecg_v1, 'heart_rate': self.heart_rate,
                  'o2': self.o2, 'temperature': self.temperature}
        self.update_plots(**kwargs)

        self.sleep_time = sleep_time
        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.__set_properties()
        self.setup_layout()

    def __set_properties(self):
        self.SetTitle(self.name)

    def on_close(self, evt):
        wx.PostEvent(self.parent, evt)

    def setup_layout(self):
        frame_sizer = wx.BoxSizer(wx.HORIZONTAL)
        col1_sizer = wx.BoxSizer(wx.VERTICAL)
        col2_sizer = wx.BoxSizer(wx.VERTICAL)
        # organize air flow objects
        col1_sizer.Add(self.air_flow_plt, 1, wx.EXPAND, 0)
        # organize acceleration magnitude objects
        col1_sizer.Add(self.acc_magn_plt, 1, wx.EXPAND, 0)
        # organize ECG V1 objects
        col1_sizer.Add(self.ecg_v1_plt, 1, wx.EXPAND, 0)
        frame_sizer.Add(col1_sizer, 1, wx.EXPAND, 0)
        # organize heart rate objects
        col2_sizer.Add(self.heart_rate_plt, 1, wx.EXPAND, 0)
        # organize SPO2 objects
        col2_sizer.Add(self.o2_plt, 1, wx.EXPAND, 0)
        # organize temperature objects
        col2_sizer.Add(self.temperature_plt, 1, wx.EXPAND, 0)
        frame_sizer.Add(col2_sizer, 1, wx.EXPAND, 0)

        self.SetSizer(frame_sizer)
        frame_sizer.Fit(self)
        self.MinSize = frame_sizer.Size
        self.SetAutoLayout(True)
        self.Layout()

    def update_plots(self, **kwargs):
        if 'air_flow' in kwargs:
            self.air_flow.extend(kwargs['air_flow'])
            self.air_flow_plt.plot(range(self.air_flow.maxlen), self.air_flow,
                                   ylabel='Air Flow')
        if 'acc_magn' in kwargs:
            self.acc_magn.extend(kwargs['acc_magn'])
            self.acc_magn_plt.plot(range(self.acc_magn.maxlen), self.acc_magn,
                                   ylabel='Body Accel.')
        if 'ecg_v1' in kwargs:
            self.ecg_v1.extend(kwargs['ecg_v1'])
            self.ecg_v1_plt.plot(range(self.ecg_v1.maxlen), self.ecg_v1,
                                 ylabel='ECG')
        if 'heart_rate' in kwargs:
            self.heart_rate.extend(kwargs['heart_rate'])
            self.heart_rate_plt.plot(range(self.heart_rate.maxlen),
                                     self.heart_rate, ylabel='Heart Rate')
        if 'o2' in kwargs:
            self.o2.extend(kwargs['o2'])
            self.o2_plt.plot(range(self.o2.maxlen), self.o2, ylabel='%%SPO2')
        if 'temperature' in kwargs:
            self.temperature.extend(kwargs['temperature'])
            self.temperature_plt.plot(range(self.temperature.maxlen),
                                      self.temperature, ylabel='Body Temp.')

