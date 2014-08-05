# -*- coding: utf-8 -*-
# icons' license http://icons8.com/license/
from __future__ import division, print_function

import ConfigParser
from datetime import datetime, timedelta
import os
import sys
from threading import current_thread
from time import sleep

from PyTango import DeviceProxy  # , ConnectionFailed
# from numpy import nan, isnan
import wx
# from wxmplot import PlotPanel

from Timer import Timer


# from colorsys import hsv_to_rgb
# begin wxGlade: extracode
# end wxGlade
class MainFrame(wx.Frame):

    monitor_address = 'C3/health_monitor/1'
    yellow_alrm_thrsh = 0.25
    red_alrm_thrsh = 0.5
    sleep_time = 5

    def __init__(self, *args, **kwds):
        self.proxy = DeviceProxy(MainFrame.monitor_address)
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        # self.alarm_plt = PlotPanel(self)

        self.img_size = 32
        self.src_panels = {}
        src_names = self.proxy.source_names
        src_sexes = self.proxy.source_sexes
        src_addresses = self.proxy.source_addresses
        self.src_data = [(addr, src_names[i], src_sexes[i])
                         for i, addr in enumerate(src_addresses)]
        self.font = wx.Font(self.img_size / 2, wx.FONTFAMILY_DEFAULT,
                            wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        for address, name, sex in self.src_data:
            container = {}
            container['proxy'] = DeviceProxy(address)
            # name & sex section
            container['sex'] = sex
            if sex == 'male':
                sex_image = wx.Image('images/male-icon.png')
            else:
                sex_image = wx.Image('images/female-icon.png')
            sex_image = sex_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['sex_icon'] = wx.StaticBitmap(
                                    self, -1, wx.BitmapFromImage(sex_image)
                                                    )
            container['name'] = name
            container['name_lbl'] = wx.StaticText(self, -1, name)
            container['name_lbl'].SetFont(self.font)
            container['name_lbl'].Wrap(self.img_size * 8)
            # air flow section
            air_image = wx.Image('images/air-icon.png')
            air_image = air_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['air_icon'] = wx.StaticBitmap(
                                        self, -1, wx.BitmapFromImage(air_image)
                                                    )
            container['air_lbl'] = wx.StaticText(self, -1, 'nan')
            container['air_lbl'].SetFont(self.font)
            container['air_lbl'].Wrap(self.img_size * 8)
            # acceleration section
            acc_image = wx.Image('images/acc-icon.png')
            acc_image = acc_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['acc_icon'] = wx.StaticBitmap(
                                        self, -1, wx.BitmapFromImage(acc_image)
                                                    )
            container['acc_lbl'] = wx.StaticText(self, -1, 'nan')
            container['acc_lbl'].SetFont(self.font)
            container['acc_lbl'].Wrap(self.img_size * 8)
            self.src_panels[address] = container
            # ECG section
            ecg_image = wx.Image('images/ecg-icon.png')
            ecg_image = ecg_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['ecg_icon'] = wx.StaticBitmap(
                                        self, -1, wx.BitmapFromImage(ecg_image)
                                                    )
            # heart rate section
            hr_image = wx.Image('images/hr-icon.png')
            hr_image = hr_image.Scale(self.img_size, self.img_size,
                                      wx.IMAGE_QUALITY_HIGH)
            container['hr_icon'] = wx.StaticBitmap(
                                        self, -1, wx.BitmapFromImage(hr_image)
                                                   )
            container['hr_lbl'] = wx.StaticText(self, -1, 'nan')
            container['hr_lbl'].SetFont(self.font)
            container['hr_lbl'].Wrap(self.img_size * 8)
            # SPO2 section
            o2_image = wx.Image('images/o2-icon.png')
            o2_image = o2_image.Scale(self.img_size, self.img_size,
                                      wx.IMAGE_QUALITY_HIGH)
            container['o2_icon'] = wx.StaticBitmap(
                                        self, -1, wx.BitmapFromImage(o2_image)
                                                   )
            container['o2_lbl'] = wx.StaticText(self, -1, 'nan')
            container['o2_lbl'].SetFont(self.font)
            container['o2_lbl'].Wrap(self.img_size * 8)
            self.src_panels[address] = container
            # temperature section
            temp_image = wx.Image('images/temp-icon.png')
            temp_image = temp_image.Scale(self.img_size, self.img_size,
                                          wx.IMAGE_QUALITY_HIGH)
            container['temp_icon'] = wx.StaticBitmap(
                                    self, -1, wx.BitmapFromImage(temp_image)
                                                     )
            container['temp_lbl'] = wx.StaticText(self, -1, 'nan')
            container['temp_lbl'].SetFont(self.font)
            container['temp_lbl'].Wrap(self.img_size * 8)
            self.src_panels[address] = container

        self.__set_properties()
        self.__do_layout()

        self.timer_thread = Timer(target=self.timer_tick)
        self.alarms = set()
        self.timer_thread.start()

    def __set_properties(self):
        self.SetTitle("Health Monitor GUI")

    def __do_layout(self):
        self.frame_sizer = wx.BoxSizer(wx.VERTICAL)
        for i, container in enumerate(self.src_panels.itervalues()):
            if i:
                self.frame_sizer.AddSpacer(3)
            row_sizer = wx.BoxSizer(wx.HORIZONTAL)
            id_sizer = wx.BoxSizer(wx.HORIZONTAL)
            air_sizer = wx.BoxSizer(wx.HORIZONTAL)
            acc_sizer = wx.BoxSizer(wx.HORIZONTAL)
            ecg_sizer = wx.BoxSizer(wx.HORIZONTAL)
            hr_sizer = wx.BoxSizer(wx.HORIZONTAL)
            o2_sizer = wx.BoxSizer(wx.HORIZONTAL)
            temp_sizer = wx.BoxSizer(wx.HORIZONTAL)
            id_sizer.Add(container['sex_icon'], 0, wx.ALL, 0)
            id_sizer.Add(container['name_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(id_sizer, 1, wx.EXPAND, 0)
            air_sizer.Add(container['air_icon'], 0, wx.ALL, 0)
            air_sizer.Add(container['air_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(air_sizer, 1, wx.EXPAND, 0)
            acc_sizer.Add(container['acc_icon'], 0, wx.ALL, 0)
            acc_sizer.Add(container['acc_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(acc_sizer, 1, wx.EXPAND, 0)
            ecg_sizer.Add(container['ecg_icon'], 0, wx.ALL, 0)
            row_sizer.Add(ecg_sizer, 1, wx.EXPAND, 0)
            hr_sizer.Add(container['hr_icon'], 0, wx.ALL, 0)
            hr_sizer.Add(container['hr_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(hr_sizer, 1, wx.EXPAND, 0)
            o2_sizer.Add(container['o2_icon'], 0, wx.ALL, 0)
            o2_sizer.Add(container['o2_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(o2_sizer, 1, wx.EXPAND, 0)
            temp_sizer.Add(container['temp_icon'], 0, wx.ALL, 0)
            temp_sizer.Add(container['temp_lbl'], 0, wx.ALL, 0)
            row_sizer.Add(temp_sizer, 1, wx.EXPAND, 0)
            self.frame_sizer.Add(row_sizer, 1, wx.EXPAND, 0)

        self.SetSizer(self.frame_sizer)
        self.frame_sizer.Fit(self)
        self.MinSize = self.frame_sizer.Size
        self.SetAutoLayout(True)
        self.Layout()

    def select_alrm_img(self, name_prefix, name_sufix, value):
        if value < self.yellow_alrm_thrsh:
            color = 'green'
            code = 0
        elif self.yellow_alrm_thrsh <= value < self.red_alrm_thrsh:
            color = 'yellow'
            code = 1
        else:
            color = 'red'
            code = 2
        img = wx.Image('%s%s%s' % (name_prefix, color, name_sufix))
        img = img.Scale(self.img_size, self.img_size,
                        wx.IMAGE_QUALITY_HIGH)
        return wx.BitmapFromImage(img), code

    def timer_tick(self):
        timer = current_thread()
        # proxy = self.proxy
        while not timer.stopped():
            init = datetime.now()
            for container in self.src_panels.itervalues():
                try:
                    codes = [0] * 6
                    # update air flow values
                    air_flow = container['proxy'].air_flow
                    image, codes[0] = self.select_alrm_img('images/hr-icon-',
                                                           '.png', air_flow)
                    wx.CallAfter(container['air_lbl'].SetLabel,
                                 "{:6.2f}".format(air_flow))
                    wx.CallAfter(container['air_icon'].SetBitmap, image)
                    # update acceleration values
                    acc_magn = container['proxy'].acc_magn
                    image, codes[1] = self.select_alrm_img('images/acc-icon-',
                                                           '.png', acc_magn)
                    wx.CallAfter(container['acc_lbl'].SetLabel,
                                 "{:3.2f}".format(acc_magn))
                    wx.CallAfter(container['acc_icon'].SetBitmap, image)
                    # update ecg values
                    ecg_v1 = container['proxy'].ecg_v1
                    image, codes[2] = self.select_alrm_img('images/ecg-icon-',
                                                           '.png', ecg_v1)
                    wx.CallAfter(container['ecg_icon'].SetBitmap, image)
                    # update heart rate values
                    heart_rate = container['proxy'].heart_rate
                    image, codes[3] = self.select_alrm_img('images/hr-icon-',
                                                           '.png', heart_rate)
                    wx.CallAfter(container['hr_lbl'].SetLabel,
                                 "{:3.0f}".format(heart_rate))
                    wx.CallAfter(container['hr_icon'].SetBitmap, image)
                    # update SPO2 values
                    o2 = container['proxy'].o2
                    image, codes[4] = self.select_alrm_img('images/o2-icon-',
                                                           '.png', o2)
                    wx.CallAfter(container['o2_lbl'].SetLabel,
                                 "{:4.2f}".format(o2))
                    wx.CallAfter(container['o2_icon'].SetBitmap, image)
                    # update temperature values
                    temperature = container['proxy'].temperature
                    image, codes[5] = self.select_alrm_img('images/temp-icon-',
                                                           '.png', temperature)
                    wx.CallAfter(container['temp_lbl'].SetLabel,
                                 "{:4.2f}".format(temperature))
                    wx.CallAfter(container['temp_icon'].SetBitmap, image)
                    alarm_code = sum(codes) / len(codes)
                    image, _ = self.select_alrm_img('images/%s-icon-' %
                                                        container['sex'],
                                                    '.png', alarm_code)
                    wx.CallAfter(container['sex_icon'].SetBitmap, image)
                except Exception as e:  # ConnectionFailed:
                    print(e)
            sleep_time = timedelta(seconds=MainFrame.sleep_time)
            sleep_time -= (datetime.now() - init)
            sleep_time = max(sleep_time, timedelta(0)).total_seconds()
            sleep(sleep_time)

    def plot(self, data):
        x_data, y_data = data
        self.alarm_plt.plot(x_data, y_data, title="Alarm Level History")


config = ConfigParser.RawConfigParser()
dirname = os.path.dirname(os.path.abspath(sys.argv[0]))
cfg_filename = os.path.join(dirname, 'health_monitor_gui.cfg')
config.read(cfg_filename)
MainFrame.monitor_address = config.get('Monitor_GUI', 'monitor_address')
MainFrame.yellow_alrm_thrsh = config.getfloat('Monitor_GUI',
                                              'yellow_alrm_thrsh')
MainFrame.red_alrm_thrsh = config.getfloat('Monitor_GUI', 'red_alrm_thrsh')
MainFrame.sleep_time = config.getint('Monitor_GUI', 'sleep_time')
