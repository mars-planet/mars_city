# -*- coding: utf-8 -*-
# icons' license http://icons8.com/license/
from __future__ import division, print_function

import ConfigParser
from datetime import datetime, timedelta
import json
import os
import sys

from dateutil.parser import parse as dateutil_parse
import numpy as np
from PyTango import DeviceProxy, ConnectionFailed
import wx

from details_frame import DetailsFrame
from Timer import Timer
import utils

class MainFrame(wx.Frame):

    monitor_address = 'C3/health_monitor/1'
    yellow_alrm_thrsh = 0.25
    red_alrm_thrsh = 0.5
    sleep_time = 5

    def __init__(self, *args, **kwds):
        self.proxy = DeviceProxy(MainFrame.monitor_address)
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)

        self.img_size = 32
        self.src_panels = {}
        src_names = self.proxy.source_names
        src_sexes = self.proxy.source_sexes
        src_addresses = self.proxy.source_addresses
        self.src_data = [(addr, src_names[i], src_sexes[i])
                         for i, addr in enumerate(src_addresses)]
        self.font = wx.Font(self.img_size / 2, wx.FONTFAMILY_DEFAULT,
                            wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.active_plots = {}
        for address, name, sex in self.src_data:
            container = {}
            container['address'] = address
            container['proxy'] = DeviceProxy(address)
            freq = container['proxy'].get_attribute_poll_period('air_flow')
            container['air_flow_poll_depth'] = (self.sleep_time * 1000) // freq
            freq = container['proxy'].get_attribute_poll_period('acc_magn')
            container['acc_magn_poll_depth'] = (self.sleep_time * 1000) // freq
            freq = container['proxy'].get_attribute_poll_period('ecg_v1')
            container['ecg_v1_poll_depth'] = (self.sleep_time * 1000) // freq
            freq = container['proxy'].get_attribute_poll_period('heart_rate')
            container['heart_rate_poll_depth'] = ((self.sleep_time * 1000) //
                                                  freq)
            freq = container['proxy'].get_attribute_poll_period('o2')
            container['o2_poll_depth'] = (self.sleep_time * 1000) // freq
            freq = container['proxy'].get_attribute_poll_period('temperature')
            container['temperature_poll_depth'] = ((self.sleep_time * 1000) //
                                                   freq)
            # name & sex section
            container['sex'] = sex
            if sex == 'male':
                sex_image = wx.Image('images/male-icon-gray.png')
            else:
                sex_image = wx.Image('images/female-icon-gray.png')
            sex_image = sex_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['sex_icon'] = wx.StaticBitmap(
                                    self, wx.ID_ANY,
                                    wx.BitmapFromImage(sex_image)
                                                    )
            container['name'] = name
            container['name_lbl'] = wx.StaticText(self, wx.ID_ANY, name)
            container['name_lbl'].SetFont(self.font)
            container['name_lbl'].Wrap(self.img_size * 8)
            # air flow section
            air_image = wx.Image('images/air_flow-icon-gray.png')
            air_image = air_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['air_flow_icon'] = wx.StaticBitmap(
                                        self, wx.ID_ANY,
                                        wx.BitmapFromImage(air_image)
                                                         )
            container['air_flow_lbl'] = wx.StaticText(self, wx.ID_ANY, 'nan')
            container['air_flow_lbl'].SetFont(self.font)
            container['air_flow_lbl'].Wrap(self.img_size * 8)
            # acceleration section
            acc_image = wx.Image('images/acc_magn-icon-gray.png')
            acc_image = acc_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['acc_magn_icon'] = wx.StaticBitmap(
                                        self, wx.ID_ANY,
                                        wx.BitmapFromImage(acc_image)
                                                         )
            container['acc_magn_lbl'] = wx.StaticText(self, wx.ID_ANY, 'nan')
            container['acc_magn_lbl'].SetFont(self.font)
            container['acc_magn_lbl'].Wrap(self.img_size * 8)
            self.src_panels[address] = container
            # ECG section
            ecg_image = wx.Image('images/ecg_v1-icon-gray.png')
            ecg_image = ecg_image.Scale(self.img_size, self.img_size,
                                        wx.IMAGE_QUALITY_HIGH)
            container['ecg_v1_icon'] = wx.StaticBitmap(
                                        self, wx.ID_ANY,
                                        wx.BitmapFromImage(ecg_image)
                                                        )
            # heart rate section
            hr_image = wx.Image('images/heart_rate-icon-gray.png')
            hr_image = hr_image.Scale(self.img_size, self.img_size,
                                      wx.IMAGE_QUALITY_HIGH)
            container['heart_rate_icon'] = wx.StaticBitmap(
                                        self, wx.ID_ANY,
                                        wx.BitmapFromImage(hr_image)
                                                           )
            container['heart_rate_lbl'] = wx.StaticText(self, wx.ID_ANY, 'nan')
            container['heart_rate_lbl'].SetFont(self.font)
            container['heart_rate_lbl'].Wrap(self.img_size * 8)
            # SPO2 section
            o2_image = wx.Image('images/o2-icon-gray.png')
            o2_image = o2_image.Scale(self.img_size, self.img_size,
                                      wx.IMAGE_QUALITY_HIGH)
            container['o2_icon'] = wx.StaticBitmap(
                                        self, wx.ID_ANY,
                                        wx.BitmapFromImage(o2_image)
                                                   )
            container['o2_lbl'] = wx.StaticText(self, wx.ID_ANY, 'nan')
            container['o2_lbl'].SetFont(self.font)
            container['o2_lbl'].Wrap(self.img_size * 8)
            # temperature section
            temp_image = wx.Image('images/temperature-icon-gray.png')
            temp_image = temp_image.Scale(self.img_size, self.img_size,
                                          wx.IMAGE_QUALITY_HIGH)
            container['temperature_icon'] = wx.StaticBitmap(
                                    self, wx.ID_ANY,
                                    wx.BitmapFromImage(temp_image)
                                                            )
            container['temperature_lbl'] = wx.StaticText(self, wx.ID_ANY,
                                                         'nan')
            container['temperature_lbl'].SetFont(self.font)
            container['temperature_lbl'].Wrap(self.img_size * 8)

            container['details_btn'] = wx.Button(self, wx.ID_ANY, label='>>',
                                                 name=address)
            self.Bind(event=wx.EVT_BUTTON, handler=self.on_details_click,
                      source=container['details_btn'])
            self.src_panels[address] = container

        self.__set_properties()
        self.setup_layout()

        self.alarms = set()
        self.timer = wx.Timer(self)  # message will be sent to the panel
        self.Bind(wx.EVT_TIMER, self.timer_tick, self.timer)
        self.timer.Start(MainFrame.sleep_time*1000)  # x100 milliseconds
        self.Bind(event=wx.EVT_CLOSE, handler=self.on_close, source=self)

    def __set_properties(self):
        self.SetTitle("Health Monitor GUI")

    def setup_layout(self):
        frame_sizer = wx.BoxSizer(wx.VERTICAL)
        for i, container in enumerate(self.src_panels.itervalues()):
            if not i:
                frame_sizer.AddSpacer(5)
            row_sizer = wx.BoxSizer(wx.HORIZONTAL)
            id_sizer = wx.BoxSizer(wx.HORIZONTAL)
            air_sizer = wx.BoxSizer(wx.HORIZONTAL)
            acc_sizer = wx.BoxSizer(wx.HORIZONTAL)
            ecg_sizer = wx.BoxSizer(wx.HORIZONTAL)
            hr_sizer = wx.BoxSizer(wx.HORIZONTAL)
            o2_sizer = wx.BoxSizer(wx.HORIZONTAL)
            temp_sizer = wx.BoxSizer(wx.HORIZONTAL)
            # Organize id data
            id_sizer.Add(container['sex_icon'], 0, wx.EXPAND, 0)
            id_sizer.Add(container['name_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(id_sizer, 1, wx.EXPAND, 0)
            # Organize air flow data
            air_sizer.Add(container['air_flow_icon'], 0, wx.EXPAND, 0)
            air_sizer.Add(container['air_flow_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(air_sizer, 1, wx.EXPAND, 0)
            # Organize acceleration magnitude data
            acc_sizer.Add(container['acc_magn_icon'], 0, wx.EXPAND, 0)
            acc_sizer.Add(container['acc_magn_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(acc_sizer, 1, wx.EXPAND, 0)
            # Organize ECG V1 data
            ecg_sizer.Add(container['ecg_v1_icon'], 0, wx.EXPAND, 0)
            row_sizer.Add(ecg_sizer, 1, wx.EXPAND, 0)
            # Organize heart rate data
            hr_sizer.Add(container['heart_rate_icon'], 0, wx.EXPAND, 0)
            hr_sizer.Add(container['heart_rate_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(hr_sizer, 1, wx.EXPAND, 0)
            # Organize SPO2 data
            o2_sizer.Add(container['o2_icon'], 0, wx.EXPAND, 0)
            o2_sizer.Add(container['o2_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(o2_sizer, 1, wx.EXPAND, 0)
            # Organize temperature data
            temp_sizer.Add(container['temperature_icon'], 0, wx.EXPAND, 0)
            temp_sizer.Add(container['temperature_lbl'], 0, wx.EXPAND, 0)
            row_sizer.Add(temp_sizer, 1, wx.EXPAND, 0)
            # add details button
            row_sizer.Add(container['details_btn'], 0, wx.EXPAND, 0)
            frame_sizer.Add(row_sizer, 1, wx.EXPAND, 0)

        self.SetSizer(frame_sizer)
        frame_sizer.Fit(self)
        self.MinSize = frame_sizer.Size
        self.SetAutoLayout(True)
        self.Layout()

    def on_details_click(self, evt):
        address = evt.EventObject.Name
        if address not in self.active_plots:
            container = self.src_panels[address]
            plot_lens = {
                     'air_flow_len': container['air_flow_poll_depth'] * 4,
                     'acc_magn_len': container['acc_magn_poll_depth'] * 4,
                     'ecg_v1_len': container['ecg_v1_poll_depth'] * 4,
                     'heart_rate_len': container['heart_rate_poll_depth'] * 4,
                     'o2_len': container['o2_poll_depth'] * 4,
                     'temperature_len': container['temperature_poll_depth'] * 4
                         }
            details_frame = DetailsFrame(
                            self, name=container['name'], address=address, 
                            capture_file_name=self.details_capture_file_name,
                            sleep_time=MainFrame.sleep_time,
                            **plot_lens)
            self.Bind(event=wx.EVT_CLOSE, handler=self.on_details_close,
                      source=details_frame)
            details_frame.Show()
            self.active_plots[address] = details_frame

    def on_details_close(self, evt):
        details_frame = evt.EventObject
        self.Unbind(event=wx.EVT_CLOSE, handler=self.on_details_close,
                    source=details_frame)
        del self.active_plots[details_frame.address]
        details_frame.Destroy()

    def on_close(self, evt):
        self.timer.Stop()
        self.Destroy()

    def select_alrm_img(self, name_prefix, name_sufix, value):
        if np.isnan(value):
            color = 'gray'
            code = 0
        elif value < self.yellow_alrm_thrsh:
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

    def update_var_lbl_icon(self, var_name, var_values, alarms, container,
                            format_str='{}', has_lbl=True):
        if len(alarms) > 0:
            alarm_lvls = zip(*alarms)[1]
        else:
            alarm_lvls = [np.nan]
        image, code = self.select_alrm_img('images/%s-icon-' % var_name,
                                           '.png', np.max(alarm_lvls))
        wx.CallAfter(container['%s_icon' % var_name].SetBitmap, image)
        if has_lbl:
            wx.CallAfter(container['%s_lbl' % var_name].SetLabel,
                         format_str.format(var_values[-1][1]))
        return code

    def timer_tick(self, evt):
        init = datetime.now()
        alarms = json.loads(self.proxy.get_all_alarms(self.sleep_time))
        for container in self.src_panels.itervalues():
            address = container['address']
            params = json.dumps({'period': 2})
            vars_values = json.loads(container['proxy'].get_data(params))
            vars_values = {k: [(dateutil_parse(i[0]), i[1]) for i in v]
                           for k, v in vars_values.iteritems()}
            try:
                codes = [0] * 6
                # update air flow values
                codes[0] = self.update_var_lbl_icon(
                                        var_name='air_flow',
                                        var_values=vars_values['air_flow'],
                                        alarms=alarms[address]['air_flow'],
                                        container=container,
                                        format_str='{:6.2f}'
                                                              )
                # update acceleration values
                codes[1] = self.update_var_lbl_icon(
                                        var_name='acc_magn',
                                        var_values=vars_values['acc_magn'],
                                        alarms=alarms[address]['acc_magn'],
                                        container=container,
                                        format_str='{:3.2f}'
                                                              )
                # update ecg values
                codes[2] = self.update_var_lbl_icon(
                                        var_name='ecg_v1',
                                        var_values=vars_values['ecg_v1'],
                                        alarms=alarms[address]['ecg_v1'],
                                        container=container,
                                        has_lbl=False
                                                            )
                # update heart rate values
                codes[3] = self.update_var_lbl_icon(
                                    var_name='heart_rate',
                                    var_values=vars_values['heart_rate'],
                                    alarms=alarms[address]['heart_rate'],
                                    container=container,
                                    format_str='{:3.0f}'
                                                                )
                # update SPO2 values
                codes[4] = self.update_var_lbl_icon(
                                            var_name='o2',
                                            var_values=vars_values['o2'],
                                            alarms=alarms[address]['o2'],
                                            container=container,
                                            format_str='{:4.2f}'
                                                        )
                # update temperature values
                codes[5] = self.update_var_lbl_icon(
                                    var_name='temperature',
                                    var_values=vars_values['temperature'],
                                    alarms=alarms[address]['temperature'],
                                    container=container,
                                    format_str='{:4.2f}'
                                                                 )
                # update sex image
                alarm_code = sum(codes) / len(codes)
                image, _ = self.select_alrm_img('images/%s-icon-' %
                                                    container['sex'],
                                                '.png', alarm_code)
                wx.CallAfter(container['sex_icon'].SetBitmap, image)
                if address in self.active_plots:
                    # get variable's values without timestamps
                    vars_values = {k: zip(*v)[1]
                                   for k, v in vars_values.items()
                                   if v}
                    frame = self.active_plots[address]
                    filename = frame.capture_file_name
                    wx.CallAfter(self.active_plots[address].update_plots,
                                 **vars_values)
                    utils.capture_window(frame, filename)
            except ConnectionFailed as e:
                print('Could not retrieve data from {}. Error: {}'.
                      format(address, e))
            except Exception as e:
                print('An exception occurred. Error: {}. vars_values: {}'.
                      format(e, vars_values))
        utils.capture_window(self, self.main_capture_file_name)


config = ConfigParser.RawConfigParser()
dirname = os.path.dirname(os.path.abspath(sys.argv[0]))
cfg_filename = os.path.join(dirname, 'health_monitor_gui.cfg')
config.read(cfg_filename)
MainFrame.monitor_address = config.get('Monitor_GUI', 'monitor_address')
MainFrame.yellow_alrm_thrsh = config.getfloat('Monitor_GUI',
                                              'yellow_alrm_thrsh')
MainFrame.red_alrm_thrsh = config.getfloat('Monitor_GUI', 'red_alrm_thrsh')
MainFrame.sleep_time = config.getint('Monitor_GUI', 'sleep_time')
MainFrame.main_capture_file_name = config.get('Monitor_GUI',
                                              'main_capture_file_name')
MainFrame.details_capture_file_name = config.get('Monitor_GUI',
                                                 'details_capture_file_name')

