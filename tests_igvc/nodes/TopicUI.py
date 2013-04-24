#!/usr/bin/python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('tests_igvc')
import rospy, wx, time, math
from sensor_msgs.msg import LaserScan,Image
from vn_200_imu.msg import vn_200_accel_gyro_compass
from PSoC_Listener.msg import PSoC

sensors = { 
    'Camera':   {'topic': "usb_cam/image_raw",  
                 'kind': Image,     
                 'package_node': "usb_cam/usb_cam_node"},
    
    'VN 200':   {'topic': "vn_200_accel_gyro_compass",       
                 'kind': vn_200_accel_gyro_compass,   
                 'package_node': "vn_200_imu/vn_200_node"}, 
    
    'Sonar':    {'topic': "sonar_data",        
                 'kind': LaserScan, 
                 'package_node': "SonarArray/SonarArray.py"}, 
    
    'PSoC':     {'topic': "psoc_data",         
                 'kind': PSoC,      
                 'package_node': "PSoC_Listener/PSoc_Listener.py"}
    }


class Example(wx.Frame):
  
    def __init__(self, parent, title):
        super(Example, self).__init__(parent, title=title, size=(650, 40 + 30*len(sensors)))
            
        self.InitUI()
        self.Centre()
        self.Show()
        
    def InitUI(self):
        vbox = wx.BoxSizer(wx.VERTICAL)

        gs = wx.GridSizer(4, 3, 5, 5)
        items = [
            (wx.StaticText(self, label='Topic'), wx.EXPAND),
            (wx.StaticText(self, label='Last heard'), wx.EXPAND),
            (wx.StaticText(self, label='package/node'), wx.EXPAND)
            ]
    
        self.topicElements = []

        for name in sensors:
            timeLbl = wx.StaticText(self, label='infinity')
            sensors[name]['elem'] = timeLbl

            arr = [
                (wx.StaticText(self, label=sensors[name]['topic']), wx.EXPAND),
                (timeLbl, wx.EXPAND),
                (wx.StaticText(self, label=sensors[name]['package_node']), wx.EXPAND)
                ]

            items.extend(arr)
        
        gs.AddMany(items)

        vbox.Add(gs, proportion=1, flag=wx.EXPAND)
        self.SetSizer(vbox)

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.update, self.timer)
        self.timer.Start(500)

    def update(self, event):
        for name in sensors:
            lastheard = sensors[name]['lastheard']
            curTime = time.time()
            if lastheard == -1:
                sensors[name]['elem'].SetLabel("infinity ago!")
            else:
                sensors[name]['elem'].SetLabel(str(round(10*(curTime - lastheard))/10) + " seconds ago")

def make_callback(sensor_name):
    def callback(data):
        # rospy.loginfo("saw " + sensor_name + " data!")
        sensors[sensor_name]['lastheard'] = time.time()

    return callback

if __name__ == '__main__':
    rospy.init_node('connectivity_tester')

    for name in sensors:
        topic_name = sensors[name]['topic']
        data_type = sensors[name]['kind']
        callback = make_callback(name)
        rospy.Subscriber(topic_name, data_type, callback)

        sensors[name]['lastheard'] = -1

    app = wx.App()
    ex = Example(None, title='Topic UI')

    app.MainLoop()

    




