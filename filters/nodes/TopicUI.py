#!/usr/bin/python
# -*- coding: utf-8 -*-

import wx, time

class TopicData:
    def __init__(self, name, package_node):
        self.name = name
        self.package_node = package_node
        self.lastheard = -1  
        self.timeLbl = None

    def setTimeLbl(self, timeLbl):
        self.timeLbl = timeLbl

    def setTime(self):
        self.lastheard = time.clock()

    def update(self):
        if self.timeLbl != None:
            curTime = time.clock()
            if self.lastheard == -1:
                self.timeLbl.SetLabel("infinity!")
            else:
                self.timeLbl.SetLabel(str(curTime - self.lastheard))

TOPIC_LIST = [
    TopicData("/psoc_data", "PSoC_Listener/PSoc_Listener"),
    TopicData("/vel_data", "PSoC_Listener/PSoC_Velocities"),
    TopicData("/usb_cam/image_raw", "usb_cam/usb_cam_node"),
    ]

class Example(wx.Frame):
  
    def __init__(self, parent, title):
        super(Example, self).__init__(parent, title=title, size=(800, 40 + 30*len(TOPIC_LIST)))
            
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

        for topic in TOPIC_LIST:
            timeLbl = wx.StaticText(self, label='infinity')
            topic.setTimeLbl(timeLbl)

            arr = [
                (wx.StaticText(self, label=topic.name), wx.EXPAND),
                (timeLbl, wx.EXPAND),
                (wx.StaticText(self, label=topic.package_node), wx.EXPAND)
                ]

            items.extend(arr)
        
        gs.AddMany(items)

        vbox.Add(gs, proportion=1, flag=wx.EXPAND)
        self.SetSizer(vbox)

if __name__ == '__main__':
    app = wx.App()
    Example(None, title='Topic UI')
    app.MainLoop()

    




