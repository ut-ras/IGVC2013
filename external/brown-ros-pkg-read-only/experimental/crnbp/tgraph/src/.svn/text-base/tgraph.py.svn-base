#!/usr/bin/env python
import math
import numpy
import mtTkinter as Tkinter

#
# n.b. Note the frequent invocations of Tkinter.update().  Without these the
# graphics only appear when the program can catch its breath, or when tk.mainloop()
# is invoked.
#
class Tgraph:
    # The height and width of the canvas widget
    height = 500
    width = 500
    # The drawable area seems to be slightly smaller than the actual size.  The cheat
    # value is the width (in pixels) of that undrawable border.
    cheat = 3
    # These are global variables for holding tk constructs.
    master = 0
    w = []
    cur = 0
    tick = 4

    def __init__(self, width, height):
    # Adjust the size to accommodate the non-plotting areas around the edge.
        self.height = height
        self.width = width
        self.master = Tkinter.Tk()
        self.w.append(Tkinter.Canvas(self.master, width=self.width, height=self.height))

        self.hfake = self.height - (2 * self.cheat)
        self.wfake = self.width - (2 * self.cheat)

        self.xmin = 0.0 ; self.ymin = 0.0
        self.xmax = 1.0 ; self.ymax = 1.0
        self.xscale = self.xmax - self.xmin ; self.yscale = self.ymax - self.ymin

        # These two lists keep track of the ids of objects drawn on the canvas
        # on the theory that we might use them to selectively erase stuff.
        self.objectList = []
        self.axesList = []

        # Draw a nice border
        self.axesList.append(self.w[self.cur].create_rectangle(self.cheat, self.cheat,
                                                               self.width - self.cheat,
                                                               self.height - self.cheat,
                                                               outline = "red"))
        self.w[self.cur].update()

    def new_graph(self):
        # Add the new graph to the array of canvases we're supporting.
        self.w.append(Tkinter.Canvas(self.master, width=self.width, height=self.height))
        self.cur += 1
        self.axesList.append(self.w[self.cur].create_rectangle(self.cheat, self.cheat,
                                                               self.width - self.cheat,
                                                               self.height - self.cheat,
                                                               outline = "red"))
        self.w[self.cur].pack()
        self.w[self.cur].update()


    def make_xticks(self, xvals):
        for xval in xvals:
            # Calculate the screen coordinates.
            xpos = self.screen_x(xval)
            # Tick marks on bottom
            self.axesList.append(\
                self.w[self.cur].create_line(xpos, self.cheat,
                                             xpos, self.cheat + 3 * self.tick,
                                             fill = "blue"))
            # Tick marks on top
            self.axesList.append(\
                self.w[self.cur].create_line(xpos, self.cheat + self.hfake,
                                             xpos, self.cheat + self.hfake - 3 * self.tick,
                                             fill = "blue"))
            # Labels
            self.axesList.append(\
                self.w[self.cur].create_text(xpos,
                                             self.cheat + self.hfake - 5 * self.tick,
                                             text = "%.2f" % (xval,) ))

    def make_yticks(self, yvals):
        for yval in yvals:
            # Calculate screen coordinates
            ypos = self.screen_y(yval)
            # Ticks on left
            self.axesList.append(\
                self.w[self.cur].create_line(self.cheat, ypos,
                                             self.cheat + 3 * self.tick, ypos,
                                             fill = "blue"))
            # Ticks on right
            self.axesList.append(\
                self.w[self.cur].create_line(self.wfake + self.cheat, ypos,
                                             self.wfake + self.cheat - 3 * self.tick, ypos,
                                             fill = "blue"))
            # Labels on left
            self.axesList.append(\
                self.w[self.cur].create_text(self.cheat + 6 * self.tick,
                                             ypos, text = "%.2f" % (yval,) ))

    def screen_x(self, x):
        return self.cheat + self.wfake * ((x - self.xmin) / self.xscale)

    def screen_y(self, y):
        return self.cheat + self.hfake - (self.hfake * ((y - self.ymin) / self.yscale))

    def screen_coords(self, x, y):
        x = self.screen_x(x)
        y = self.screen_y(y)
        return (x,y)

    def round(self, x):
        if (x > math.floor(x) + 0.5):
            return(math.ceil(x))
        else:
            return(math.floor(x))

    # ptype = 1, crosses, ptype = 2, little circles, scaled by cindex
    def draw_scatter(self, earray, xindex, yindex, cindex, ptype, recalc=True, origin=True):
        if recalc:
            self.xmax = earray[:,xindex].max() ; self.xmin = earray[:,xindex].min()
            self.ymax = earray[:,yindex].max() ; self.ymin = earray[:,yindex].min()
            # This one affects the color.
            if (cindex >= 0):
                self.cmax = earray[:,cindex].max() ; self.cmin = earray[:,cindex].min()

            if origin:
                self.xmax = max(self.xmax, 0.0) ; self.xmin = min(self.xmin, 0.0)
                self.ymax = max(self.ymax, 0.0) ; self.ymin = min(self.ymin, 0.0)
            self.xscale = self.xmax - self.xmin ; self.yscale = self.ymax - self.ymin
            if (self.xscale * self.yscale == 0):
                self.xscale = max(self.xscale, self.yscale); self.yscale = self.xscale


            # Unless the two scales are wildly different, use the larger
            # one for both
            if (math.fabs(math.log(self.xscale) - math.log(self.yscale)) < 2.5):
                self .xscale = max(self.xscale, self.yscale)
                self.yscale = self.xscale

        # Figure out the ranges to use.
        xRange = self.xmax - self.xmin; yRange = self.ymax - self.ymin

        xinterval = math.pow(10, self.round(math.log(self.xscale, 10)))/4
        yinterval = math.pow(10, self.round(math.log(self.yscale, 10)))/4

        # Basic tick size, in pixels.
        self.tick = 4

        self.plot_points(earray, xindex, yindex, cindex, ptype)

        # Mark the origin
        # Find the screen coordinates of the origin.
        if origin:
            (xzero, yzero) = self.screen_coords(0.0, 0.0)

            self.axesList.append(\
                self.w[self.cur].create_line(xzero - 5 * self.tick, yzero,
                                             xzero + 5 * self.tick, yzero,
                                             fill="blue"))
            self.axesList.append(\
                self.w[self.cur].create_line(xzero, yzero - 5 * self.tick,
                                             xzero, yzero + 5 * self.tick,
                                             fill="blue"))
            self.axesList.append(\
                self.w[self.cur].create_oval(xzero - 2, yzero - 2,
                                             xzero + 2, yzero + 2, fill="blue"))

        # Apply tick marks
        self.make_xticks(
            numpy.arange(math.floor(self.xmin/xinterval) * xinterval, self.xmin + self.xscale, xinterval))
        self.make_yticks(
            numpy.arange(math.floor(self.ymin/yinterval) * yinterval, self.ymin + self.yscale, yinterval))

        # If all the points are the same (errors too small?), draw a
        # little circle around them, to help find it.
        if ((math.fabs(xRange) < 0.01) and (math.fabs(yRange) < 0.01)):
            print "compact, circle at: (", earray[0][xindex], earray[0][yindex], ")"
            (xspot, yspot) = self.screen_coords(earray[0][xindex], earray[0][yindex])
            self.objectList.append(\
                self.w[self.cur].create_oval(xspot - 10, yspot - 10,
                                             xspot + 10, yspot + 10))

        self.w[self.cur].pack()
        self.w[self.cur].update()

    def plot_points(self, earray, xindex, yindex, cindex, ptype):
        # Sort through the array and plot each point.
        for point in earray:
            self.plot_point(point, xindex, yindex, cindex, ptype)
        self.w[self.cur].update()


    def plot_point(self, point, xindex, yindex, cindex, ptype):
        if (ptype == "s"):
            self.plot_point_cross(point, xindex, yindex, cindex)
        elif (ptype == "c"):
            self.plot_point_circle(point, xindex, yindex, cindex)

    # colval is a scale set up when the scatter plot was initialized.
    def plot_point_circle(self, point, xindex, yindex, cindex,
                          fill = True, colval=-999):
        (x, y) = self.screen_coords(point[xindex], point[yindex])
        cval = point[cindex]
        if (colval == -999):
            if (self.cmax == self.cmin):
                col = 255
            else:
                col = min(255,max(0,int(255*(cval - self.cmin)/(self.cmax-self.cmin))))
        else:
            col = min(255,max(0,int(255*(colval - self.cmin)/(self.cmax-self.cmin))))
        fillcolor = "#%02X%02X%02X" % (col, 0, int(255-col))

        if fill:
            c = self.w[self.cur].create_oval(x - cval, y - cval, x + cval, y + cval, \
                                             fill = fillcolor, outline=fillcolor)
        else:
            c = self.w[self.cur].create_oval(x - cval, y - cval, x + cval, y + cval, \
                                             fill = "", outline=fillcolor)

        self.objectList.append(c)
        self.w[self.cur].update()
        return c

    def plot_point_cross(self, point, xindex, yindex, cindex):
        (x, y) = self.screen_coords(point[xindex], point[yindex])
        if (cindex >= 0):
            if (self.cmax == self.cmin):
                cval = 0
            else:
                cval = min(255,max(0,math.floor(255*(point[cindex] - self.cmin)/(self.cmax-self.cmin))))
            fillcolor = "#%02X%02X%02X" % (cval, 0, int(255-cval))
        else:
            fillcolor = "red"
        # Horizontal line.
        self.objectList.append(\
            self.w[self.cur].create_line(x - self.tick, y,
                                         x + self.tick, y, fill = fillcolor))
        # Vertical line.
        self.objectList.append(\
            self.w[self.cur].create_line(x, y - self.tick,
                                         x, y + self.tick, fill = fillcolor))

    def plot_lines(self, earray, xindex, yindex, cindex, ptype):
        startpoints = earray[0:len(earray)-1]
        endpoints = earray[1:len(earray)]

        if len(earray) > 2:
            for startpoint, endpoint in zip(startpoints, endpoints):
                # Find screen coordinates.
                (xst, yst) = self.screen_coords(startpoint[xindex], startpoint[yindex])
                (xen, yen) = self.screen_coords(endpoint[xindex], endpoint[yindex])
                fillcolor = ptype
                line = self.w[self.cur].create_line(xst, yst, xen, yen, fill = fillcolor)
                self.objectList.append(line)
        elif len(earray) == 2:
            # Find screen coordinates.
            (xst, yst) = self.screen_coords(startpoints[0][xindex], startpoints[0][yindex])
            (xen, yen) = self.screen_coords(endpoints[0][xindex], endpoints[0][yindex])
            fillcolor = ptype
            line = self.w[self.cur].create_line(xst, yst, xen, yen, fill = fillcolor)
            self.objectList.append(line)
        else:
            return # error.

        return line

    # Plot a line from point to point+diff
    def plot_diff(self, tg, pt, diff, xi, yi, ci, fill):
        newpt = (pt[0] + diff[0], pt[1] + diff[1], pt[2] + diff[2])
        self.plot_lines(np.array((pt, newpt)), xi, yi, ci, fill)
        self.w[self.cur].update()


    def clearObjects(self, canv=-1):
        if canv == -1:
            canv = self.cur
        for obj in self.objectList:
            self.w[canv].delete(obj)

    def mainloop(self):
        self.master.mainloop()
