#!/usr/bin/env python
import math, pygame

COLOR_STR_TO_RGB = {
    "black": (0x00, 0x00, 0x00),

    "gray": (0x80, 0x80, 0x80),
    "lightGray": (0xD3, 0xD3, 0xD3),
    "darkGray": (0xA9, 0xA9, 0xA9),

    "blue": (0x00, 0x00, 0xFF),
    "lightBlue": (0xAD, 0xD8, 0xE6),
    "green": (0x00, 0xFF, 0x00),
    "lightGreen": (0x90, 0xEE, 0x90),
    "darkGreen": (0x00, 0x64, 0x00),
    "red": (0xFF, 0x00, 0x00)
}

VECTOR_COLOR = "black"
ARROW_COLOR = "darkGray"

BG_RGB = COLOR_STR_TO_RGB["lightGray"]
AXIS_RGB = COLOR_STR_TO_RGB["gray"]

ARROW_LENGTH = 0.3

class Plotter:
    # expecting originx and originy to be between 0 and 1
    # where (0,0) would be the bottom-left of the canvas, and
    # (1,1) would be the top-right.
    def __init__(self, window, width, height, originx, originy, scalex, scaley):
        self.window = window
        self.background = pygame.Surface(self.window.get_size())
        self.background = self.background.convert()
        self.background.fill(BG_RGB)

        self.width = width
        self.height = height
        self.originx = originx
        self.originy = originy
        self.scalex = float(scalex)
        self.scaley = float(scaley)

    def plotToCanvasX(self, x):
        return x*self.scalex + self.originx*self.width

    def plotToCanvasY(self, y):
        return self.height - (y*self.scaley + self.originy*self.height)

    def line(self, x1, y1, x2, y2, color, width):
        color = COLOR_STR_TO_RGB[color]

        x1 = self.plotToCanvasX(x1)
        y1 = self.plotToCanvasY(y1)
        x2 = self.plotToCanvasX(x2)
        y2 = self.plotToCanvasY(y2)

        pygame.draw.line(
            self.window,
            color,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            width
            )

    def arc(self, x, y, radius, color, width):
        color = COLOR_STR_TO_RGB[color]

        x = self.plotToCanvasX(x)
        y = self.plotToCanvasY(y)

        pygame.draw.circle(
            self.window,
            color,
            (int(x), int(y)),
            int(radius),
            width
            )

    def getPlotCoords(self, x, y):
        return [
            (x - self.originx*self.width)/self.scalex,
            (self.height - y - self.originy*self.height)/self.scaley
        ]

    # where ratioxy = 1 will show a tick on the axis every 1 unit
    #               = 2 will shows every 2 units, etc
    def drawAxises(self, ratiox, ratioy):
        color = AXIS_RGB

        height = self.height
        width = self.width
        originx = self.originx
        originy = self.originy
        scalex = self.scalex
        scaley = self.scaley

        pygame.draw.line(
            self.window,
            color,
            (0, height - originy*height),
            (width, height - originy*height),
            3
            )

        pygame.draw.line(
            self.window,
            color,
            (originx*width, 0),
            (originx*width, height),
            3
            )

        for x in range(int(originx*width), int(width+ratiox*scalex), int(ratiox*scalex)):
            pygame.draw.line(
                self.window,
                color,
                (x, height - originy*height),
                (x, height - originy*height - 5),
                3
                )

        for x in range(int(originx*width), int(0-ratiox*scalex), int(-ratiox*scalex)):
            pygame.draw.line(
                self.window,
                color,
                (x, height - originy*height),
                (x, height - originy*height - 5),
                3
                )

        for y in range(int(originy*height), int(height+ratioy*scaley), int(ratioy*scaley)):
            pygame.draw.line(
                self.window,
                color,
                (originx*width, height - y),
                (originx*width + 5, height - y),
                3
                )

        for y in range(int(originy*height), int(0-ratioy*scaley), int(-ratioy*scaley)):
            pygame.draw.line(
                self.window,
                color,
                (originx*width, height - y),
                (originx*width + 5, height - y),
                3
                )

    def plotPoint(self, x, y, radius, color):
        radius *= self.scalex

        self.arc(x, y, radius, color, 0)

    def plotCircle(self, x, y, radius, color):
        radius *= self.scalex

        self.arc(x, y, radius, color, 1)

    def plotVector(self, x, y, dir, color=VECTOR_COLOR):
        self.line(
            x,
            y,
            x + 1e6*math.cos(dir),
            y + 1e6*math.sin(dir),
            color,
            1
            )

    def plotArrow(self, x, y, dir, color=ARROW_COLOR):
        endx = x + ARROW_LENGTH*math.cos(dir)
        endy = y + ARROW_LENGTH*math.sin(dir)

        self.line(x, y, endx, endy, color, 2)
        self.line(
            endx + ARROW_LENGTH/2.0*math.cos(dir + 4*math.pi/5.0),
            endy + ARROW_LENGTH/2.0*math.sin(dir + 4*math.pi/5.0),
            endx,
            endy,
            color,
            2
            )
        self.line(
            endx + ARROW_LENGTH/2.0*math.cos(dir - 4*math.pi/5.0),
            endy + ARROW_LENGTH/2.0*math.sin(dir - 4*math.pi/5.0),
            endx,
            endy,
            color,
            2
            )

    def clear(self):
        self.window.blit(self.background, (0, 0))

    def display(self):
        pygame.display.flip()

