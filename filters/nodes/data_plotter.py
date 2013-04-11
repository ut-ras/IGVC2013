#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, pygame

from vn_200_imu.msg import vn_200_accel_gyro_compass


X_OFFSET = 2.45331919332
Y_OFFSET = 1.564559527120

graphics = None

class Graphics:
    def __init__(
            self,
            SIZEX=400,
            SIZEY=400,
            BACKGROUND_RGB=(0, 0, 0),
            POINT_RGB=(0, 0, 255),
            POINT_RADIUS=1,
            POINT_WIDTH=1,
            XAXIS_LENGTH=10,
            YAXIS_LENGTH=10
            ):
        pygame.init()

        self.window = pygame.display.set_mode((SIZEX, SIZEY))
        self.background = pygame.Surface(self.window.get_size())
        self.background = self.background.convert()
        self.background.fill(BACKGROUND_RGB)

        self.SIZEX = SIZEX
        self.SIZEY = SIZEY
        self.POINT_RGB = POINT_RGB
        self.POINT_RADIUS = POINT_RADIUS
        self.POINT_WIDTH = POINT_WIDTH

        self.XSCALE = SIZEX/float(XAXIS_LENGTH)
        self.YSCALE = SIZEY/float(YAXIS_LENGTH)

        pygame.draw.line(
            self.window,
            (255, 255, 255),
            (SIZEX/2, 0),
            (SIZEX/2, SIZEY),
            2
            )

        for y in range(YAXIS_LENGTH + 1):
            y = self.coordsToPixels(SIZEX/2, y - YAXIS_LENGTH/2)[1]
            pygame.draw.line(
                self.window,
                (255, 255, 255),
                (SIZEX/2, y),
                (SIZEX/2 + 5, y),
                2
                )

        pygame.draw.line(
            self.window,
            (255, 255, 255),
            (0, SIZEY/2),
            (SIZEX, SIZEY/2),
            2
            )

        for x in range(XAXIS_LENGTH + 1):
            x = self.coordsToPixels(x - XAXIS_LENGTH/2, SIZEY/2)[0]
            pygame.draw.line(
                self.window,
                (255, 255, 255),
                (x, SIZEY/2 - 5),
                (x, SIZEY/2),
                2
                )
        """
        (x, y) = self.coordsToPixels(2.45331919332, 1.564559527120)

        # self.window.blit(self.background, (0,0))
        pygame.draw.circle(
                self.window,
                (255, 0, 0),
                (int(x), int(self.SIZEY - y)),
                5,
                5
                )
        """

        pygame.display.flip()

    def coordsToPixels(self, x, y):
        x = x*self.XSCALE + self.SIZEX/2
        y = y*self.YSCALE + self.SIZEY/2
        return (x, y)

    def plot(self, x, y):
        (x, y) = self.coordsToPixels(x, y)

        pygame.draw.circle(
                self.window,
                self.POINT_RGB,
                (int(x), int(self.SIZEY - y)),
                self.POINT_RADIUS,
                self.POINT_WIDTH
                )

        pygame.display.flip()

totalx = 0
totaly = 0
counter = 0.0
def vn_200_imu_callback(data):
    global graphics, X_OFFSET, Y_OFFSET
    graphics.plot(data.compass.x - X_OFFSET, data.compass.y - Y_OFFSET)

    global totalx, totaly, counter
    totalx += data.compass.x
    totaly += data.compass.y
    counter += 1.0
    print totalx/counter, totaly/counter

def init():
    rospy.init_node('data_plotter', anonymous=True)

    global graphics
    graphics = Graphics()

    rospy.Subscriber("vn_200_accel_gyro_compass", vn_200_accel_gyro_compass, vn_200_imu_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass

