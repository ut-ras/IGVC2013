import rospy, pygame, math

class Graphics:
    def __init__(
            self,
            SIZEX=400,
            SIZEY=400,
            BACKGROUND_RGB=(255, 255, 255),
            AXIS_RGB=(0, 0, 0),
            POINT_RADIUS=1,
            POINT_WIDTH=1,
            XAXIS_LENGTH=3,
            YAXIS_LENGTH=3,
            ORIGIN_X=200,
            ORIGIN_Y=200
            ):
        pygame.init()

        self.window = pygame.display.set_mode((SIZEX, SIZEY))
        self.background = pygame.Surface(self.window.get_size())
        self.background = self.background.convert()
        self.background.fill(BACKGROUND_RGB)

        self.SIZEX = SIZEX
        self.SIZEY = SIZEY
        self.POINT_RADIUS = POINT_RADIUS
        self.POINT_WIDTH = POINT_WIDTH

        self.XSCALE = 2*SIZEX/float(XAXIS_LENGTH)
        self.YSCALE = 2*SIZEY/float(YAXIS_LENGTH)
        self.XAXIS_LENGTH = XAXIS_LENGTH
        self.YAXIS_LENGTH = YAXIS_LENGTH

        self.AXIS_RGB = AXIS_RGB
        self.ORIGIN_X = ORIGIN_X
        self.ORIGIN_Y = ORIGIN_Y

    def draw_axis(self):
        #
        # Draw each axis
        #
        pygame.draw.line(
            self.window,
            self.AXIS_RGB,
            (self.ORIGIN_X-5, 0),
            (self.ORIGIN_X-5, self.ORIGIN_Y),
            2
            )

        pygame.draw.line(
            self.window,
            self.AXIS_RGB,
            (0, self.ORIGIN_Y),
            (self.ORIGIN_X, self.ORIGIN_Y),
            2
            )

        #
        # Draw tick marks on each axis
        #
        for y in range(self.YAXIS_LENGTH + 1):
            y = self.coordsToPixels(self.ORIGIN_X, y - self.YAXIS_LENGTH/2)[1]
            pygame.draw.line(
                self.window,
                self.AXIS_RGB,
                (self.ORIGIN_X, y),
                (self.ORIGIN_X + 5, y),
                2
                )

        for x in range(self.XAXIS_LENGTH + 1):
            x = self.coordsToPixels(x - self.XAXIS_LENGTH/2, self.ORIGIN_Y)[0]
            pygame.draw.line(
                self.window,
                self.AXIS_RGB,
                (x, self.ORIGIN_Y - 5),
                (x, self.ORIGIN_Y),
                2
                )

    def coordsToPixels(self, x, y):
        x = x*self.XSCALE + self.ORIGIN_X
        y = y*self.YSCALE + self.ORIGIN_Y
        return (x, y)

    def plot(self, x, y, rgb_color=(0,0,255)):
        (x, y) = self.coordsToPixels(x, y)

        pygame.draw.circle(
                self.window,
                rgb_color,
                (int(x), int(self.SIZEY - y)),
                self.POINT_RADIUS,
                self.POINT_WIDTH
                )

    def plot_line(self, x1, y1, x2, y2, rgb_color=(0,0,255)):
        (x1, y1) = self.coordsToPixels(x1, y1)
        (x2, y2) = self.coordsToPixels(x2, y2)

        pygame.draw.line(
            self.window,
            rgb_color,
            (x1, self.SIZEY - y1),
            (x2, self.SIZEY - y2),
            2
            )

    def clear(self):
        self.window.blit(self.background, (0,0))

    def display(self):
        self.draw_axis()
        pygame.display.flip()

