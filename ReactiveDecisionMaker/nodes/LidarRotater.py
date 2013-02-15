import math, pygame

SIZEX = 400
SIZEY = 400
DISTANCE_GUI_SCALE = 100

MAX_VAL_THREASHOLD = 1e-3 # precision around max
MIN_VAL = 2e-2 # if ranges are below this, assume they are actually max values

class LidarValue:
    def __init__(self, dist, angle):
        self.dist = dist
        self.angle = angle

pi2 = math.pi*2.0
def boundAngleTo2PI(angle):
    return (angle%pi2 + pi2)%pi2;

class LidarProcessor:
    def __init__(self, SHOW_GRAPHICS=True):
        self.rotatedLidar = None
        self.SHOW_GRAPHICS = SHOW_GRAPHICS

        if self.SHOW_GRAPHICS:
            pygame.init() 
            self.window = pygame.display.set_mode((SIZEX, SIZEY)) 

            self.background = pygame.Surface(self.window.get_size())
            self.background = self.background.convert()
            self.background.fill((0, 0, 0))

            self.center = (SIZEX/2, SIZEY/2)

    def drawLidar(self):
        self.window.blit(self.background, (0,0))

        for i in range(len(self.rotatedLidar)):
            dist = self.rotatedLidar[i].dist
            angle = self.rotatedLidar[i].angle
            x = self.center[0] + DISTANCE_GUI_SCALE*dist*math.cos(angle)
            y = self.center[1] + DISTANCE_GUI_SCALE*dist*math.sin(angle)

            pygame.draw.line(self.window, (255, 255, 255), self.center, (x, SIZEY-y))

        pygame.display.flip() 

    def shortenAndCorrectScan(self, data, maxval):
        if self.shortenedLidar == None:
            self.shortenedLidar = [LidarValue(0,0) for i in range(len(data.ranges))]

        for i in range(len(data.ranges)) :
            dist = data.ranges[i]
            angle = i*data.angle_increment - math.pi/2.0
            
            dist = min(dist, maxval)

            if dist < MIN_VAL:
                dist = maxval

            self.shortenedLidar[i].dist = dist
            self.shortenedLidar[i].angle = angle

        if self.SHOW_GRAPHICS:
            self.drawLidar();

        return self.rotatedLidar

    def rotateScan(self, lidarValues, heading):
        for i in range(len(data.ranges)) :
            lidarValues[i].angle = boundAngleTo2PI(lidarValues[i].angle + heading)

        if self.SHOW_GRAPHICS:
            self.drawLidar();

        return self.rotatedLidar
            
