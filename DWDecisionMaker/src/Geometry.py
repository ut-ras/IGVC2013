#!/usr/bin/env python

import math

class GLib:
    @staticmethod
    def euclid(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))

    @staticmethod
    def boundAngle0to2Pi(angle):
        return angle - math.floor(angle/(2*math.pi))*2*math.pi

    @staticmethod
    def angleDif(ang1, ang2):
        res = GLib.boundAngle0to2Pi(ang1) - GLib.boundAngle0to2Pi(ang2)

        if res > math.pi:
            res -= math.pi*2
        elif res < -math.pi:
            res += math.pi*2

        return res

    @staticmethod
    def calcTrajectoryStep(
        cur_x,
        cur_y,
        cur_dir,
        linear,
        angular,
        arclen,
        new_dir
        ):

        if abs(angular) < 1e-2:
            x = cur_x + arclen*math.cos(cur_dir)
            y = cur_y + arclen*math.sin(cur_dir)
        elif angular < 0:
            beta = float(cur_dir - new_dir)
            R = arclen/beta

            x = cur_x + R*math.cos(cur_dir - math.pi/2.0) \
                      + R*math.cos(cur_dir + math.pi/2.0 - beta)
            y = cur_y + R*math.sin(cur_dir - math.pi/2.0) \
                      + R*math.sin(cur_dir + math.pi/2.0 - beta)
        elif angular > 0:
            beta = float(new_dir - cur_dir)
            R = arclen/beta

            x = cur_x + R*math.cos(cur_dir + math.pi/2.0) \
                      + R*math.cos(cur_dir - math.pi/2.0 + beta)
            y = cur_y + R*math.sin(cur_dir + math.pi/2.0) \
                      + R*math.sin(cur_dir - math.pi/2.0 + beta)

        return [x, y, new_dir]

    @staticmethod
    def calcTrajectoryStepFromArc(
        cur_x,
        cur_y,
        cur_dir,
        linear,
        angular,
        arclen
        ):

        if abs(linear) <= 1e-6:
            raise Exception("exception: linear velocity cannot be zero here!")

        new_dir = cur_dir + (arclen/linear)*angular

        return GLib.calcTrajectoryStep(
            cur_x,
            cur_y,
            cur_dir,
            linear,
            angular,
            arclen,
            new_dir
            )

    @staticmethod
    def calcTrajectoryStepFromTime(
        cur_x,
        cur_y,
        cur_dir,
        linear,
        angular,
        dt
        ):

        arclen = dt*linear
        new_dir = cur_dir + dt*angular

        return GLib.calcTrajectoryStep(
            cur_x,
            cur_y,
            cur_dir,
            linear,
            angular,
            arclen,
            new_dir
            )

print "glib compiles!"

