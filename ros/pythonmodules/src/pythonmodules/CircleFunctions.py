#!/usr/bin/env python
from __future__ import division
import math

pi = math.pi

def mod_angle(angle):
    angle = angle%(2*pi)
    return angle

def circle_dist(start_angle,stop_angle):
    diff1 = mod_angle(stop_angle) - mod_angle(start_angle)
    if start_angle < stop_angle:
        diff2 = stop_angle - 2*math.pi - start_angle
    else:
        diff2 = 2*math.pi - start_angle + stop_angle
    abs_min = min(abs(diff1),abs(diff2))
    if abs_min == abs(diff1):
        return diff1
    else:
        return diff2

def radians_to_degrees(angle):
    return angle*180/pi

def degrees_to_radians(angle):
    return angle*pi/180

if __name__ == '__main__':
    pi = math.pi
    start_angle = [pi/4, pi/4, 7/4*pi, 7/4*pi]
    stop_angle = [0, 7/4*pi, 0, pi/4]
    for ang in range(len(start_angle)):
        start_ang = start_angle[ang]
        stop_ang = stop_angle[ang]
        diff = circle_dist(start_ang,stop_ang)
        print "start_angle = %s" % (str(radians_to_degrees(start_ang)))
        print "stop_angle = %s" % (str(radians_to_degrees(stop_ang)))
        print "diff = %s" % (str(radians_to_degrees(diff)))
