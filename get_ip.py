#!/usr/bin/env python
import urllib2
import sys

try:
    hostname = sys.argv[1]
except IndexError:
    print 'No hostname given, all hostnames will be printed.'
    print
    hostname = None

URL = "http://mosca.caltech.edu/ips.txt"
lines = urllib2.urlopen(URL).read().splitlines()
for line in lines:
    data = line.split()
    if hostname is None:
        print data[0], data[1]
    elif hostname == data[0]:
        print data[1]
