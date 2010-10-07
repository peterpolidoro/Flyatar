#!/usr/bin/env python
import urllib2
import socket

hostname=socket.gethostname()

URL = "http://mosca.caltech.edu/update_ip/update_ip.cgi?%s"%(hostname,)
print "from ",URL
print
print urllib2.urlopen(URL).read()
