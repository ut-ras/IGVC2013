#!/usr/bin/env python
# A class for prepending the host name onto the topic and service names.
# Just a wrapper for os.uname, really, but it is convenient to encapsulate
# the process for changing the networking and for times that it isn't 
# really necessary.
#
import os

class HName(object):
    def __init__(self, usename=True):
        if usename:
            self.hname = os.uname()[1]
        else:
            self.hname = ""

    def topic(self, tname):
        """
        Prepends the hostname onto the input topic.
        """
        return self.hname + "/" + tname if self.hname else tname
