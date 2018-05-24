__author__ = 'gleicher'

"""
A simple timer class.

-*- coding: utf-8 -*-
"""

import time


def ms(f):
    return round(f, 1)


class Timer():
    def __init__(self, _name="Timer"):
        self.name = _name
        self.count = 0
        self.total = 0
        self.min = -1
        self.max = -1
        self.create = time.time()
        self.lastEnd = -1
        self.lastStart = -1

    def start(self):
        t = time.time()
        self.lastStart = t

    def end(self):
        e = time.time()
        dur = e - self.lastStart
        self.lastEnd = e
        if (self.min < 0) or (self.min > dur):
            self.min = dur
        if dur > self.max:
            self.max = dur
        self.count += 1
        self.total += dur

    def stop(self):
        self.end()

    def __repr__(self):
        if self.count > 1:
            str = "<Timer %s(%g sec/%d) Avg:%g Elap:%g range[%g %g]>" % (self.name, ms(self.total), self.count,
                                                                         ms(float(self.total) / float(
                                                                             self.count)) if self.count > 0 else 0,
                                                                         ms(self.lastEnd - self.create),
                                                                         ms(self.min), ms(self.max)
                                                                         )
        elif self.count == 1:
            str = "<Timer %s(%g sec/%d)>" % (self.name, ms(self.total), self.count)
        else:
            str = "<Timer %s (0)>" % (self.name)
        return str

    def totalString(self):
        return "<Timer %s (%g s)>" % (self.name, self())

    def __call__(self):
        """
        :return: average time (in milliseconds)
        """
        return ms(float(self.total) / float(self.count))
