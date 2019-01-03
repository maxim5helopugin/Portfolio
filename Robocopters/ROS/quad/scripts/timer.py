"""Class to track timed operations and output the results."""
from __future__ import division, print_function
from time import time
from numpy import mean


class Timer:
    def __init__(self, name="Timer"):
        self.name = name
        self.durations = []
        self.on = False
        self.t0 = None
        self.t1 = None

    def start(self):
        self.t0 = time()
        self.on = True

    def end(self, flag=0):
        self.t1 = time()
        if not self.on:
            print("Timer was not on.")
            return
        self.on = False
        self.durations.append(self.t1 - self.t0)
        if flag == 1:  # print results
            self.output()
        if flag == 2:   # print verbose results
            self.output(1)

    def output(self, verbose=0):
        """Print timer results."""
        n = len(self.durations)
        if n > 1:
            x = mean(self.durations)
            total = sum(self.durations)
            print('(Timer: %s) avg time: %fs, n: %d, total time: %fs' % (self.name, x, n, total))
        else:
            print('(Timer: %s)' % self.name, self.durations)