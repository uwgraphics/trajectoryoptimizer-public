__author__ = ['gleicher', 'cbodden']

"""
A generic robot interface.

A robot is a function that takes a state vector and produces a list of
3D positions (points). The function produces ALL of the points on the
robot - rather than the specific ones needed. This is because we don't
know about caching, so we might as well compute everything once (no lazy
evaluation).
"""

"""
CB TODO

1) define "large" bounds constant
2) noZ doesn't make sense in the abstract class... move
3) don't define get frames here
"""
import numpy as N


class Robot:
    """
    Abstract class for making a robot. The actual base class doesn't do much.
    """

    def __init__(self, _nvars, _npoints, _name="", _cleanupCallback=None,
                 _stateUpperBounds=None, _stateLowerBounds=None, _stateDefault=None):
        self.npoints = _npoints
        self.nvars = _nvars
        self.__name__ = "Robot" if _name == "" else "Robot:%s" % _name
        self.varnames = ["v"] * self.nvars
        self.cleanupCallback = _cleanupCallback
        self.xUBounds = N.full(self.nvars, float(1000)) if _stateUpperBounds == None else _stateUpperBounds
        self.xLBounds = N.full(self.nvars, float(-1000)) if _stateLowerBounds == None else _stateLowerBounds
        self.default = N.zeros(self.nvars) if _stateDefault == None else _stateDefault

    def __call__(self, state):
        """
        allows the robot to be treated as a function:
            state -> list(points)
        :param state: a state vector (numpy array or list)
        :return: a list of points
        """
        raise NotImplementedError

    def getFrames(self, state):
        """
        just like call - but returns a two things - the points (like call) and the frames
        :param state: a state vector (numpy array or list)
        :return: a list of points and a list of 3x3 matrices (in global coords)
        """
        raise NotImplementedError

    def constraint(self, **kwargs):
        """
        returns the evaluation of 2 sets of constraint functions (eq, ineq)
        the first list should be eq 0
        the second list should be geq 0
        note: like the evaluation function, this should be able to take ad (or oovar)
        objects
        note that we take points - which is exactly self(state) - to have compatibility
        with the constraint functions
        :param **kwargs:
                t = The current frame
                state = The state vector at frame t
                points = The point vector at frame t
                frames = The transformation matrices for the robot at frame t
                stvel = The state velocities vector at frame t
                stacc = The state accelerations vector at frame t
                ptvel = The point velocities vector at frame t
                ptacc = The point accelerations vector at frame t
        :return: pair of lists of values
        """
        raise NotImplementedError
