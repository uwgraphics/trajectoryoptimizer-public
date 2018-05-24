import trajopt.robot.robotInterface as ROBOT
from trajopt.utilities.robotOperations import *
from numbers import Number


# we model robot arms as a series of 1-axis joints, each with a rotation between them
# for now, the axes have to be X,Y or Z - this will need to change at some point
#
# a 2 axis joint would be a zero displacement - change later
#
# rotations first (so the "base" rotation is at the origin) - this would be easy to fix
#
# warning: this does everything with general 4x4 matrices - and is probably
# way inefficient
#
# the rotational offsets take things as X,Y,Z Euler angles - which is the way that ROS does
# it, so we can try to be compatible with ROS
#
# while i hate that this is AD aware, it is so much in the critical path
# that its worth making it fast
class Arm(ROBOT.Robot):
    def __init__(self, axes=[], displacements=[], rotOffsets=None, dispOffset=(0, 0, 0), name="Arm"):
        # we allow to specify the axes as "Z" - in which case all joints are Z
        if axes == "Z" or axes == "z":
            self.axes = ["Z"] * len(displacements)
            noZ = True
        else:
            self.axes = axes
            noZ = False

        self.varsPerJoint = 1

        # the displacements should be a list of tuples, but sometimes its convenient
        # to just give the X axis
        self.displacements = [((t, 0, 0) if isinstance(t, Number) else tuple(t)) for t in displacements]
        # create a list of the rotational offsets
        if rotOffsets == None:
            self.rotOffsets = None
        else:
            self.rotOffsets = [eulerTupleTo3x3(t) if not (t is None) else None for t in rotOffsets]

        # now we're ready to initialize
        ROBOT.Robot.__init__(self, _nvars=len(self.axes) * self.varsPerJoint, _npoints=len(self.axes) + 1, _name=name)
        self.noZ = noZ
        self.varnames = ["J%d.%c" % (jn, ax) for jn, ax in enumerate(self.axes)]
        self.cleanupCallback = deSpinCB  # not always what we want
        self.xUBounds = N.full(self.nvars, twopi)
        self.xLBounds = N.full(self.nvars, -twopi)
        self.default = N.full(self.nvars, .1)
        self.dispOffset = dispOffset

    def cleanupMode(self, mode="array"):
        if self.rep == "angle":
            if mode == "array":
                self.cleanupCallback = lambda x: despinArray(x, self.nvars)
            elif mode == "perframe":
                self.cleanupCallback = deSpinCB
            elif mode == None:
                self.cleanupCallback = None
            else:
                raise NameError("Bad Cleanup Mode")
        else:
            self.cleanupCallback = None

    def constraint(self, **kwargs):
        return [], []

    def __call__(self, state):
        """
        given the state vector, return all the points
        this is really performance critical for automatic differentiaiton
        so try to figure out if we need a fast path
        :param state:
        :return:
        """
        try:
            if state.dtype == object:
                do_ad = True
            else:
                do_ad = False
        except:
            do_ad = True  # be conservative

        pt = N.array(self.dispOffset)
        pts = [self.dispOffset]
        rot = N.eye(3)
        for i, axis in enumerate(self.axes):
            if self.varsPerJoint == 1:
                if do_ad == False:
                    s = math.sin(state[i])
                    c = math.cos(state[i])
                else:
                    s = sin(state[i])
                    c = cos(state[i])
            else:
                s, c = normSC(state[i * 2], state[i * 2 + 1])
            # since we know that the rot matrix doesn't change, and that
            # this is an affine thing, we can do this a bit more quickly
            # lmat = N.dot(rotMatrix(axis,s,c) , transMatrix(self.displacements[i]))
            if self.rotOffsets:
                if not (self.rotOffsets[i] is None):
                    rot = rot.dot(self.rotOffsets[i])
            rmat = rot3(axis, s, c)
            rot = rot.dot(rmat)
            pt = rot.dot(self.displacements[i]) + pt
            pts.append(pt)
        return pts

    def getFrames(self, state):
        """
        given the state vector, return all the points
        this is really performance critical for automatic differentiaiton
        so try to figure out if we need a fast path
        :param state:
        :return:
        """
        try:
            if state.dtype == object:
                do_ad = True
            else:
                do_ad = False
        except:
            do_ad = True  # be conservative

        pt = N.array(self.dispOffset)
        pts = [self.dispOffset]
        rot = N.eye(3)
        frames = [rot]
        for i, axis in enumerate(self.axes):
            if self.varsPerJoint == 1:
                if do_ad == False:
                    s = math.sin(state[i])
                    c = math.cos(state[i])
                else:
                    s = sin(state[i])
                    c = cos(state[i])
            else:
                s, c = normSC(state[i * 2], state[i * 2 + 1])
            # since we know that the rot matrix doesn't change, and that
            # this is an affine thing, we can do this a bit more quickly
            # lmat = N.dot(rotMatrix(axis,s,c) , transMatrix(self.displacements[i]))
            if self.rotOffsets:
                if not (self.rotOffsets[i] is None):
                    rot = rot.dot(self.rotOffsets[i])
            rmat = rot3(axis, s, c)
            rot = rot.dot(rmat)
            pt = rot.dot(self.displacements[i]) + pt
            pts.append(pt)
            frames.append(rot)
        return pts, frames
