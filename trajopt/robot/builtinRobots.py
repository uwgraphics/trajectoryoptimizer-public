from trajopt.robot.robotInterface import Robot
from trajopt.robot.arm import Arm
from trajopt.utilities.robotOperations import *
from itertools import chain
import numpy as N

class Particle2DRobot(Robot):
    """
    Simplest possible test robot - a collection of 2D points
    Useful for testing various things
    """
    def __init__(self, _npoints):
        Robot.__init__(self, _npoints*2, _npoints, "Particle2D")
        self.noZ = True
        self.varnames = list(chain.from_iterable([["x%d"%i,"y%d"%i] for i in range(self.npoints)]))

    def __call__(self, state):
        return [ (state[i*2], state[i*2+1], 0) for i in range(self.npoints) ]

    def getFrames(self,state):
        """
        just call call - but returns a two things - the points (like call) and the frames
        :param state:
        :return: a list of points and a list of 3x3 matrices (in global coords)
        """
        pts = self(state)
        eye = N.eye(3)
        return pts, [eye for i in range(len(pts))]

    def constraint(self, **kwargs):
        return [],[]

class TwoLink(Robot):
    """
    Simple 2D articulated arm. Uses agles. Assumes unit segment lengths.
    """

    def __init__(self):
        Robot.__init__(self, 2, 3, "Arm2D")
        self.noZ = True

    def constraint(self, **kwargs):
        return [], []

    def getFrames(self,state):
        """
        just call call - but returns a two things - the points (like call) and the frames
        :param state:
        :return: a list of points and a list of 3x3 matrices (in global coords)
        """
        pts = self(state)
        eye = N.eye(3)
        return pts, [eye for i in range(len(pts))]

    def __call__(self, state):
        return [(0, 0),
                (cos(state[0]), sin(state[0])),
                (cos(state[0]) + cos(state[0] + state[1]), sin(state[0]) + sin(state[0] + state[1]))
                ]


class Reactor(Arm):
    def __init__(self, *args, **kwargs):
        self.joint_names = ['shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll']
        self.joint_axes = ['z', 'y', 'y', 'y', 'x']
        self.displacements = [(0, 0, 0.081), (0, 0, 0.0265), (-0.1445, 0, 0.0385), (0.1535, 0, 0), (0.071, 0, 0),
                              (0.0762, 0, 0)]
        self.rotations = [(0, 0, 0), (0, math.radians(-270), 0), (0, math.radians(-90), math.radians(180)), (0, 0, 0),
                          (0, 0, 0)]
        Arm.__init__(self,
                     self.joint_names,
                     displacements=self.displacements[1:],
                     dispOffset=self.displacements[0],
                     name="Reactor",
                     rotOffsets=self.rotations,
                     *args, **kwargs)


class Mico(Arm):
    def __init__(self, *args, **kwargs):
        self.displacements = [(0, 0, 0.1535), (0, 0, -0.1185), (0.29, 0, 0),
                              (0.123, 0, -0.00845), (0.037, 0, -0.06408), (0.037, 0, -0.06408), (0, 0, -.16)]
        self.rotations = [(math.radians(180), 0, 0), (math.radians(-90), math.radians(-90), 0),
                          (math.radians(180), 0, math.radians(180)), (0, math.radians(-90), 0),
                          (0, math.radians(60), math.radians(180)), (0, math.radians(60), math.radians(180))]
        Arm.__init__(self,
                     axes=['z'] * 6,
                     displacements=self.displacements[1:],
                     dispOffset=self.displacements[0],
                     name="Mico",
                     rotOffsets=self.rotations,
                     *args, **kwargs
                     )
        self.default = [-1.4870590852717527, 3.0559304792307405, 1.371559253533616, -2.0360851903259105,
                        1.4648866713001893, 1.311377006061951]

class UR5(Arm):
    def __init__(self,*args,**kwargs):
        self.displacements = [ (0, 0, 0.089159), (0, 0.13585, 0) , (0, -0.1197, 0.42500) ,
                      (0, 0, 0.39225) , (0, 0.093, 0) , (0, 0, 0.09465), (0,0.2423,0) ]
        self.rotations = [(0, 0, 0), (0, math.radians(90), 0),
                          (0, 0, 0), (0, math.radians(90), 0),
                          (0, 0, 0), (0, 0, 0)]

        Arm.__init__(self,
                     axes = ['z','y','y','y','z','y'],
                     isplacements=self.displacements[1:],
                     dispOffset=self.displacements[0],
                     name="UR5",
                     rotOffsets=self.rotations,
                     *args,**kwargs
                     )
        self.default = [0.0, -1.570796, 0.0, 0.0, 0.0, 0.0]