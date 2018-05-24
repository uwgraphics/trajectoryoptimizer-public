__author__ = ['gleicher', 'cbodden']

"""
An abstract objective term class.

Implements an objective term, typically for all times.
"""


class ObjectiveTerm:
    """
    This class defines a term of an objective function.
    It is defined at each point in time.
    It can access the positions of the points and state variables and
    their derivatives.
    We need to know what it uses, so the outside caller can decide if its
    appropriate (for example, if it uses velocities, it isn't appropriate in
    an IK solver)
    """

    def __init__(self, _usesPoints=1, _usesState=1, _usesPointDerivatives=0, _usesStateDerivatives=0,
                 _meaningfulForIK=True):
        # do we use point information?
        self.usesPoints = _usesPoints
        self.usesPointDerivatives = _usesPointDerivatives

        # do we use state?
        self.usesState = _usesState
        self.usesStateDerivatives = _usesStateDerivatives

        # is this meaningful for IK - when its a frame extracted from a motion
        self.meaningfulForIK = _meaningfulForIK

    def __call__(self, **kwargs):
        """
        an objective function takes information about the state of the robot
        and the points

        it also gets derivative information about these things if it wants
        if the derivatives don't exist, None is passed, not a vector

        if all goes according to plan, if something uses a derivative it
        doesn't get called when it doesn't need it

        :param **kwargs:
                t = The current frame
                state = The state vector at frame t
                points = The point vector at frame t
                frames = The transformation matrices for the robot at frame t
                stvel = The state velocities vector at frame t
                stacc = The state accelerations vector at frame t
                ptvel = The point velocities vector at frame t
                ptacc = The point accelerations vector at frame t
        :return: The current value of the objective at time t for the given state
        information.
        """
        raise NotImplementedError
