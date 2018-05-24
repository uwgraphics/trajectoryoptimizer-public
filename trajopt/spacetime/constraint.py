__author__ = ['gleicher', 'cbodden']

"""
An abstract constraint class.

Implements a constraints for a specific unit of time. So, it can be used for
IK for spacetime, you need to associate it with a specific time frame. It can
have multiple scalar constraints, so it outputs 2 lists (eqs, ineqs). Note
that this is the same as Robot.constraint.
"""


class Constraint:
    def __init__(self, _eqs, _ineqs, _noZ, _usesPoints=1, _usesState=1, _usesPointDerivatives=0,
                 _usesStateDerivatives=0):
        self.noZ = _noZ
        self.eqs = _eqs
        self.ineqs = _ineqs

        # do we use point information?
        self.usesPoints = _usesPoints
        self.usesPointDerivatives = _usesPointDerivatives

        # do we use state?
        self.usesState = _usesState
        self.usesStateDerivatives = _usesStateDerivatives

    def getConstInfo(self):
        return self.numConstraints, self.cUBounds, self.cLBounds

    def constraint(self, **kwargs):
        raise NotImplemented
