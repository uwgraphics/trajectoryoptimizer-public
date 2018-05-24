__author__ = 'cbodden'

"""
A generic solver interface. Allows users to easily plug in new solvers
if needed. The interface is simple, a solver takes a spacetime problem
and provides a function to solve it. Any additional initialization, etc
is hidden behind the interface.
"""

from trajopt.spacetime.spacetime import Spacetime

class Solver:
    def __init__(self, _spacetimeProblem):
        self.spacetimeProblem = _spacetimeProblem

    def __call__(self):
        """
        allows the solver to be treated as a function:
        :return: a state sequence for the solved trajectory
        """
        raise NotImplementedError