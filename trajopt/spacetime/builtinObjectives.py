from trajopt.spacetime.objective import ObjectiveTerm
import trajopt.utilities.adInterface as AD
import numpy as N
import math


class StateValue(ObjectiveTerm):
    def __init__(self, baseVal=None):
        """
        objective function that minimizes the difference between the state
        vector and a provided base value.
        no base value makes the base state be zero
        :param baseVal:
        :return:
        """
        ObjectiveTerm.__init__(self, _usesState=1)
        self.base = baseVal

    def __call__(self, state, **kwargs):
        if self.base == None:
            return N.dot(state, state)
        else:
            diff = state - self.base
            return N.dot(diff, diff)


class StateVelocity(ObjectiveTerm):
    """
    minimize the velocity in state space (joint angles)
    """

    def __init__(self):
        ObjectiveTerm.__init__(self, _usesStateDerivatives=1)

    def __call__(self, stvel, **kwargs):
        v = N.dot(stvel, stvel)
        return v


class StateAcceleration(ObjectiveTerm):
    """
    minimize the accelerations in state space (smooth trajectories in joint angles)
    """

    def __init__(self):
        ObjectiveTerm.__init__(self, _usesStateDerivatives=2)

    def __call__(self, stacc, **kwargs):
        v = N.dot(stacc, stacc)
        return v


class PointVelocity(ObjectiveTerm):
    """
    minimize the velocity of an end-effector point (specify which one!)
    will make points go in a straight line
    """

    def __init__(self, _pt):
        ObjectiveTerm.__init__(self, _usesPointDerivatives=1)
        self.pt = _pt

    def __call__(self, ptvel, **kwargs):
        v = N.dot(ptvel[self.pt], ptvel[self.pt])
        return v


class PointAcceleration(ObjectiveTerm):
    """
    minimize the acceleration of an end-effector point (specify which one)
    should make the point follow a smooth path
    """

    def __init__(self, _pt):
        ObjectiveTerm.__init__(self, _usesPointDerivatives=2)
        self.pt = _pt

    def __call__(self, ptacc, **kwargs):
        v = N.dot(ptacc[self.pt], ptacc[self.pt])
        return v


class ElbowVelocity(ObjectiveTerm):
    """
    minimize the velocity of all joints
    allows for weighting, but the default is uniform
    """

    def __init__(self, weights=None):
        ObjectiveTerm.__init__(self, _usesPointDerivatives=1)
        self.weights = weights

    def __call__(self, ptvel, **kwargs):
        v = 0
        for i, ve in enumerate(ptvel):
            if not self.weights:
                v += N.dot(ve, ve)
            else:
                v += self.weights[i] * N.dot(ve, ve)
        return v


class AncaLegible(ObjectiveTerm):
    def __init__(self, goal, avoid, start, nframes, skipFrames):
        ObjectiveTerm.__init__(self, _usesState=1)
        self.goal = N.array(goal)
        self.start = N.array(start)
        self.avoid = avoid
        self.currentTraj = []
        for i in range(nframes):
            self.currentTraj.append(self.start)

        vec_to_goal = self.goal - N.array(self.start)
        self.optimalCost = N.sqrt(N.dot(vec_to_goal, vec_to_goal))
        self.nframes = nframes
        self.skipFrames = skipFrames
        self.denom = 0.5 * nframes

    def __call__(self, t, state, **kwargs):
        legibility = 0.0
        if t not in self.skipFrames:
            self.currentTraj[t] = state
            weight = 1 - (t / self.nframes)

            # prob to goal
            goalProb = self.ProbGgivenTraj(t, self.goal)

            # compute Bayesian regularization factor
            regularizer_Z = goalProb
            for i in range(len(self.avoid)):
                regularizer_Z = regularizer_Z + self.ProbGgivenTraj(t, N.array(self.avoid[i]))
            regularizer_Z = 1 / regularizer_Z

            # compute legibility score from
            # https://www.ri.cmu.edu/pub_files/2013/3/legiilitypredictabilityIEEE.pdf
            Prob = regularizer_Z * goalProb
            legibility = -(weight * Prob)
        return legibility

    def ProbGgivenTraj(self, currentFrame, G):
        size = len(self.currentTraj[currentFrame])
        # optimal cost to goal at current position
        Dist_optimal_to_goal = G[:size] - self.currentTraj[currentFrame]
        mag = N.dot(Dist_optimal_to_goal, Dist_optimal_to_goal)  # hack to fix ad problem with 0 length vectors
        Cost_Q_G = 0.0
        if mag != 0.0:
            Cost_Q_G = N.sqrt(mag)

        # current trajectory cost to current point
        Cost_S_Q = 0.0
        for i in range(currentFrame):
            if currentFrame > 0:
                Dist_vec = self.currentTraj[i + 1] - self.currentTraj[i]
                mag = N.dot(Dist_vec, Dist_vec)  # hack to fix ad problem with 0 length vectors
                if mag != 0.0:
                    Cost_S_Q = Cost_S_Q + N.sqrt(mag)

        # calc prob
        num = N.power(math.e, -(Cost_Q_G + Cost_S_Q))
        den = N.power(math.e, -self.optimalCost)
        P = num / den
        return P


# Formulation for HRI'16 Paper
class ProbFieldLegible(ObjectiveTerm):
    """
    minimize velocity defined by a Gaussian atractor
    """

    def __init__(self, _pt, _tar, _sigmaX, _sigmaY, _usesPoints=1):
        ObjectiveTerm.__init__(self)
        self.pt = _pt
        self.tar = _tar
        self.sigmaX = _sigmaX
        self.sigmaY = _sigmaY

    def __call__(self, t, points, **kwargs):
        x = points[self.pt][0]
        y = points[self.pt][1]
        z = points[self.pt][2]

        vX = self.evalDerivGauss(x, self.tar[0], self.sigmaX, x, y, z)
        vY = self.evalDerivGauss(y, self.tar[1], self.sigmaY, x, y, z)
        vel = N.array([vX, vY])
        speed = N.dot(vel, vel)

        return speed

    def evalDerivGauss(self, _k, _q, _r, _x, _y, _z):
        val = ((_k - _q) / N.power(_r, 2)) * self.evalGauss(_x, _y, _z)
        return val

    def evalGauss(self, _x, _y, _z):
        val = math.exp(-((N.power((_x - self.tar[0]), 2) / (2 * N.power(self.sigmaX, 2))) + (
            N.power((_y - self.tar[1]), 2) / (2 * N.power(self.sigmaY, 2)))))
        return val

# BEGIN RSS'16 LEGIBLE FORMULATION

# RSS'16 End-Effector to Goal vector, S
class LegibleS(ObjectiveTerm):
    """
    minimize vector from end effector to target
    """

    def __init__(self, _pt, _tar, nFrames):
        ObjectiveTerm.__init__(self, _usesPoints=1)
        self.pt = _pt
        self.nFrames = nFrames
        self.tar = N.array([_tar[0], _tar[1], _tar[2]])

    def __call__(self, t, points, frames, ptvel, **kwargs):
        s_vec = self.tar - points[self.pt]
        val = N.dot(s_vec, s_vec)
        return val


# RSS'16 Current Prediction to Goal vector, G
class LegibleG(ObjectiveTerm):
    """
    minimize vector from prediction to target
    """

    def __init__(self, _pt, _tar, nFrames, _heuristicFunction):
        ObjectiveTerm.__init__(self, _usesPoints=1, _usesPointDerivatives=1)
        self.pt = _pt
        self.nFrames = nFrames
        self.tar = N.array([_tar[0], _tar[1], _tar[2]])
        self.closestPoint = _heuristicFunction
        self.predictedPoints = []  # hack to record predicted points
        for i in range(nFrames):
            self.predictedPoints.append(_tar)

    def __call__(self, t, points, frames, ptvel, **kwargs):
        P = self.closestPoint(points[self.pt], ptvel[self.pt], frames[self.pt],
                              (1 - (t / self.nFrames)))  # predicted point
        g_vec = self.tar - P  # vector from prediction to target
        val = N.dot(g_vec, g_vec)
        self.predictedPoints[t] = P  # record predicted point
        return val


# END RSS'16 LEGIBLE FORMULATION

class PointDistance(ObjectiveTerm):
    """
    makes sure a point is at least some distance from a fixed location
    """

    def __init__(self, _pointID, _r, _x, _y, _z, _noZ=False):
        ObjectiveTerm.__init__(self, _usesPoints=1, _noZ=_noZ)
        self.pointID = _pointID
        self.r = _r
        self.x = _x
        self.y = _y
        self.z = _z

    def __call__(self, points, **kwargs):
        dx = self.x - points[self.pointID][0]
        dy = self.y - points[self.pointID][1]
        dz = 0 if self.noZ else self.z - points[self.pointID][2]
        dst = AD.MATH.sqrt(dx * dx + dy * dy + dz * dz)

        return self.r - dst if self.r > dst else 0


class AlignAxis(ObjectiveTerm):
    def __init__(self, _pointID, _ax, _vec):
        ObjectiveTerm.__init__(self)
        self.pointID = _pointID
        self.ax = _ax
        self.vec = _vec

    def __call__(self, frames, **kwargs):
        return 1 - N.dot(frames[self.pointID][:, self.ax], self.vec)
