__author__ = ['gleicher', 'cbodden']
"""
an attempt to define spacetime problems

at one level, all a spacetime problem is is a function that given a vector
(the KeyVariables - see states.py) returns the value of the objective function,
and the vector of constraint values (well, two - one for eqs, one for ineqs)

to do this, it needs to keep a lot of stuff
"""
from itertools import chain
import numpy as N
import trajopt.utilities.adInterface as AD
import trajopt.spacetime.states as ST
from trajopt.utilities.timer import Timer


def returnListOrNone(lst):
    try:
        if len(lst) > 0:
            return lst
    except:
        pass
    print "Zero Length Constraint List May Be a Problem!"
    return []


def pointVels(a, b, _noZ):
    if _noZ:
        return [(a[i][0] - b[i][0], a[i][1] - b[i][1]) for i in range(len(a))]
    else:
        return [(a[i][0] - b[i][0], a[i][1] - b[i][1], a[i][2] - b[i][2]) for i in range(len(a))]


def pointAccs(a, b, c, _noZ):
    if _noZ:
        return [(a[i][0] - b[i][0] * 2 + c[i][0], a[i][1] - b[i][1] * 2 + c[i][1]) for i in range(len(a))]
    else:
        return [(a[i][0] - b[i][0] * 2 + c[i][0], a[i][1] - b[i][1] * 2 + c[i][1], a[i][2] - b[i][2] * 2 + c[i][2]) for
                i in range(len(a))]

class Spacetime:
    def __init__(self, robot, nstates):
        self.excludeKeys = []

        # things that define the spacetime problem
        self.robot = robot
        self.interpolationScheme = None

        # we need a state vector for injecting the keyvariables into
        self.defaultState = ST.KeyVector(nstates, self.robot.nvars)
        for i in range(len(self.defaultState)):
            self.defaultState[i] = robot.default

        # the constraints and objectives that make up the problem
        # note: these should be added by the "add" functions - so that
        # appropriate checking happens - that's why the names have the underscores
        # constraints
        self._pointConstraints = []  # tuples (time,constraint)
        self._allTimesConstraints = []  # point constraints applied to all times
        # objective functions - needs to be a tuple (weight, PointObjTerm)
        self._pointObjectives = []  # applied to ALL times (where the derivatives exist)

        # we need to know whether or not we have different kinds of constraints
        # before we ever gather them up (for example, in setting up the solver)
        self.hasIneq = False
        self.hasEq = False
        # it's also useful to know if we need derivatives in evaluation (so we can save
        # effort)
        self.maxStateDeriv = 0
        self.maxPointDeriv = 0

        # keep track of these kinds of things
        self.evalTimer = Timer("eval")
        self.nobjGTimer = Timer("nObjG")
        self.evalGTimer = Timer("evalG")

    def __len__(self):
        return len(self.defaultState)

    def makeBlankState(self):
        """
        this makes something like the default state, but since the default state is special,
        the code is redundant
        :return:  a state vector with the correct initial configuration
        """
        newstate = ST.KeyVector(len(self), self.robot.nvars)
        for i in range(len(self)):
            newstate[i] = self.robot.default
        return newstate

    def addConstraint(self, t, cons):
        self._pointConstraints.append((t, cons))
        if cons.eqs:   self.hasEq = True
        if cons.ineqs: self.hasIneq = True
        self.maxStateDeriv = max(self.maxStateDeriv, cons.usesStateDerivatives)
        self.maxPointDeriv = max(self.maxPointDeriv, cons.usesPointDerivatives)

    def addAllTimeConstraint(self, cons):
        self._allTimesConstraints.append(cons)
        if cons.eqs:   self.hasEq = True
        if cons.ineqs: self.hasIneq = True
        self.maxStateDeriv = max(self.maxStateDeriv, cons.usesStateDerivatives)
        self.maxPointDeriv = max(self.maxPointDeriv, cons.usesPointDerivatives)

    def addPointObjective(self, tupleOrObjective, weight=1.0):
        try:
            ptObjective = tupleOrObjective[0]
            weight = tupleOrObjective[1]
        except:
            ptObjective = tupleOrObjective
        self._pointObjectives.append((ptObjective, weight))
        self.maxStateDeriv = max(self.maxStateDeriv, ptObjective.usesStateDerivatives)
        self.maxPointDeriv = max(self.maxPointDeriv, ptObjective.usesPointDerivatives)

    def changeWeight(self, objective, newWeight):
        changed = None
        # since we cannot change the tuple, we have to go through the list
        # ugly, and non Pythonic
        for i in range(len(self._pointObjectives)):
            if self._pointObjectives[i][0] == objective:
                changed = True
                self._pointObjectives[i] = (objective, newWeight)
        if changed is None:
            raise KeyError("didn't find objective")

    def makeStateVector(self, keyvariables):
        """
        this makes a state vector (an array of state variables) from a key vector
        (an array of variables, with only the active variables)
        :param keyvariables: remember this takes a KEYVARIABLES (see states.py)
        :return:
        """
        # make the state vector
        keyvec = self.defaultState.inject(keyvariables, self.excludeKeys)
        # turn this into a state sequence
        states = keyvec if self.interpolationScheme == None else self.interpolationScheme(keyvec)
        return states

    def getStates(self, keyvariablesOrStateVector):
        """
        if you're passed either key variables or a state vector, make good use of it
        :param keyvariablesOrStateVector:
        :return: a state vector appropriate for this spacetime problem
        """
        # if we're passed a state vector, allow us to evaluate it
        nstates = len(self.defaultState)
        try:
            if keyvariablesOrStateVector.nkeys == nstates:
                states = keyvariablesOrStateVector
            else:
                raise IndexError("Wrong size State Vector to Spacetime Eval")
        except AttributeError:
            states = self.makeStateVector(keyvariablesOrStateVector)

        return states

    def getVarBounds(self):
        nstates = len(self) - len(self.excludeKeys)
        upper = N.empty(nstates * self.robot.nvars)
        lower = N.empty(nstates * self.robot.nvars)
        for i in range(nstates):
            upper[i * self.robot.nvars:(i + 1) * self.robot.nvars] = self.robot.xUBounds
            lower[i * self.robot.nvars:(i + 1) * self.robot.nvars] = self.robot.xLBounds

        return (lower, upper)

    def eval(self, keyvariablesOrStateVector):
        """
        evaluate the spacetime problem from a given state vector
        :param keyvariablesOrStateVector:
        :return: three values a scalar (objective) and a lists of the eqs and ineqs
        """
        self.evalTimer.start()
        # keep this around and handy
        nstates = len(self.defaultState)

        states = self.getStates(keyvariablesOrStateVector)

        # compute the velocity and acceleration vectors
        # just in case we need them
        # note: the ends might not be useful - but we compute something anyway
        stvels = None if self.maxStateDeriv < 1 else [
            (states[i] - states[i - 1] if i > 0 else states[i + 1] - states[i]) for i in range(nstates)]
        stacc = None if self.maxStateDeriv < 2 else \
            [states[1] * 2 - states[0] - states[2]] + \
            [(states[i] * 2 - states[i - 1] - states[i + 1]) for i in range(1, nstates - 1)] + \
            [states[nstates - 2] * 2 - states[nstates - 3] - states[nstates - 1]]

        # compute the point position for each point for each time frame
        # might be a little wasteful, but can serve as a caching strategy if they are used
        # note: the ends may be bogus, so don't use them if you really care
        # points = [self.robot(state) for state in states]
        points = []
        frames = []
        for state in states:
            p, f = self.robot.getFrames(state)
            points.append(p)
            frames.append(f)

        ptvels = None if self.maxPointDeriv < 1 else \
            [pointVels(points[1], points[0], self.robot.noZ)] + \
            [pointVels(points[i], points[i - 1], self.robot.noZ) for i in range(1, nstates)]
        ptacc = None if self.maxPointDeriv < 2 else \
            [pointAccs(points[0], points[1], points[2], self.robot.noZ)] + \
            [pointAccs(points[i - 1], points[i], points[i + 1], self.robot.noZ) for i in range(1, nstates - 1)] + \
            [pointAccs(points[nstates - 3], points[nstates - 2], points[nstates - 1], self.robot.noZ)]

        #######################
        # now gather up all of the constraints - point constraints are at specific
        # times
        eqs = []
        ineqs = []

        # CB - pass constraints values similar to objectives
        conTerms = {"states": states, "points": points, "t": 0}

        # first let the robot add the constraints it wants to
        for t in range(nstates):
            if t not in self.excludeKeys:
                conTerms["t"] = t
                conTerms["state"] = states[t]
                conTerms["points"] = points[t]
                conTerms["frames"] = frames[t]
                if stvels: conTerms["stvel"] = stvels[t]
                if stacc: conTerms["stacc"] = stacc[t]
                if ptvels: conTerms["ptvel"] = ptvels[t]
                if ptacc: conTerms["ptacc"] = ptacc[t]
                e, i = self.robot.constraint(**conTerms)
                eqs.append(e)
                ineqs.append(i)

        # now add the point constraints
        # check to avoid things on excluded keys
        for t, c in self._pointConstraints:
            if t not in self.excludeKeys:
                conTerms["t"] = t
                conTerms["state"] = states[t]
                conTerms["points"] = points[t]
                conTerms["frames"] = frames[t]
                if stvels: conTerms["stvel"] = stvels[t]
                if stacc: conTerms["stacc"] = stacc[t]
                if ptvels: conTerms["ptvel"] = ptvels[t]
                if ptacc: conTerms["ptacc"] = ptacc[t]
                e, i = c.constraint(**conTerms)
                eqs.append(e)
                ineqs.append(i)

        # now add the all times constraints
        # note that we skip times with excluded keys
        for t in range(nstates):
            if t not in self.excludeKeys:
                for c in self._allTimesConstraints:
                    conTerms["t"] = t
                    conTerms["state"] = states[t]
                    conTerms["points"] = points[t]
                    conTerms["frames"] = frames[t]
                    if stvels: conTerms["stvel"] = stvels[t]
                    if stacc: conTerms["stacc"] = stacc[t]
                    if ptvels: conTerms["ptvel"] = ptvels[t]
                    if ptacc: conTerms["ptacc"] = ptacc[t]
                    e, i = c.constraint(**conTerms)
                    eqs.append(e)
                    ineqs.append(i)

        #######################
        # now make the objective function
        # warning - rather than += (have obj be a number) collect all the terms
        # as a list and use sum - this way automatic differentiation can look at
        # all terms together
        # this did not actually seem to make a difference in terms of performance
        # so maybe it could be switched back
        # the issue is that sum just seems to use radd - so we could provide a better
        # implementation of sum someday
        # because objTerms is build at each time step, we want to loop over time steps
        objlist = []
        objTerms = {"states": states, "points": points, "t": 0}
        for t in range(nstates):
            objTerms["t"] = t
            objTerms["state"] = states[t]
            objTerms["points"] = points[t]
            objTerms["frames"] = frames[t]
            if stvels: objTerms["stvel"] = stvels[t]
            if stacc: objTerms["stacc"] = stacc[t]
            if ptvels: objTerms["ptvel"] = ptvels[t]
            if ptacc: objTerms["ptacc"] = ptacc[t]
            for po in self._pointObjectives:
                # we assume that its a tuple (obj, weight)
                try:
                    p = po[0]
                    w = po[1]
                except:
                    p = po
                    w = 1
                # in the event that the end derivatives aren't useful, avoid using them
                # for first derivatives, the 0 time is suspect, for 2nd derivatives, the end is as well
                dmax = max(p.usesPointDerivatives, p.usesStateDerivatives)
                if dmax < 1 or t > 0:  # if we use derivatives skip the first
                    if dmax < 2 or t < nstates - 1:  # if we do second derivatives, skip the last
                        objlist.append(p(**objTerms) * w)
        # ad does this in a naive way
        obj = AD.fsum(objlist)

        self.lastKeyVariables = keyvariablesOrStateVector
        self.lastStates = states
        self.lastPoints = points
        self.lastFrames = frames

        self.evalTimer.end()

        return obj, list(chain.from_iterable(eqs)), list(chain.from_iterable(ineqs))

    # evaluation with derivatives
    def evalG(self, x):
        self.evalGTimer.start()
        v = self.makeAdVars(x)
        fv, ev, iv = self.eval(v)

        f = fv.x if isinstance(fv, AD.ADF) else fv
        fg = fv.gradient(v) if isinstance(fv, AD.ADF) else N.zeros((len(x)))

        if ev != None and len(ev):
            e = [(c.x if isinstance(c, AD.ADF) else c) for c in ev]
            el = [(c.gradient(v) if isinstance(c, AD.ADF) else N.zeros((len(x)))) for c in ev]
            eg = N.vstack(el)
        else:
            e = []
            eg = []

        if iv != None and len(iv):
            i = [(c.x if isinstance(c, AD.ADF) else c) for c in iv]
            ig = N.vstack([(c.gradient(v) if isinstance(c, AD.ADF) else N.zeros((len(x)))) for c in iv])
        else:
            i = []
            ig = []

        self.evalGTimer.stop()
        return f, e, i, fg, eg, ig

    def makeAdVars(self, vector):
        """
        this makes a keyvariables vector - but makes each of the variables an adnumber
        so we can take derivates of it. each is assigned a meaningful name.
        :param vector: the initial state of the variables
        :return: a vector of adnumbers
        """
        stateIds = [i for i in range(self.defaultState.nkeys) if i not in self.excludeKeys]
        adv = []
        c = 0
        for t in stateIds:
            for (i, vn) in enumerate(self.robot.varnames):
                adv.append(AD.adnumber(vector[c], "%d:%s" % (t, vn)))
                c += 1
        return N.array(adv)
