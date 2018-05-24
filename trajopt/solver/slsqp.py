__author__ = ['gleicher', 'cbodden']

import numpy as N
import math
from trajopt.utilities.timer import Timer
from trajopt.solver.solverInterface import Solver
import scipy.optimize.slsqp


class SLSQPSolver(Solver):
    def __init__(self, _stProblem,
                 _iter=200, _acc=1.0E-6,
                 _iprint=1, _x0=None,_disp=None, _full_output=0,
                 _verbose=True,
                 _callback=False,
                 _doBounds=True):
        Solver.__init__(self,_stProblem)
        self.func = lambda (x): self.spacetimeProblem.eval(x)
        self.fprime = lambda (x): self.spacetimeProblem.evalG(x)
        self.x0 = self.spacetimeProblem.defaultState.extract(excludeKeys=self.spacetimeProblem.excludeKeys) if (_x0 == None) else _x0
        self.lower, self.upper = self.spacetimeProblem.getVarBounds() if _doBounds else (None, None)
        self.iter = _iter
        self.acc = _acc
        self.iprint = _iprint
        self.disp = _disp
        self.full_output = _full_output
        self.verbose = _verbose
        self.callback = self.spacetimeProblem.robot.cleanupCallback if _callback else None
        self.slsqp_last_trace = []
        self.slsqp_last_status = {}
        self.epsilon = math.sqrt(N.finfo(float).eps)
        self.slsqp_exit_modes = {-1: "Gradient evaluation required (g & a)",
                                 0: "Optimization terminated successfully.",
                                 1: "Function evaluation required (f & c)",
                                 2: "More equality constraints than independent variables",
                                 3: "More than 3*n iterations in LSQ subproblem",
                                 4: "Inequality constraints incompatible",
                                 5: "Singular matrix E in LSQ subproblem",
                                 6: "Singular matrix C in LSQ subproblem",
                                 7: "Rank-deficient equality constraint subproblem HFTI",
                                 8: "Positive directional derivative for linesearch",
                                 9: "Iteration limit exceeded"}

    def __call__(self):
        self.slsqp_last_trace = []
        self.slsqp_last_status = {}
        Ttotal = Timer("total")
        Tslsqp = Timer("_slsqp")
        Teval = Timer("eval")
        TevalG = Timer("evalG")

        Ttotal.start()

        # do an evaluation to count the number of constraints
        # since we need the gradients on the first evaluation, we'll do that
        # remember - for us, fprime returns the values as well
        # we actually only need the sizes to make the workspaces
        # note: its unclear whether the values are ever used: the first iteration
        # might recompute them. but we need to build the datastructures
        TevalG.start()
        fret, eret, iret, fgret, egret, igret = self.fprime(self.x0)
        TevalG.stop()

        # n = number of variables
        n = len(self.x0)

        # m = total number of constraints
        # meq = number of equality constraints
        meq = len(eret)
        miq = len(iret)
        m = meq + miq

        # Define the workspaces for SLSQP
        n1 = n + 1
        mineq = m - meq + n1 + n1  # note: this is NOT MIQ!
        len_w = (3 * n1 + m) * (n1 + 1) + (n1 - meq + 1) * (mineq + 2) + 2 * mineq + (n1 + mineq) * (n1 - meq) \
                + 2 * meq + n1 + ((n + 1) * n) // 2 + 2 * m + 3 * n + 3 * n1 + 1
        len_jw = mineq
        w = N.zeros(len_w)
        jw = N.zeros(len_jw)

        # setup bounds - whether we need them or not
        xl = self.makeBoundsArray(n, self.lower, -1.0E12)
        xu = self.makeBoundsArray(n, self.upper, 1.0E12)

        # Initialize the iteration counter and the mode value
        mode = N.array(0, int)
        acc = N.array(self.acc, float)
        majiter = N.array(self.iter, int)
        majiter_prev = 0

        # build the fortran versions from the returned versions
        fx = fret
        c = self.assembleConsts(eret, iret)
        a = self.assembleCjacs(egret, igret)
        g = N.append(fgret, 0.0)  # see note in scipy - this is required

        # make a zero size null jacobian, just in case
        # note that its zero rows all have the right number of columns
        a0 = N.zeros((1, n + 1))

        # get the starting point as our vector
        x = N.array(self.x0,dtype=float)

        # stats
        neval = 0
        ngeval = 0
        nloop = 0

        self.slsqp_last_trace.append((int(majiter), nloop, fx, max(c) if len(c) else 0, N.dot(c, c)) if len(c) else 0)

        # life with fortran - we need an infinite loop with a break...
        while 1:
            # note that we are already all set up and ready to go...
            # print "    %3d %3d %6.2f %6.2f %6.2f %d:%6.2f %6.2f"%(       mode, majiter, fx, min(x), max(x), len(g), N.dot(g,g), N.sum(a))
            Tslsqp.start()
            if a is None or len(a) == 0:
                a = a0
            scipy.optimize._slsqp.slsqp(m, meq, x, xl, xu, fx, c, g, a, acc, majiter, mode, w, jw)
            Tslsqp.stop()
            # print "%3d %3d %3d %6.2f %6.2f %6.2f %d:%6.2f %6.2f"%(nloop, mode, majiter, fx, min(x), max(x), len(g), N.dot(g,g), N.sum(a))

            if self.callback is not None and majiter > majiter_prev:
                self.callback(x)

            if majiter > majiter_prev:
                # we don't actually remember the values for the iteration,
                # so we need to do an extra eval per iteration (ick!)
                # so that we can keep track of progress
                lf, le, li = self.func(x)
                try:
                    cmax = max(le)
                    cmag = N.dot(le, le)
                except:
                    cmax = 0
                    cmag = 0
                try:
                    cimax = max(li)
                    cmax = max(cmax, cimax)
                    cmag += N.dot(li, li)
                except:
                    pass
                self.slsqp_last_trace.append((int(majiter), nloop, lf, cmax, cmag))

            if abs(mode) != 1:
                break

            # do we need an eval?
            if mode == 0 or mode == 1:
                Teval.start()
                fret, eret, iret = self.func(x)
                Teval.stop()
                c = self.assembleConsts(eret, iret)
                fx = fret
                neval += 1

            # do we need a gradient eval (note: we're wasting the regular eval)
            if mode == 0 or mode == -1:
                TevalG.start()
                fret, eret, iret, fgret, egret, igret = self.fprime(x)
                TevalG.stop()
                a = self.assembleCjacs(egret, igret)
                g = N.append(fgret, 0.0)  # see note in scipy - this is required
                ngeval += 1

            nloop += 1
            majiter_prev = int(majiter)

        Ttotal.stop()

        if self.verbose:
            print "Done! - stop criteria(%d = %s)" % (int(mode), self.slsqp_exit_modes[int(mode)])
            print "loops(%d) evals(%d) grads(%d)" % (nloop, neval, ngeval)
            print "minimum", fx
            print Ttotal
            print Tslsqp
            print Teval
            print TevalG

        self.slsqp_last_status["Ttotal"] = Ttotal
        self.slsqp_last_status["Tslsqp"] = Tslsqp
        self.slsqp_last_status["Teval"] = Teval
        self.slsqp_last_status["TevalG"] = TevalG
        self.slsqp_last_status["mode"] = int(mode)
        self.slsqp_last_status["mode-name"] = self.slsqp_exit_modes[int(mode)]
        self.slsqp_last_status["minimum"] = fx
        self.slsqp_last_status["loops"] = nloop
        self.slsqp_last_status["evals"] = neval
        self.slsqp_last_status["ngeval"] = ngeval

        return x

    def makeBoundsArray(self, n, val, default):
        if val is None or val is False:
            return N.array([default] * n)
        else:
            try:  # see if its an array - otherwise we assume its a number
                if len(val) == n:
                    return N.array(val)
                else:
                    raise TypeError("Boundary array is the wrong length")
            except:  # assume that its a number
                return N.full(val, n)

    def assembleConsts(self, eqs, ineqs):
        return N.concatenate((eqs if eqs != None else [], ineqs if ineqs != None else []))

    def assembleCjacs(self, eqs, ineqs):
        if ineqs is None or len(ineqs) == 0:
            a = eqs
        elif eqs is None or len(eqs) == 0:
            a = ineqs
        else:
            a = N.vstack((eqs, ineqs))
        # we need to add an extra column (?) for the fortran workspace
        if len(a) == 0 or a is None:
            a = None
        else:
            a = N.concatenate((a, N.zeros([len(eqs) + len(ineqs), 1])), 1)
        return a
