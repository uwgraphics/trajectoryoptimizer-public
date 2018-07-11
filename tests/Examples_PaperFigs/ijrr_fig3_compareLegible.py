# This generates Figure 3 (top) of the IJRR paper
# the 2D point robot moving to an array of goals, where the goal
# set is along a line segment
# motions are generated using "our" legibility approach
# as well as with the one of Dragan and Srinivasa.
# This does not include the drawing code (that produce the diagrams from these numbers)
#   very simple drawing using matplotlib has been added
#
# the paper used an older version of the code, but this should be equivalent

# normally I would import from - but for clarity, I keep
# the whole module names
from trajopt.solver.slsqp import SLSQPSolver
import trajopt.spacetime.spacetime
import trajopt.robot.builtinRobots
import trajopt.spacetime.builtinObjectives
import trajopt.spacetime.builtinConstraints
import numpy
import copy

# to draw a picture
from matplotlib import pyplot as plt

# description of the problem
# target endpoints
points = [(-4,5,0),(-1.5,5,0),(0,5,0),(2.5,5,0),(5,5,0)]
# beginning of whole thing
startpos=(0,0,0)

# parameters of the solution
nsteps=15
pointID = -1
noZ = True
ptvel = 75
ptvel2 = 1
legw = 7.5

# put the results into an arrays
outputs = []

# generate a path for each point using our approach
for i in range(len(points)):
    # basic point robot problem
    robot = trajopt.robot.builtinRobots.Particle2DRobot(1)
    st = trajopt.spacetime.spacetime.Spacetime(robot, nsteps)

    # constraints
    c1 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints - 1, (0, 0, 0), robot.noZ)
    st.addConstraint(0, c1)
    c2 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints - 1, points[i], robot.noZ)
    st.addConstraint(nsteps - 1, c2)

    # objectives
    st.addPointObjective(trajopt.spacetime.builtinObjectives.PointVelocity(pointID), ptvel)


    # st.addPointObjective(SO.StateVelocity(), ptvel)
    def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
        xCoord = _eePoint[0]
        if (xCoord > 5):
            xCoord = 5
        elif (xCoord < -4):
            xCoord = -4

        P = numpy.array([xCoord, 5, 0])
        return P


    st.addPointObjective(trajopt.spacetime.builtinObjectives.LegibleG(pointID, points[i], nsteps, closePointToLine), legw)  # g vector term
    # note: this is changed, since the demo code used the old version
    # it should be the same
    st.addPointObjective(trajopt.spacetime.builtinObjectives.LegibleS(pointID, points[i], nsteps), legw / 10)  # s vector term

    # solve
    # r = st.myslsqp()
    solver = SLSQPSolver(st, _callback=True)
    solution = solver()

    outputs.append( ("Our_Legible_%i"%i,solution) )


# generate a path for each point using the S&A approach
#solve predictables
robot = trajopt.robot.builtinRobots.Particle2DRobot(1) # basic point robot problem

for i in range(len(points)):
    # objectives
    # for S&A we need a list of non-goals to avoid, as well as goals to seek
    nongoals = copy.deepcopy(points[:i] + points[(i+1):])
    goal = copy.deepcopy(points[i])

    # create
    st = trajopt.spacetime.spacetime.Spacetime(robot, nsteps)

    # constraints
    c1 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints-1, (0,0,0), robot.noZ)
    st.addConstraint(0,c1)
    c2 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints-1, points[i], robot.noZ)
    st.addConstraint(nsteps-1,c2)

    #objectives
    #                                                                    goal, avoid,    start,    nframes, skipFrames
    st.addPointObjective(trajopt.spacetime.builtinObjectives.AncaLegible(goal, nongoals, startpos, nsteps,  []), 2)
    st.addPointObjective(trajopt.spacetime.builtinObjectives.StateVelocity(), ptvel2) # regularization

    # solve
    # r = solve(st)
    solver = SLSQPSolver(st, _callback=True)
    solution = solver()

    outputs.append( ("SA_Legible_%i"%i,solution) )

# draw a picture
def splitxy(vector):
    return vector[0::2],vector[1::2]

def picture(listOfResults):
    plt.clf()
    for p in listOfResults:
        x,y = splitxy(p[1])
        plt.plot(x,y)
        plt.scatter(x,y)

# call "picture(outputs)" to see the results