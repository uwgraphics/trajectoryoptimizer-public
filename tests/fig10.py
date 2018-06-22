# try the code from Figure 10 of the paper

from trajopt.solver.slsqp import SLSQPSolver
from trajopt.spacetime.spacetime import Spacetime
from trajopt.robot.builtinRobots import Particle2DRobot
from trajopt.spacetime.builtinObjectives import PointVelocity
from trajopt.spacetime.builtinConstraints import Nail, pointDistance

# Timesteps and Waypoints
nsteps = 51 # number of keypoints
last = nsteps - 1
middle = last/2
startPoint = (0,0,0)
midPoint = (5,0,0)
endPoint = (5,5,0)
pointID = 0 # the point we are interested in
pointVelocityWeight = 1.0

# Create Robot
robot = Particle2DRobot(1)

# Create Spacetime Problem
st = Spacetime(robot, nsteps)

# Setup Objective (minimum velocity)
st.addPointObjective(PointVelocity(pointID), pointVelocityWeight)

# Setup Constraints
st.c0 = Nail(pointID, startPoint, True) # start
st.c1 = Nail(pointID, midPoint, True) # intermediate waypoint
st.c2 = Nail(pointID, endPoint, True) # end
st.c3 = pointDistance(pointID,1,5,2.5,0,True) # obstacle
st.c4 = pointDistance(pointID,1,1.5,-0.5,0,True) # obstacle
st.c5 = pointDistance(pointID,1,1.5,2,0,True) # obstacle
st.addConstraint(0, st.c0)
st.addConstraint(middle, st.c1)
st.addConstraint(last, st.c2)
st.addAllTimeConstraint(st.c3)
st.addAllTimeConstraint(st.c4)
st.addAllTimeConstraint(st.c5)

# Help the solver a bit by configuring known states in x0
st.defaultState[last] = [endPoint[0], endPoint[1]]
st.defaultState[middle] = [midPoint[0], midPoint[1]]

# Create Solver and solve the Spacetime problem
solver = SLSQPSolver(st, _callback=True)
solution = solver()
