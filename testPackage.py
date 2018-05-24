from trajopt.solver.slsqp import SLSQPSolver
from trajopt.spacetime.spacetime import Spacetime
from trajopt.robot.builtinRobots import Mico
from trajopt.spacetime.builtinObjectives import StateVelocity, AlignAxis
from trajopt.spacetime.builtinConstraints import Nail, alignAxisGT
from trajopt.utilities.fileOperations import writeTrajectoryFile

# Create Robot
ready2 = [-2.45697320422539, 2.957355675785974, 1.327502162859815, -1.243116391883747, -1.0383748231146699,
          1.7725862364966851] # default state
robot = Mico()
robot.default = ready2

# Create Spacetime Problem
numFrames = 15
targetPosition = (-0.300000, -0.500000, 0.060000)
targetState = [-0.81547991, 4.84380595, 2.03528418, 1.25361843, -1.58674811, 0.23720043]

st = Spacetime(robot, numFrames)
st.defaultState[numFrames - 1] = targetState
st.excludeKeys = [0]
st.excludeKeys.append(numFrames - 1)

# Setup Objective (minimum joint velocity)
minJointVelocityTerm = StateVelocity()
minJointVelocityWeight = 100.0
st.addPointObjective(minJointVelocityTerm, minJointVelocityWeight) # add to objectives

# Setup Objective (align end-effector axis upwards)
robotEndEffectorUpVector = (0, 0, 1)
robotEndEffectorUpAxis = 1
robotEndEffectorPoint = -1 # last point on robot

alignAxisTerm = AlignAxis(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector)
minJointVelocityWeight = 25.0
st.addPointObjective(alignAxisTerm, minJointVelocityWeight) # add to objectives

# Setup Constraints

# Nail end position at target location
st.addConstraint(numFrames - 1, Nail(robotEndEffectorPoint, targetPosition, False))
st.addConstraint(numFrames - 1, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))

# Create Solver and solve the Spacetime problem
solver = SLSQPSolver(st,_callback=True)
solution = solver()

# Write solution to file
writeTrajectoryFile(st, solution, "C:\\Users\\cbodden\\Desktop\\test.txt")
