# trajectoryoptimizer-public
Repository for the Python trajectory optimizer for legible arm motion.

*Note: This repo only contains the code for generating trajectories. It does not contain the Unity simulation playback framework.*

## DISCLAIMER
This code base is no longer actively maintained.

This software is provided 'as is' without warranty of any kind, either express or implied, including, but not limited to, the implied warranties of fitness for a purpose, or the warranty of non-infringement. Without limiting the foregoing, we make no warranty that:

* the software will meet your requirements
* the software will be uninterrupted, timely, secure or error-free
* the results that may be obtained from the use of the software will be effective, accurate or reliable
* the quality of the software will meet your expectations
* any errors in the software obtained will be corrected.

## Basic Usage
For a complete usage guide please see the STrOBE section in the included white paper: [docs/TR1833.pdf](docs/TR1833.pdf).

For simple example code please see [opt_example.py](opt_example.py).

This framework has only been run using the [Anaconda Python](https://www.anaconda.com) distribution, version 2.7.

The basic steps for generating a trajectory are:

1. Create a robot object
2. Create a Spacetime problem
3. Create optimization objectives and add objectives to the Spacetime problem
4. Create constraints and add constraints to the Spacetime problem
5. Create a solver with the Spacetime problem
6. Solve!

### Available Robots
Currently the trajectory optimizer contains 5 built-in robots:

* Particle2DRobot: A point robot that moves in 2 dimensions.
* TwoLink: A simple 2D robot with two links.
* UR5: A [Univeral Robots UR5](https://www.universal-robots.com) (six joints).
* Mico: A [Kinova Mico](https://www.kinovarobotics.com) (six joints).
* Reactor: A [Trossen Robotics PhantomX Reactor](http://www.trossenrobotics.com) (five joints).

More robots can be added by creating a list of the rotations & displacements between successive links starting from the base. See the implementation of the built-in robots in [trajopt/robot/builtinRobots.py](trajopt/robot/builtinRobots.py) for examples.

### Available Objectives
Currently the trajectory optimizer contains 12 built-in optimization objectives. See the implementation of the built-in objectives in [trajopt/spacetime/builtinObjectives.py](trajopt/spacetime/builtinObjectives.py) for more details.

More objectives can be added by implementing the interface defined by [trajopt/spacetime/objective.py](trajopt/spacetime/objective.py). 

### Available Constraints
Currently the trajectory optimizer contains 9 built-in constraints. See the implementation of the built-in constraints in [trajopt/spacetime/builtinConstraints.py](trajopt/spacetime/builtinConstraints.py) for more details.

More constraints can be added by implementing the interface defined by [trajopt/spacetime/constraint.py](trajopt/spacetime/constraint.py).

### Available Solvers
Currently only a Sequential Least SQuares Programming (SLSQP) solver is implemented. More solvers can be added by implementing the solver interface [trajopt/solver/solverInterface.py](trajopt/solver/solverInterface.py). See the SLSQP implementation for more details [trajopt/solver/slsqp.py](trajopt/solver/slsqp.py).
