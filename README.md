# PassivityControlOfHapticRobot
 This project is the implementation of Adaptive Energy Reference Time Domain Passivity Control on the PHANToM Omni haptic device.
 
The haptic device is interacting with a 2D virtual environment. The human operator creates movements in x and y directions.

The interaction with the virtual environment can be unstable due to sampling time, delay and ... in haptic devices.
In the CommandJointTorque.cpp, an energy reference is designed to create a stable behaviour.
The parameter of the energy reference is selected by solving an optimization problem.
A virtual damper is utilized to change the force feedback in order to follow the desired stable energy reference.
