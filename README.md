# Simulation interface for robotic manipulators

Based on [Beautiful Bullet](https://github.com/nash169/beautiful-bullet).

This repository contains the files used for the simulation of [this](https://github.com/Bbosc/GDSOA/tree/master) project.

It's interfacing a zmq Requester with the Beautiful Bullet simulator. 
The core file **./src/sim_join_acc.cpp** takes joint accelerations as input via a ZeroMQ Stream, and passes them 
to a QP Controller outputing optimized torques.
