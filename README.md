# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

This project works with Udacity self driving simulator (for term 2) and
uses a Model Predictive Controller in order to steer a vehicle around a track.

## MPC
The MPC is a controller that set its outputs based on a physical model that is integrated inside the controller.
This controller uses finite future horizon in order to take into acocunt the vehicle future state when changing its outputs.

The MPC is solving an non linear optimization problem, in our code this part is done by using IpOpt which does most of the heavy
lifting.

The structure of this controller makes it more rubust then PID in a way that it can handle latency in the system when responding to
the acutators changes.


