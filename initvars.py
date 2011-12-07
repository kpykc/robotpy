# -*- coding: utf-8 -*-
"""
Created on Thu Dec 24 19:25:13 2009

@author: kp
"""

def initvars():
    print "!> Init variables"
    OUTBOUNDS = 5.0; #if the robot is OUTBOUNDS (m) distance away from the goal, then the controller has probably exploded
    SAMPLES = 50; #max number of samples to take
    if simulate:
        SAMPLINGTIME = 0.15;
    ## Initialise controller
    startPose = array([0.0, 0.0, pi/2.0], float);
    goalPose = array([0.0, 0.0, pi/3.0], float);
    pose = startPose; # current position
    PhiPrime = array([0.0, 0.0], float); # speed of each wheel [rad/s]
    S = array([0.0, 0.0], float); # accumulated encoder values [m]
    dS = array([0.0, 0.0], float); # encoder value since last time step [m]
    rho = OUTBOUNDS; # distance from the goal

    ## INITIALIZE SOME VARIABLES.
    trajectory = multiply(ones((SAMPLES, 3), float), nan);
    data = multiply(ones((SAMPLES, 10), float), nan);

    return (PhiPrime, SAMPLINGTIME, S, pose, goalPose, startPose, trajectory, data)
