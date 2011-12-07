# -*- coding: utf-8 -*-
"""
Created on Fri Dec 25 12:28:19 2009

@author: kp
"""

try:
    from numpy import *
except:
    sys.exit(1)

# Globals
#
# Wheels numbers: 0 - right; 1 - left.
r=0
l=1
# Some other
Stop=0

global Steps
Steps = 0
global EndCond
EndCond = 0
global wheelRadiusValue
wheelRadiusValue = 0.027
global wheelbaseValue
wheelbaseValue = 0.142;
global Krho
Krho = 0.15;
global Kalpha
Kalpha = 0.5;
global Kbeta
Kbeta = -0.2;
global goalX
goalX = 0.5;
global goalY
goalY = -0.5;
global goalTheta
goalTheta = 0.0;
global goalErrorDist
goalErrorDist = 0.1;
global goalErrorAngle
goalErrorAngle = 25.0;

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

def setToRange(angle):
    tmpAngle=mod(angle,2.0*pi);
    if tmpAngle > pi:
        tmpAngle=tmpAngle-2.0*pi;
    return tmpAngle

def poseUpdate(dSr, dSl, x, y, theta, halfWheelbase):

    dx = ((dSl + dSr) / 2.0) * cos(theta);
    dy = ((dSl + dSr) / 2.0) * sin(theta);

    x = x + dx;
    y = y + dy;

    theta = theta + (dSr - dSl) / (2.0 * halfWheelbase)

    theta = setToRange(theta)

    pose = array([x, y, theta],float)

    return pose

def stepLaw(dS, S, goalPose, pose, startPose, robotConst, Time, PhiPrime):
    global Steps
    Steps = Steps + 1

    Sl = S[l]; # accumulated encoder values for the left wheel [m]
    Sr = S[r]; # accumulated encoder values for the right wheel [m]
    dSl = dS[l]; # change in the encoder value for the left wheel since the last time step [m]
    dSr = dS[r]; # change in the encoder value for the right wheel since the last time step [m]
    wheelRadius = robotConst[0] # [m]
    halfWheelbase = robotConst[1] # [m]
    x = pose[0] # [m]
    y = pose[1] # [m]
    theta = pose[2] # [rad]
    xg = goalPose[0]; # [m]
    yg = goalPose[1]; # [m]
    thetag = goalPose[2]; # [rad]
    dist_error = goalPose[3]; # [m]
    angle_error = goalPose[4]; # [rad]
    # We don't need the start position!!!

    OpenLoop = 1;
    ClosedLoop = 0;


    #PhiPrime, rho, alpha, beta, pose, EndCond = stepLaw0(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time)

    #
    #PhiPrime, rho, alpha, beta, pose, EndCond = stepLaw1(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time)
    #
    PhiPrime, rho, alpha, beta, pose, EndCond = stepLaw2(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time)

    return (PhiPrime, rho, alpha, beta, pose, EndCond)

def stepLaw0(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time):
    EndCond = 0;
    if Time < 5:
        PhiPrime[l] = 10.0;
        PhiPrime[r] = 0.0;
    else :
        PhiPrime[l] = 0.0;
        PhiPrime[r] = 0.0;
        EndCond = 1;

    pose=poseUpdate(dSr, dSl, x, y, theta, halfWheelbase)

    rho = 0.0;
    alpha = 0.0;
    beta = 0.0;
    return (PhiPrime, rho, alpha, beta, pose, EndCond)

def stepLaw1(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time):
    EndCond = 0;
    if Time < 3:
        PhiPrime = array([10.0, 0.0])
    elif Time < 6.5:
        PhiPrime = array([5.0, 2.5])
    elif Time < 11:
        PhiPrime = array([5.0, 5.0])
    elif Time < 14.5:
        PhiPrime = array([5.0, 2.5])
    elif Time < 19:
        PhiPrime = array([5.0, 5.0])
    elif Time < 22:
        PhiPrime = array([5.0, 2.5])
    elif Time < 25:
        PhiPrime = array([5.0, 5.0])
    else :
        PhiPrime = array([0.0, 0.0])
        EndCond = 1

    pose = poseUpdate(dSr, dSl, x, y, theta, halfWheelbase)

    rho = 0.0;
    alpha = 0.0;
    beta = 0.0;
    return (PhiPrime, rho, alpha, beta, pose, EndCond)

def stepLaw2(Sr, Sl, dSr, dSl, wheelRadius, halfWheelbase, PhiPrime, x, y, theta, xg, yg, thetag, dist_error, angle_error, Time):
    # Compute rho, alpha, beta.
    pose=poseUpdate(dSr, dSl, x, y, theta, halfWheelbase)

    # pythagoras theorem, sqrt(dx^2 + dy^2)
    rho = sqrt((xg-x)**2+(yg-y)**2);

    # angle of the vector pointing from the robot to the goal in the inertial frame
    Lambda = math.atan2(yg-y, xg-x);

    # angle of the vector pointing from the robot to the goal in the robot frame
    alpha = Lambda - theta;

    alpha = setToRange(alpha)

    BackwardOK = 1;

    if BackwardOK == 1 : # Backward speed allowed
        # If obstacle in "front" => go forward
        if(abs(alpha)<=pi/2.0):
            beta = thetag-Lambda
            Krho2 = Krho
        else :
            alpha=Lambda-theta-pi
            alpha=setToRange(alpha)
            beta= thetag-Lambda-pi
            Krho2=-Krho
    else :
        beta = thetag-Lambda
        Krho2 = Krho

    beta = setToRange(beta)

    # Compute omega and vu (control output)
    # write here...
    vu = Krho2 * rho; # [m/s]
    omega = Kalpha * alpha + Kbeta * beta; # [°/s]

    # Calculate wheel speeds
    M = array([[wheelRadius/2, wheelRadius/2], [wheelRadius/(2*halfWheelbase), -wheelRadius/(2*halfWheelbase)]], float)
    Minv = linalg.inv(M);
    PhiPrime = (dot(Minv, array([vu, omega]).conj().transpose())).conj().transpose();

    dtheta = abs(setToRange(theta-thetag));

    EndCond = ((rho < dist_error) and (dtheta < angle_error)) or (rho > 2);

    return (PhiPrime, rho, alpha, beta, pose, EndCond)

def simulate():
    print "!> Simulate"

    ## INITIALIZE VARIABLES.
    PhiPrime, SAMPLINGTIME, S, pose, goalPose, startPose, trajectory, data = initvars();

    EndCondition = 0;

    # sets the robot goal pose
    goalPose = array([goalX, goalY, goalTheta, goalErrorDist, goalErrorAngle*pi/180], float);

    robotConst=array([0, 0], float);
    robotConst[0] = wheelRadiusValue; # wheel radius
    robotConst[1] = wheelbaseValue/2; # 1/2 wheelbase

    print "Initial conditions"
    print "goalPose:   " + str(goalPose)
    print "robotConst: " + str(robotConst)
    print "startPose:  " + str(startPose)
    print "PhiPrime:   " + str(PhiPrime)

    dEncoder = array([0.0, 0.0], float);
    encoder = array([0.0, 0.0], float);
    ## CONTROL LOOP.
    n = 1;
    EndCond = 0;

    # robot trajectory
    trajectory=zeros((1,3), float).reshape(1,3)

    while ((not EndCond) and (not Stop)):
        ## ESTIMATE TRAVELLED DISTANCES.
        # interpolate encoder value for simulation
        dEncoder = PhiPrime*SAMPLINGTIME
        encoder = encoder + dEncoder

        # calculate change in displacement from previous time step
        dS = dEncoder * robotConst[0]

        # accumulate total displacement
        S = S + dS;
        Time = n*SAMPLINGTIME;

        ## CONTROL STEP.
        # run the control step
        [PhiPrime, rho, alpha, beta, pose, EndCond] = stepLaw(dS, S, goalPose, pose, startPose, robotConst, Time, PhiPrime);

        trajectory=append(trajectory, pose.reshape(1, 3), axis=0)

        ## SET ROBOT WHEEL SPEEDS.
        # simulate motor saturation
        maxspeed = 6;
        if (PhiPrime[r] > maxspeed):
            PhiPrime[r] = maxspeed;
        if (PhiPrime[r] < -maxspeed):
            PhiPrime[r] = -maxspeed;
        if (PhiPrime[l] > maxspeed):
            PhiPrime[l] = maxspeed;
        if (PhiPrime[l] < -maxspeed):
            PhiPrime[l] = -maxspeed;

        ## STORE AND PLOT DATA.

        n = n+1; #increment loop counter

    if Stop:
        print u"Execution Stopped!\n";
    else :
        print u"!> End condition reached: execution complete!";
    
    print "!> %d steps" % Steps,"and %d iterations was done" % n

    return (trajectory, startPose, n)
