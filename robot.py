#!/usr/bin/python
# Original matlab code from ETHZ

from numpy import *
from replot import *

import scipy as Sci
import scipy.linalg

#S = array([0,0])
#dS = array([0,0])
#robotConst = ([0, 0, 0.142]);
robotConst=array([0, 0]);
# Wheels numbers: 0 - right; 1 - left.
r=0
l=1

Stop=0

def ContStep(dS, S, goalPose, pose, startPose, robotConst, Time, PhiPrime):
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
	
	theta = (60)*(2*pi/360);
	rho = 1.8;
	alpha = (45)*(2*pi/360);
	beta = (0)*(2*pi/360);
	
	OpenLoop = 1;
	BackwardOK = 1;
	
	#Time = 10;# Set in upper file
	#PhiPrime = array([0,0])
	
	if OpenLoop == 1:
		EndCond = 0;
		if Time < 5:
			PhiPrime[l] = 10;
			PhiPrime[r] = 0;
		else :
			PhiPrime[l] = 0;
			PhiPrime[r] = 0;
			EndCond = 1;
			
		dx = ((dSl + dSr) / 2) * cos(theta);
		dy = ((dSl + dSr) / 2) * sin(theta);
		
		x = x + dx;
		y = y + dy;
		theta = theta + (dSr - dSl) / (2 * halfWheelbase);
		
		theta1=mod(theta,2*pi);
		if theta1 > pi:
			theta1=theta1-2*pi;
		
		theta=theta1;
		
		pose = array([x, y, theta]);
		#rho = 0;
		#alpha = 0;
		#beta = 0; 
	return (PhiPrime, rho, alpha, beta, pose, EndCond)

def StoreData(pose):
	print "Store data\n"
	trajectory[n,] = pose;
	data[n,1] = rho;
	data[n,2] = alpha;
	data[n,3] = pose[3]; # theta [rad]
	data[n,4] = encoder[1]; # right wheel [rad]
	data[n,5] = encoder[2]; # left wheel [rad]
	data[n,6] = dEncoder[1]; # right wheel [drad]
	data[n,7] = dEncoder[2]; # left wheel [drad]
	
	# New: beta, Time
	data[n,8] = beta; # beta [rad]
	data[n,9] = Time;
	data[n,10] = PhiPrime[1]; # [rad/s]
	data[n,11] = PhiPrime[2]; # [rad/s]

def PlotTrajectory():
	x = trajectory[:,1];
	y = trajectory[:,2];
	theta = trajectory[:,3];
	
	# Clear axes (in order to delete all arrows)
	# cla(handles.AxesTrajectory);
	#hold(handles.AxesTrajectory,'on');
	# Redraw the whole trjectory
	
	if n != 1 :
		for i in range(2,n):
			#plot(handles.AxesTrajectory,[x(i-1); x(i)], [y(i-1); y(i)], 'LineWidth', 2); # draw the trajectory
			i=i;
	
	# Draw the last arrow
	arrowlength = 0.8;
	
	#if n!=1 :
	#	#quiver(handles.AxesTrajectory,x(n),y(n),arrowlength*cos(theta(n)),arrowlength*sin(theta(n)),'Color','r','MaxHeadSize',2.5);
	
	# Draw goal position
	#rectangle('Position',[goalPose(1)-goalPose(4), goalPose(2)-goalPose(4), 2*goalPose(4), 2*goalPose(4)], 'Curvature',[1,1],'EdgeColor',[1 0 1],'LineWidth',2,'Parent',handles.AxesTrajectory);
	#plot(handles.AxesTrajectory,startPose(1), startPose(2), 'ro', 'LineWidth',2); 
	# draw the initial position of the robot
	
	#drawnow(); # continuously update plo
	

def PlotData():
	print "plot data!"
	# Distance
	#hold(handles.AxesDistance,'on');
	
	#data_new = cat(2, data(:,2), data(:,8));
	#data_new = cat(2, data_new, data(:,3));
	
	#global AX;
	#[AX,H1,H2] = plotyy(handles.AxesDistance,data(:,9), data(:,1), data(:,9), data_new, 'plot', 'plot');
	#legend(AX(1),'Distance');
	#leg_handle = legend(AX(2),'Alpha', 'Beta', 'Omega');
	#set(leg_handle, 'Color', 'w');
	
	# Commands (output of the controller) => phiprime
	#hold(handles.AxesAngles,'on');
	#plot(handles.AxesAngles,data(:,9), data(:,10), 'r');
	#plot(handles.AxesAngles,data(:,9), data(:,11), 'b');
	#legend(handles.AxesAngles,'PhiPrime(r)', 'PhiPrime(l)');
	#hold(handles.AxesAngles,'off');
	
	# Odometry
	#hold(handles.AxesOdoAngle,'on');
	#stairs(handles.AxesOdoAngle,data(:,9), data(:,4), 'b');
	#stairs(handles.AxesOdoAngle,data(:,9), data(:,5), 'r');
	#legend(handles.AxesOdoAngle,'Encoder (Right)', 'Encoder (Left)', 'Location', 'Best');
	#hold(handles.AxesOdoAngle,'off');
	
	#hold(handles.AxesOdoSpeed,'on');
	# Inputs (phiprime * dt = dphi
	#if SIMULATE :
	#	plot(handles.AxesOdoSpeed,data(:,9), data(:,10)*SAMPLINGTIME, 'm');
	#	plot(handles.AxesOdoSpeed,data(:,9), data(:,11)*SAMPLINGTIME, 'c');
	#else :
	#	plot(handles.AxesOdoSpeed,data(:,9), data(:,10), 'm');
	#	plot(handles.AxesOdoSpeed,data(:,9), data(:,11), 'c');
	
	# Outputs
	#plot(handles.AxesOdoSpeed,data(:,9), data(:,6), 'r');
	#plot(handles.AxesOdoSpeed,data(:,9), data(:,7), 'b');
	#legend(handles.AxesOdoSpeed,'dPhi(r)', 'dPhi(l)', 'dEncoder(r)', 'dEncoder(l)', 'Location', 'Best');
	#hold(handles.AxesOdoSpeed,'off');
	
	#drawnow();

def initvars():
	print "Init variables\n"
	OUTBOUNDS = 5; #if the robot is OUTBOUNDS (m) distance away from the goal, then the controller has probably exploded
	SAMPLES = 50; #max number of samples to take
	if simulate:
		SAMPLINGTIME = 0.15;
	## Initialise controller
	startPose = ([0, 0, pi/2]);
	goalPose = ([0, 0, pi/3]);
	pose = startPose; # current position
	PhiPrime = ([0, 0]); # speed of each wheel [rad/s]
	S = ([0, 0]); # accumulated encoder values [m]
	dS = ([0, 0]); # encoder value since last time step [m]
	rho = OUTBOUNDS; # distance from the goal
	
	## INITIALIZE SOME VARIABLES.
	trajectory = multiply(ones((SAMPLES, 3), float), nan);
	data = multiply(ones((SAMPLES, 10), float), nan);
	
	return (PhiPrime, SAMPLINGTIME, S, pose, goalPose, startPose)

def simulate(robotConst):
	print "run\n"
	##clc; # clear console
	#global tt;
	#global Stop;
	## INITIALIZE VARIABLES.
	PhiPrime, SAMPLINGTIME, S, pose, goalPose, startPose = initvars();
	
	wheelRadiusValue = 0.027;
	wheelbaseValue = 0.142;
	KrhoValue = 0.15;
	KalphaValue = 0.5;
	KbetaValue = -0.2;
	goalXValue = 0.5;
	goalYValue = 0.5;
	goalThetaValue = 0.0;
	goalErrorDistValue = 0.1;
	goalErrorAngleValue = 25;
	
	EndCondition = 0;
	
	goalPose = array([ goalXValue, goalYValue, goalThetaValue, goalErrorDistValue, goalErrorAngleValue*pi/180]); # sets the robot goal pose
	
	robotConst[0] = wheelRadiusValue; # wheel radius
	robotConst[1] = wheelbaseValue/2; # 1/2 wheelbase
	
	encoder = ([0, 0]);
	## CONTROL LOOP.
	n = 1;
	EndCond = 0;
	
	#tic;
	while ((not EndCond) and (not Stop)):
			## ESTIMATE TRAVELLED DISTANCES.
			dEncoder = multiply(PhiPrime, SAMPLINGTIME); # interpolate encoder value for simulation
			encoder = encoder + dEncoder;
			dS = dEncoder * robotConst[0]; # calculate change in displacement from previous time step
			S = S + dS; # accumulate total displacement
			Time = n*SAMPLINGTIME;
			
			## CONTROL STEP.
			[PhiPrime, rho, alpha, beta, pose, EndCond] = ContStep(dS, S, goalPose, pose, startPose, robotConst, Time, PhiPrime); # run the control step
			print "Pose x = %f" % pose[0]
			print "Pose y=%f" % pose[1]
			print "Pose theta=%f" %  pose[2] 
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
			#StoreData(pose);
			#PlotTrajectory();
			n = n+1; #increment loop counter
			
			if Stop:
				print "Execution Stopped!\n";
			else :
				print "End condition reached: execution complete!";
			PlotData;
		
	

simulate(robotConst)
