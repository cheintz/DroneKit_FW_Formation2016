from dronekit import connect, VehicleMode, Vehicle
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import jsonpickle
import math as m
from datetime import datetime, timedelta
import numpy as np
import signal
import copy
from pid import PIDController

acceptableControlMode = VehicleMode("FBWA")

logging.basicConfig(level=logging.WARNING)


	

class Controller(threading.Thread): 	#Note: This is a thread, not a process,  because the DroneKit vehicle doesn't play nice with processes. 
					#There is little to no performance problem, because the "main" process doesn't do much, and 
					#so the GIL isn't an issue for speed
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime):
		threading.Thread.__init__(self)
		self.isRunning=True
		self.loggingQueue = loggingQueue
		self.transmitQueue = transmitQueue
		self.receiveQueue = receiveQueue
		self.stateVehicles = {}
		self.vehicle=vehicle
		self.parameters = defaultParams
		self.vehicleState = FullVehicleState()
		self.vehicleState.bvs.ID = int(self.vehicle.parameters['SYSID_THISMAV'])
		self.vehicleState.startTime = datetime.now()
		self.vehicleState.bvs.counter = 0
		self.trimThrottle= self.vehicle.parameters['TRIM_THROTTLE'] 
		self.rollToThrottle = self.vehicle.parameters['TECS_RLL2THR'] 
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime
		
		k = defaultParams.gains
		self.rollController = PIDController(k['kTheta'],-k['rollLimit'],k['rollLimit'],-k['maxETheta'],k['maxETheta'])
		self.pitchController = PIDController(k['kAlt'],-k['pitchLimit'],k['pitchLimit'],-k['maxEAlt'],k['maxEAlt'])
		self.throttleController = PIDController(k['kSpeed'],0,100,-k['maxESpeed'],k['maxESpeed'])

	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		#signal.signal(signal.SIGINT, signal.SIG_IGN) #not needed because this is a thread in the same process as flightProgram.py
		print "AccelFirst: " + str(self.vehicleState.bvs.fwdAccel)
		while(not self.stoprequest.is_set()):#not self.kill_received):
			loopStartTime=datetime.now()
			while(not self.stoprequest.is_set()):
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
					#self.receiveQueue.task_done() #May or may not be helpful
				#except Queue.Empty:
				except:
					break #no more messages.
			self.getVehicleState() #Get update from the Pixhawk
			print "RelTime: " + str((datetime.now() - self.startTime).total_seconds())
			print "counter: " + str(self.vehicleState.bvs.counter)

			if(not self.vehicleState.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.vehicleState.isFlocking and True): #self.vehicleState.ID != self.parameters.leaderID):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
					self.computeControl() #writes the control values to self.vehicleState
					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.stateVehicles[self.vehicleState.bvs.ID] = self.vehicleState.bvs
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
#			self.vehicleState.RCLatch = False
#			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			if(not self.vehicleState.isFlocking): #extra precaution to ensure control is given back
				self.releaseControl()
			timeToWait = max(self.parameters.Ts - (datetime.now() -loopStartTime).total_seconds(), 1E-6)
		#	print "Waiting: " + str(timeToWait)
			time.sleep(timeToWait) #variable pause
			
				#TODO: find a way to clear timeouts, if necessary
		self.stop()
		self.releaseControl()
		self.vehicle.close()			
		print "Control Stopped"
			
	def updateGlobalStateWithData(self,msg):
		if (msg.type == "UAV"):
			self.parseUAVMessage(msg)
		else: #From GCS
			self.parseGCSMessage(msg)
		
	def parseUAVMessage(self,msg):
		if(msg.content.ID>0):
			ID=int(msg.content.ID)
			self.stateVehicles[ID] = msg.content
			self.vehicleState.timeout.peerLastRX[ID]=datetime.now()
			if(not self.vehicleState.isFlocking): #reset accumulated position error if not flocking
				self.vehicleState.controlState.accPosError[msg.content.ID] = np.matrix([[0],[0]])
			
	def scaleAndWriteCommands(self):

		xPWM = self.vehicleState.command.rollCMD * self.parameters.rollGain+self.parameters.rollOffset
		yPWM = self.vehicleState.command.pitchCMD*self.parameters.pitchGain + self.parameters.pitchOffset
		zPWM = self.vehicleState.command.throttleCMD*self.parameters.throttleGain + self.parameters.throttleMin

		xPWM = saturate(xPWM,1000,2000)
		yPWM = saturate(yPWM,1000,2000)
		zPWM = saturate(zPWM,1000,2000)

		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}

	def releaseControl(self):
		self.vehicle.channels.overrides = {}
#		print "releasing control"
#		print self.vehicle.channels.overrides 
		self.vehicleState.controlState = ControlState()
		self.rollController.reset()
		self.pitchController.reset()
		self.throttleController.reset()
#		self.vehicleState.controlState.accAltError=0
#		self.vehicleState.controlState.accHeadingError=0
#		self.vehicleState.controlState.accAirspeedError=0
		 #This is handled in parseMessage self.vehicleState.accPosError[(self.parameters.leaderID)

	def checkAbort(self): #only call if flocking!!
		# print "in checkAbort" + str(time.time())
		if(self.checkTimeouts()): #If we had a timeout
			"Abort - Timeout" + str(datetime.now())
			self.vehicleStateabortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
			self.releaseControl()
			self.commenceRTL()
			self.vehicleState.command = Command()			
			return True
		print "Flight Mode: " + str(self.vehicle.mode)
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Abort - control mode" + str(datetime.now())
			self.vehicleState.RCLatch = True			
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			# print "About to RTL" + str(time.time())
			self.releaseControl()			
			#self.commenceRTL()
			# print "returned from RTL function" + str(time.time())
			self.vehicleState.command = Command()			

			return True
		#if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100):
		if(False):
			print "Abort - Geofence not enabled"
			self.vehicleState.RCLatch = True
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Geofence"
			self.releaseControl()
			self.vehicleState.command = Command()			
			self.commenceRTL()
			return True
		if (self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True			
			self.abortReason = "RC Disable"
			print "RC Disable" + str(time.time())
			self.releaseControl()
			self.vehicleState.command = Command()			
			return True
		print "Do not abort flocking"
		return False
		
	def checkEngageFlocking(self):
		#Check Timeouts
		if(self.checkTimeouts()):
			print "Won't engage - Timeouts"
			self.vehicleState.RCLatch = True
			return False
		#check expected number of peers
		if(len(self.stateVehicles) < self.parameters.expectedMAVs-1):
			print "Won't engage; Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys())
			self.vehicleState.RCLatch = True
			return False	

		#Check RC enable
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Won't engage - control mode" 
			print "In Mode: "  + str(self.vehicle.mode)
			self.vehicleState.RCLatch = True			
			return False
		if(False):			
	#	if(self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100): #Geofence
			print "Won't engage. Geofence not enabled"
			print "Ch7: " +str(self.vehicle.channels['7'])
			self.vehicleState.RCLatch = True
			return False
		if(self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			print "Won't engage. Channel 6 = " + str(self.vehicle.channels['6'])
			self.vehicleState.RCLatch = False #We got this far, which means that the only issue is the enable. Thus, if they enable, we can engage
			return False
		elif(self.vehicleState.RCLatch == True): #Catch the latch to ensure any new passing condition doesn't cause flocking to (re)start
			return False

		self.vehicleState.RCLatch = True #Set the latch
		self.vehicleState.isFlocking= True #enable flocking
		print "OK to engage flocking"
		return True
			
	def getVehicleState(self):		#Should probably check for timeout, etc.
		self.vehicleState.timeout.peerLastRX[self.vehicleState.bvs.ID]=datetime.now()
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()

		if self.vehicleState.bvs.timestamp is None: #startup calculation of Ts
			self.vehicleState.bvs.timestamp = datetime.now() 
			Ts = self.parameters.Ts
		else:
			print self.vehicleState.bvs.timestamp
			Ts = (datetime.now() - self.vehicleState.bvs.timestamp).total_seconds()

		#Record sample time 
		self.vehicleState.bvs.counter+=1
		self.vehicleState.bvs.propagated = 0
		self.vehicleState.bvs.timestamp = datetime.now()
		self.thisTS = Ts

		#Cache old values for filters
		lastHeading = self.vehicleState.bvs.courseAngle.heading
		lastSpeed = self.vehicleState.groundspeed
		lastHeadingRate = self.vehicleState.bvs.courseAngle.headingRate
		lastHeadingAccel = self.vehicleState.bvs.courseAngle.headingAccel
		
		self.vehicleState.bvs.velocity = self.vehicle.velocity
		self.vehicleState.bvs.courseAngle.heading = m.atan2(self.vehicleState.bvs.velocity[0],self.vehicleState.bvs.velocity[1])
		self.vehicleState.groundspeed = np.linalg.norm(np.matrix([[self.vehicleState.bvs.velocity[1]],[self.vehicleState.bvs.velocity[0]]]),2)

	#copy other states over
		self.vehicleState.airspeed=self.vehicle.airspeed
		self.vehicleState.attitude = self.vehicle.attitude

		#print self.vehicle.channels


		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
		self.vehicleState.bvs.position = self.vehicle.location.global_relative_frame

#		print "postime" + str(self.vehicleState.position.time)
		
#		print (datetime.now() -self.vehicleState.position.time).total_seconds() #to check the timing


		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.imuAccel = self.vehicle.acceleration
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.parameters = self.parameters
		self.vehicleState.servoOut = self.vehicle.servoOut

	#Heading Rates
		#Filter startup handling
		if self.vehicleState.bvs.courseAngle.headingRate is None:
			lastHeading = self.vehicleState.courseAngle.heading
			self.vehicleState.courseAngle.headingRate = 0
			lastHeadingRate = 0
			self.vehicleState.courseAngle.headingAccel = 0
			lastHeadingAccel = 0
		if self.vehicleState.bvs.fwdAccel is None:	
		#	print "Startup groundspeed"
			lastSpeed=self.vehicleState.groundspeed
			self.vehicleState.bvs.fwdAccel = 0

		aHdg = self.parameters.gains['aFilterHdg']
		deltaHeading = wrapToPi(self.vehicleState.bvs.courseAngle.heading -lastHeading)

		#self.vehicleState.headingRate = -1* 9.81*self.vehicleState.attitude.roll/ self.vehicleState.groundspeed  #Use roll for heading rate
		if(self.vehicleState.groundspeed >5):
			self.vehicleState.headingRate = -9.81/self.vehicleState.groundspeed * m.tan(self.vehicleState.attitude.roll * m.cos(self.vehicleState.attitude.pitch))
		else:
			self.vehicleState.bvs.courseAngle.headingRate = -self.vehicle.attitude.yawspeed
		#print "Roll Estimated HdgRate: " + str(self.vehicleState.headingRate)	
		#self.vehicleState.headingRate = (1- aHdg) * lastHeadingRate +aHdg/Ts * (deltaHeading)	

	#heading Accel
		self.vehicleState.bvs.courseAngle.headingAccel = (1- aHdg) * lastHeadingAccel + aHdg/Ts * (
		self.vehicleState.bvs.courseAngle.headingRate -lastHeadingRate) 	 #Use filter for heading accel		

		ATT=self.vehicleState.attitude
		s = self.vehicleState.groundspeed;
		if(s>5):
			self.vehicleState.headingAccel = np.asscalar( 9.81/ s**2 *(s*ATT.pitchspeed*m.sin(ATT.pitch)*m.tan(ATT.roll)
				-s*m.cos(ATT.pitch)*ATT.rollspeed*1/(m.cos(ATT.roll)**2) 
				+ m.cos(ATT.pitch)*m.tan(ATT.roll)*self.vehicleState.fwdAccel) )#use heuristic for heading acceleration
		else:
			self.vehicleState.bvs.courseAngle.headingAccel = 0

#		print "Filter Estimated HdgRate: " + str(self.vehicleState.headingRate) + "\t HdgAccel: " + str(self.vehicleState.headingAccel)
#		print "hdgrt:" + str( self.vehicleState.headingRate) + "\tlastHeading:" + str(lastHeading)
		
	#FwdAcceleration
		#Filter startup handling
		

		aSpd = self.parameters.gains['aFilterSpd']	
		deltaSpd = self.vehicleState.groundspeed - lastSpeed
		lastFwdAccel = self.vehicleState.bvs.fwdAccel
		self.vehicleState.bvs.fwdAccel =  (1- aSpd) * lastFwdAccel +aSpd/Ts *(deltaSpd)
#		print "accel:" + str( self.vehicleState.fwdAccel) + "\tlastSpd:" + str(lastSpeed) + "\tDeltaSpeed: " + str(deltaSpd)


		#self.vehicleState.heading = (datetime.now() -self.vehicleState.position.time).total_seconds()
#		propagateVehicleState(self.vehicleState,(datetime.now() -self.vehicleState.position.time).total_seconds()) #propagate positions forward. Note that they will not propagated repeatedly; will just propagate repeatedly from the last message received from the Pixhawk. That should be okay for "this" agent.

		
	def pushStateToTxQueue(self):
		#print "TXQueueSize = " + str(self.transmitQueue.qsize())
		msg=Message()
		msg.type = "UAV"
		msg.sendTime = datetime.now()
		msg.content = copy.deepcopy(self.vehicleState)
#		if (self.vehicleState.channels['5'] < 1900): #This was to debug timeout functionality
		self.transmitQueue.put(msg)
		return msg
	def pushStateToLoggingQueue(self):
		self.lastLogged = self.vehicleState.bvs.counter
		msg=Message()
		msg.type = "UAV_LOG"
		msg.sendTime=time.time()
		msg.content = {}
		msg.content['thisState']=copy.deepcopy(self.vehicleState)
		msg.content['stateVehicles']=copy.deepcopy(self.stateVehicles)
#		print "\t" + str(msg.content['thisState'].counter)
		self.loggingQueue.put(msg)
	def commenceRTL(self):
#		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()
	def checkTimeouts(self):
		didTimeOut = False
		if(datetime.now() - timedelta(seconds = self.lastGCSContact)<datetime.now()- timedelta(seconds=self.parameters.GCSTimeout) ):
			print "GCS Timeout - Overridden"
		#if(True):
			self.vehicleState.timeout.GCSTimeoutTime = time.time()
#			didTimeOut = True
		if(False):
#		for IDS in self.stateVehicles.keys():
			ID=int(IDS)	
			if(self.vehicleState.timeout.peerLastRX[ID]<datetime.now()-timedelta(seconds = self.parameters.peerTimeout)):
				self.vehicleState.timeout.peerTimeoutTime[ID]=datetime.now()
				print "Timeout - ID: " + str(ID)
#				print "LastRX: " + str(self.vehicleState.timeout.peerLastRX[ID]) + "\t" + 
				didTimeOut = True
			else: #propagate forward
				dt = (datetime.now() - self.stateVehicles[ID].time).total_seconds()
				if (dt>0.0001*1e12):
					propagateVehicleState(self.stateVehicles[ID],dt) #will propagate from when we received. Other agent propagates forward from its Pixhawk position update time. The actual communication latency is not included in this.
		return didTimeOut
	def parseGCSMessage(self, msg):
#		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.type == "Parameters"):
			self.parameters = msg.content
#			self.vehicleState.timeout.GCSLastRx = msg.sentTime()
			self.vehicleState.timeout.GCSLastRx = datetime.now()

		if(msg.type == 'HEARTBEAT'):
			self.vehicleState.timeout.GCSLastRx = datetime.now()
#			self.vehicleState.timeout.GCSLastRx = msg.sendTime()


	def computeControl(self):
		computeFormationControl()
		computeHeadingConrol()
		computeThrottleControl()
		computeAltitudeControl()
		
	def computeFormationControl(self):
		#overhead
		thisCommand  = Command()
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
		#LEADER = self.stateVehicles[2]
		THIS = self.vehicleState
		CS = THIS.controlState
		GAINS = THIS.parameters.ctrlGains

		qi = np.matrix(THIS.position).transpose()
		qi_gps = np.matrix([THIS.position.lat, THIS.position.lon])
		print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		print "ql_gps" + str(ql_gps)
		ID = THIS.ID
		n = THIS.parameters.expectedMAVs
		Ts =self.thisTS

		vx = THIS.velocity[1]
		vy = THIS.velocity[0]
		pi = np.matrix([[vx],[vy]])
		pl = np.matrix([[LEADER.velocity[1]],[LEADER.velocity[0]]]).transpose()
		sl =np.linalg.norm(pl,2);

		qil = getRelPos(ql_gps,qi_gps)
		qil.shape=(2,1)
		pl.shape =(2,1)

		qd = THIS.parameters.desiredPosition # qd1 ; qd2 ; qd3 ...
		qdil=qd[ID-2,np.matrix([0,1])]
		qdil.shape=(2,1)
		
		kl = GAINS['kl']
		ka = GAINS['ka']
		alpha2 = GAINS['alpha2']
		alpha1 = GAINS['alpha1']
		d = GAINS['d']

		vMin = GAINS['vMin']
		vMax = GAINS['vMax']
		gamma = np.matrix([[0,-1],[1,0]])

		phi = LEADER.CourseAngle.heading
		phiDot = LEADER.CourseAngle.headingRate #need to estimate these better
		phiDDot = LEADER.CourseAngle.headingAccel #need to estimate these better

		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		print 'Time: = ' + str(THIS.time)
	
	#Compute from leader 
		eqil = qil - Obi.transpose()* qdil
		Eqil=CS.accPosError[(self.parameters.leaderID)]
		pil = pi - (pl + phiDot * gamma * qdil) #in plane relative velocity (inertial)

		CS.plTerm = pl
		CS.phiDotTerm = phiDot * gamma * Obi.transpose()*qdil
		CS.kplTerm = -kl.kp * eqil
		
		ui = CS.plTerm + CS.phiDotTerm+ CS.kplTerm 
		CS.uiTarget = ui

		print 'UI = ' + str(ui)
		ata = np.linalg.norm(qil,2)
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2) /m.pow(d,2))
			print 'F Repel:' + str(frepel)
			ui = ui - frepel * qil 
			
	#compute from peers
		for j in range(1,n+1):
			if(ID != j and j !=THIS.parameters.leaderID):
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
				qij = getRelPos(qj_gps,qi_gps).transpose()
				qdjl = qd[j-2,0:2]
				qdjl.shape=(2,1)
				pj = np.matrix([[JPLANE.velocity[1]],[JPLANE.velocity[0]]])
				qdij = (qdil-qdjl )
				eqij = qij-Obi.transpose()*qdij
				CS.kpjTerm[j] = -ka.kp * eqij 				
				ui = CS.kpjTerm[j]
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					print 'F Repel:' + str(frepel)
					ui = ui - frepel * qij 

		CS.qldd = LEADER.acceleration.x * np.matrix([[m.cos(phi)],[m.sin(phi)]]) + phiDot * np.matrix([[-m.sin(phi)],[m.cos(phi)]]) * sl 
		CS.pdiDot = ( qldd + phiDDot * gamma * Obi.transpose() * qdil 
			-phiDot**2 *Obi.transpose()*qdil 
			-kl.kp * ( pi- pl - phiDot*gamma*Obi.transpose()*qdil))

		CS.thetaDDotApprox = np.asscalar( 1.0/(1+ui[1]**2/ui[0]**2) * (ui[0]*pdiDot[1]-ui[1]*pdiDot[0])/ui[0]**2 )
		CS.thetaD = m.atan2(ui[1,0],ui[0,0])

		speedD = np.linalg.norm(ui,2)  #Don't reduce commanded velocity based on heading error
		CS.speedD = speedD 
		CS.speedDDot = (ui.transpose() / speedD) * ( pdiDot)

		CS = self.ControlState
		THIS =  self.vehicleState

		fi = np.matrix([[m.cos(THIS.heading)],[m.sin(THIS.heading)]])

		CS.backstepSpeed = speedD
		CS.backstepSpeedError =  1/GAINS['aSpeed']* -GAINS['gamma'] * eSpeed
		CS.backstepSpeedRate = 1/GAINS['aSpeed'] * CS.speedDDot
		CS.backstepPosError =  1/GAINS['aSpeed'] * -eqil.transpose()*fi*1/GAINS['lambda']
		
		asTarget = CS.backstepSpeed + CS.backstepSpeedRate + CS.backstepSpeedRate + CS.backstepPosError
		asTarget=max(vMin,min(vMax,asTarget)) #saturate to limit
		
		CS.asTarget = asTarget
		self.controlState=CS

	#Heading Control:
	def headingControl(self):
		CS. self.vehicleState.ControlState #Note; this is a shallow copy		
		ktheta = self.parameters.ctrlGains['ktheta']
		theta = self.vehicleState.bvs.heading
	
		etheta = wrapToPi(theta-thetaD)
		CS.etheta=etheta
		calcTurnRate = THIS.headingRate 
		groundspd = np.linalg.norm(pi,2)
		accHeadingError= CS.accHeadingError

		CS.rollPTerm=	-ktheta.kp*etheta
		CS.rollITerm=	-ktheta.ki * CS.accHeadingError
		CS.rollDTerm =  -ktheta.kd * (calcTurnRate-thetaDDotApprox)
		ffTerm = GAINS['kThetaFF']*m.atan(thetaDDotApprox * groundspd / 9.81 * m.cos(THIS.attitude.pitch))
		[rollCMD, CS.rollTerms ] = self.RollController.update(etheta,calcTurnRate-CS.thetaDDot,self.thisTS,ffTerm)	
		CS.accHeadingError=self.RollController.integrator
		self.vehicleState.command.rollCMD = rollCMD

	def speedControl(self):
		CS = self.controlState #shallow copy
		THIS = self.vehicleState #shallow copy
		kspeed = GAINS['kSpeed']
		eSpeed = THIS.airspd - CS.asTarget
		ffTerm = self.trimThrottle + 1/m.pow(m.cos(this.attitude.roll),2)
		[throttleCMD, speedTerms]= self.SpeedController.update(eSpeed,THIS.fwdAccel,self.thisTS,ffTerm)
		CS.accAirspeedError=self.SpeedController.integrator
		self.vehicleState.command.throttleCMD = throttleCMD

	def altitudeControl(self):
		CS = self.controlState #shallow copy
		desiredAltitude = qd[ID-2,2] #this is AGL  for now
		altitude = self.vehicleState.position.alt
		altError = altitude-desiredAltitude
		[pitchCMD,pitchTerms]=self.altitudeController.update(altError,self.vehicleState.bvs.velocity[2],self.thisTS,0);
		CS.accAltError  = self.altitudeController.integrator
		self.vehicleState.command.pitchCMD=saturate(thisCommand.pitchCMD,-pitchLimit,pitchLimit)
		thisCommand.timestamp = datetime.now()

def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out

def wrapToPi(value):
	return wrapTo2Pi(value+m.pi)-m.pi
	
def wrapTo2Pi(value):
	if(value<0):
		n=m.ceil(abs(value / (2*m.pi)))
		value+=n*2.*m.pi
		positiveInput=False
	else:
		positiveInput=True
	value = m.fmod(value, 2*m.pi);
	if (value == 0 and positiveInput):
		value=2*m.pi
	return value
	
#def GPSToMeters(lat,long,alt):
#	r = 6371000 + alt
#	x = r*m.cosd(lat)*m.cosd(lon)
#	y = r*m.cosd(lat)*m.sind(lon)
#	z = r*m.sind(lat)
def getRelPos(pos1,pos2): #returns the x y delta position of p2-p1 with x being longitude (east positive)
	c = 40074784 # from https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters

	dx = (pos2[0,1]-pos1[0,1]) * c * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
	dy = (pos2[0,0]-pos1[0,0]) * c /360	
	return np.matrix([dx, dy])

def windHeadingToInertial(windEstimate):
	vx = windEstimate.speed * m.cos(m.radians(90-windEstimate.dir))
	vy = windEstimate.speed * m.sin(m.radians(90-windEstimate.dir))
	vz = windEstimate.speed_z
	return {'vx':vx,'vy':vy,'vz':vz}
def propagateVehicleState(state, dt): #assumes heading rate and fwdAccel are constant
	psiDot = saturate(state.headingRate,-2,2) 
	sDot = state.fwdAccel #TODO sometimes get math domain error on this
	psi = state.heading
	
	vx = state.velocity[1]
	vy = state.velocity[0]

	s = state.groundspeed
	sf = s+sDot*dt
	psif = state.heading+psiDot*dt
	
#	dx = psiDot*(sf)*m.sin(psif)+sDot*m.cos(psif)-psiDot*s*m.sin(psi)-sDot*m.cos(psi)
#	dy = -psiDot*sf*m.cos(psif)+sDot*(m.sin(psif)-m.sin(psi))+psiDot*s*m.cos(psiDot)

#	print "dx: " + str(dx)
#	print "dy: " + str(dy)

	dx = vx*dt #simplified, assumes straight line flight
	dy = vy*dt

#	print "dx: " + str(dx)
#	print "dy: " + str(dy)

	dz = state.velocity[2]*dt

	#write output back to state
	state.velocity[0] = sf * m.sin(psif)#yes; this is the Y direction velocity 
	state.velocity[1] = sf *m.cos(psif) #yes; the X velocity
	state.heading = wrapToPi(psif)

	qGPS = np.matrix([state.position.lat, state.position.lon])
	dqGPS = getRelPos(qGPS,qGPS + 1e-6) / 1e-6
	state.position.lat = state.position.lat + dy/dqGPS[0,1]
	state.position.lon = state.position.lon + dx/dqGPS[0,0]
	state.position.alt = state.position.alt + dz
	
	state.time = state.time + timedelta(seconds = dt)	
	state.propagated = 1
	
