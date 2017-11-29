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

acceptableControlMode = VehicleMode("FBWA")

logging.basicConfig(level=logging.WARNING)


	

class Controller(threading.Thread):
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime):
		threading.Thread.__init__(self)
		self.isRunning=True
		self.loggingQueue = loggingQueue
		self.transmitQueue = transmitQueue
		self.receiveQueue = receiveQueue
		self.stateVehicles = {}
		self.vehicle=vehicle
		self.parameters = defaultParams
		self.vehicleState = VehicleState()
		self.vehicleState.ID = int(self.vehicle.parameters['SYSID_THISMAV'])
		self.vehicleState.startTime = datetime.now()
		self.counter = 0
		self.trimThrottle= self.vehicle.parameters['TRIM_THROTTLE'] 
		self.rollToThrottle = self.vehicle.parameters['TECS_RLL2THR'] 
		# print "Constructor \n\n"
		# print type(self.vehicleState)
#		self.command = Command()
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime
		
		#def servoMsgHandler(self,name,m):	
		#self.vehicle.add_message_listener('SERVO_OUTPUT_RAW',self.servoMsgHandler)
		
		#@self.vehicle.on_message('SERVO_OUTPUT_RAW')
		#def handleServoOutMsg(self,name,msg)
			
		
		
		
	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		while(not self.stoprequest.is_set()):#not self.kill_received):
			while(not self.receiveQueue.empty()):
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
					self.receiveQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
			self.getVehicleState() #Get update from the Pixhawk
			print "RelTime: " + str((datetime.now() - self.startTime).total_seconds())
			print "counter: " + str(self.counter)
			if(not self.vehicleState.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.vehicleState.isFlocking and True): #self.vehicleState.ID != self.parameters.leaderID):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
					self.computeControlPID() #writes the control values to self.vehicleState
					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.stateVehicles[self.vehicleState.ID] = self.vehicleState
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
#			self.vehicleState.RCLatch = False
			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			if(not self.vehicleState.isFlocking): #extra precaution to ensure control is given back
				self.releaseControl()
			time.sleep(self.parameters.Ts)
			
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
#			print "received from:" + str(ID)
			self.stateVehicles[ID] = msg.content
			#self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime
			self.vehicleState.timeout.peerLastRX[ID]=datetime.now()
			if(not self.vehicleState.isFlocking): #reset accumulated position error if not flocking
				self.vehicleState.controlState.accPosError[msg.content.ID] = np.matrix([[0],[0]])
			
	def scaleAndWriteCommands(self):
		#print "Writing RC commands"
	#	print str(self.vehicleState.command.headingRate)
		xPWM = self.vehicleState.command.rollCMD * self.parameters.rollGain+self.parameters.rollOffset
		yPWM = self.vehicleState.command.pitchCMD*self.parameters.pitchGain + self.parameters.pitchOffset
		zPWM = self.vehicleState.command.throttleCMD*self.parameters.throttleGain + self.parameters.throttleMin

		xPWM = saturate(xPWM,1000,2000)
		yPWM = saturate(yPWM,1000,2000)
		zPWM = saturate(zPWM,1000,2000)

		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}

	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		#print "releasing control"
		#print self.vehicle.channels.overrides 
		self.vehicleState.controlState = ControlState()
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
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100):
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
		#Check configuration
		if(not self.parameters.isComplete):
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
		if(self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100): #Geofence
			print "Won't engage. Geofence not enabled"
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
		self.vehicleState.timeout.peerLastRX[self.vehicleState.ID]=datetime.now()
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
#		print	str(time.time())  +"\t" + str(self.vehicle.attitude.roll) + "\t" + str((self.vehicleState.timeout.peerTimeoutTime)) + "\t" + str((self.vehicleState.timeout.peerLastRX))
		self.vehicleState.position = self.vehicle.location.global_relative_frame
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()
		self.vehicleState.parameters = self.parameters
		lastHeading = self.vehicleState.heading
		self.vehicleState.heading = m.atan2(self.vehicleState.velocity[0],self.vehicleState.velocity[1])

		deltaHeading = wrapToPi(self.vehicleState.heading -lastHeading)
		lastHeadingRate = self.vehicleState.headingRate
		aHdg = self.parameters.ctrlGains['aFilterHdg']
		Ts = self.parameters.Ts
		self.vehicleState.headingRate = (1- aHdg) * lastHeadingRate +aHdg/Ts *(deltaHeading)
		self.vehicleState.servoOut = self.vehicle.servoOut
		
		aSpd = self.parameters.ctrlGains['aFilterSpd']
		lastAirspd = self.vehicleState.airspeed
		self.vehicleState.airspeed=self.vehicle.airspeed
		deltaAispd = self.vehicleState.airspeed - lastAirspd
		
		
		lastFwdAccel = self.vehicleState.fwdAccel
		self.vehicleState.fwdAccel =  (1- aSpd) * lastFwdAccel +aSpd/Ts *(deltaAispd)
#		print "accel:" + str( self.vehicleState.fwdAccel) + "\tlastSpd:" + str(lastAirspd)
#		print "hdgrt:" + str( self.vehicleState.headingRate) + "\tlastHeading:" + str(lastHeading)


		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.acceleration = self.vehicle.acceleration
		#print self.vehicleState.acceleration
#		print self.vehicle.wind_estimate		
#		print self.vehicleState.wind_estimate
		self.vehicleState.time = datetime.now()
		self.counter+=1
	def pushStateToTxQueue(self):
#		print "TXQueueSize = " + str(self.transmitQueue.qsize())
		msg=Message()
		msg.type = "UAV"
		msg.sendTime = datetime.now()
		#msg.content=jsonpickle.encode(self.vehicleState)
		msg.content = self.vehicleState
	#	print msg.content.attitude.roll
	#	print type(msg)
		self.transmitQueue.put(msg)
		return msg
	def pushStateToLoggingQueue(self):
		msg=Message()
		msg.type = "UAV_LOG"
		msg.sendTime=time.time()
		msg.content = {}
		msg.content['thisState']=self.vehicleState
		msg.content['stateVehicles']=self.stateVehicles
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
		for IDS in self.stateVehicles.keys():
			ID=int(IDS)	
			if(self.vehicleState.timeout.peerLastRX[ID]<datetime.now()-timedelta(seconds = self.parameters.peerTimeout)):
				self.vehicleState.timeout.peerTimeoutTime[ID]=datetime.now()
				print "Timeout - ID: " + str(ID)
#				print "LastRX: " + str(self.vehicleState.timeout.peerLastRX[ID]) + "\t" + 
				didTimeOut = True
		
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


	#using PID for inertial velocity commands, for heading comand, and for altitude
	def computeControlPID(self):
		#overhead
		thisCommand  = Command()
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
		THIS = self.vehicleState
		CS = THIS.controlState
		GAINS = THIS.parameters.ctrlGains

		qi = np.matrix(THIS.position).transpose()
		qi_gps = np.matrix([THIS.position.lat, THIS.position.lon])
		#print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		#print "ql_gps" + str(ql_gps)
		ID = THIS.ID
		n = THIS.parameters.expectedMAVs
		Ts = THIS.parameters.Ts


		print "leader roll:" + str(LEADER.attitude.roll)

		vx = THIS.velocity[1]
		vy = THIS.velocity[0]
		pi = np.matrix([[vx],[vy]])
		pl = np.matrix([[LEADER.velocity[1]],[LEADER.velocity[0]]]).transpose()

		#print 'pl = ' + str(pl)

		qil = getRelPos(ql_gps,qi_gps) # 2 i - l
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

		phi = LEADER.heading
		phiDot = LEADER.headingRate

		#phi = m.pi / 2  #makes this an inertial frame relative position problem
		#phiDot = 0

		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		print 'Time: = ' + str(THIS.time)
		

		
		#Compute from leader 
		eqil = qil - Obi.transpose()* qdil
		Eqil=CS.accPosError[(self.parameters.leaderID)]
		pil = pi - (pl + phiDot * gamma * qdil) #in plane relative velocity (inertial)

		CS.plTerm = pl
		CS.phiDotTerm = phiDot * gamma *Obi.transpose()*  qdil
		CS.kplTerm = -kl.kp * eqil
		CS.kilTerm= -kl.ki *Eqil
		CS.kdlTerm = -kl.kd * (pil) #phi dot included in pil
		
		ui = CS.plTerm + CS.phiDotTerm+ CS.kplTerm + CS.kilTerm + CS.kdlTerm 
		CS.uiTarget = ui
		
		#integrate position error
		Eqil= antiWindupVec(eqil, -vMax,vMax, Eqil, eqil*Ts)
		Eqil = Eqil / (max(1,np.linalg.norm(Eqil,2)/GAINS['maxEqil'])) #more saturation
		CS.accPosError[(self.parameters.leaderID)] = Eqil

		print 'UI = ' + str(ui)
		ata = np.linalg.norm(qil,2)
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2) /m.pow(d,2))
			print 'F Repel:' + str(frepel)
			ui = ui - frepel * qil 
			
		#compute from peers

		for j in range(1,n+1):
			#print "in loop for plane:" + str(j)
			if(ID != j and j !=THIS.parameters.leaderID):
				print "Computing peer control based on plane " + str(j)
				print self.stateVehicles.keys()
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
				#print 'qj_gps' + str(qj_gps)
				qij = getRelPos(qj_gps,qi_gps).transpose()
				#print 'qij: ' + str(qij)
				qdjl = qd[j-2,0:2]
				qdjl.shape=(2,1)
				#print 'qdjl: ' + str(qdjl)
				pj = np.matrix([[JPLANE.velocity[1]],[JPLANE.velocity[0]]])
			#	print "pj: " + str(pj)
				qdij = -(qdil-qdjl )
				eqij = qij-Obi.transpose()*qdij
			#	print "eqij: " + str(eqij)
				Eqij=CS.accPosError[j]
			#	print "Eqij: " + str(Eqij)
				print "ka: " + str(ka.kp)
				
				ui = ui-ka.kp * eqij  - ka.ki * Eqij - ka.kd * (pi-(pj+0*phiDot*gamma*qdij)) #not logging this 
				Eqij = antiWindupVec(ui, -vMax,vMax, Eqij, eqij*Ts)
				Eqij = Eqij / (max(1,np.linalg.norm(Eqij,2)/100)) #more saturation
				CS.accPosError[j] = Eqij
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					print 'F Repel:' + str(frepel)
					ui = ui - frepel * qij 
		

	#Heading Control:
		
		ktheta = self.parameters.ctrlGains['ktheta']
		
		theta = THIS.heading
		thetaD = m.atan2(ui[1,0],ui[0,0])
		amp = 0.2
		f= 0.3
		t = (datetime.now() - THIS.startTime).total_seconds()

		thetaD = amp*m.sin(f*2*m.pi*t)

		
		
		lastThetaDDotApprox = CS.thetaDDotApprox

		a = GAINS['aFilterThetaDDot']
		Ts = THIS.parameters.Ts
		if not CS.thetaD:
			CS.thetaD=thetaD #Handle startup with zero thetaDDotApprox
		#	print "startup"
		thetaDLast = CS.thetaD
	#	print "thetaDLast" + str(thetaDLast)  + " " + str(thetaD)

		thetaDDotApprox  = (1- a) * lastThetaDDotApprox +a/Ts *wrapToPi(thetaD-thetaDLast)
		thetaDDotApprox = 2*m.pi*f * amp*m.cos(f*2*m.pi*t)
		CS.thetaDDotApprox = thetaDDotApprox 
		CS.thetaD=thetaD




		print "qil Follower Body: " + str(np.matrix([[m.cos(theta),m.sin(theta)],[-m.sin(theta),m.cos(theta)]])*qil)
		print "qil Intertial: " + str(qil)

		etheta = wrapToPi(theta-thetaD)
		CS.etheta=etheta

		print "etheta: " + str(etheta)		


		CS.thetaD = thetaD 
		eq = Obi*qil-qdil #this is in leader body, only want the first 2 elements
		eq.shape=(2,1)

		#heading control

		#calcTurnRate = 9.81*THIS.attitude.roll/ groundspd #Used by ArduPlane
		#calcTurnRate = THIS.attitude.yawspeed #using raw gyro (since no MAVlink data for the ahrs estimated yaw rate)
		calcTurnRate = THIS.headingRate # (numerically differentiation of heading

		groundspd = np.linalg.norm(pi,2)

		accHeadingError= CS.accHeadingError
		#trying lead control
		b1 = 0.8825
		a1 = 0.9910     # where u = etheta and y = CS.rollPTerm
		a0 = 25.58*2.3/10
		
		#CS.rollPTerm=-(etheta - b1*CS.e1 + a0*a1*CS.u1) / a0
		CS.rollPTerm = -(b1*CS.e1 + a0*etheta-a0*a1*CS.u1)
		CS.e1 = CS.rollPTerm
		CS.u1 = etheta
		
		#CS.rollPTerm=	ktheta.kp*etheta
		CS.rollITerm=	ktheta.ki * CS.accHeadingError
		CS.rollDTerm =  ktheta.kd * (calcTurnRate-thetaDDotApprox)
		#CS.rollFFTerm = GAINS['kThetaFF']*(thetaDDotApprox * groundspd / 9.81)
		#print "arg: " + str(thetaDDotApprox*groundspd)
		#print "tangent
		CS.rollFFTerm = -GAINS['kThetaFF']*(m.atan(thetaDDotApprox*groundspd/9.81) )
		#CS.rollFFTerm = -GAINS['kThetaFF']*LEADER.attitude.roll
		print "FFTerm " + str(CS.rollFFTerm)


		print "Etheta: " + str(CS.accHeadingError)
		print "thetaDDot: " + str(CS.thetaDDotApprox)
		
		rollCMD =CS.rollPTerm + CS.rollITerm  + CS.rollDTerm +CS.rollFFTerm

		
		

		

		rollLimit=GAINS['rollLimit']
	
		#print "rollLimit: " + str(rollLimit)
		#print "unsaturatedRoll: " + str(rollCMD)
		
		rollCMD = saturate(rollCMD,-rollLimit,rollLimit)
		#print "saturatedRoll: " + str(rollCMD)
		thisCommand.rollCMD =rollCMD
		print 'RollTarget:' + str(thisCommand.rollCMD)# str((180/m.pi)*thisCommand.rollCMD)

		accHeadingError = antiWindup(rollCMD,-rollLimit,rollLimit,accHeadingError,etheta*Ts)
		accHeadingError = saturate(accHeadingError,-GAINS['maxETheta'],GAINS['maxETheta'])		
		
		CS.accHeadingError=accHeadingError


		#speed control
		speedD = np.linalg.norm(ui,2) * m.cos(theta-thetaD) #reduce commanded velocity based on heading error
		CS.speedD = speedD	
		
		groundspd = np.linalg.norm(pi,2)
		airspd = THIS.airspeed
		#print 'groundspeed: '+str(groundspd)
		#print 'airspeed: ' + str(airspd)

		speedD = 20

		
		asTarget = speedD + (airspd-groundspd)
		
		asTarget=max(vMin,min(vMax,asTarget)) #saturate to limit

		CS.asTarget = asTarget

		kspeed = GAINS['kSpeed']
		rollAngle = THIS.attitude.roll
		eSpeed = airspd - asTarget

		accAirspeedError=CS.accAirspeedError

		CS.throttlePTerm=	- kspeed.kp * eSpeed
		CS.throttleITerm=	-kspeed.ki * accAirspeedError
		CS.throttleDTerm =  - kspeed.kd* THIS.fwdAccel
		CS.throttleFFTerm = self.trimThrottle + 1/m.pow(m.cos(rollAngle),2)
		
		thisCommand.throttleCMD = CS.throttlePTerm + CS.throttleITerm +CS.throttleDTerm+CS.throttleFFTerm
		
		accAirspeedError = antiWindup(CS.asTarget, vMin,vMax,accAirspeedError, eSpeed*Ts)
		accAirspeedError = saturate(accAirspeedError,-20,20)		
		CS.accAirspeedError=accAirspeedError


		#altitude control
		print 'qd id ' + str(ID) + str(qd[ID-2,:])
		desiredAltitude = qd[ID-2,2] #this is AGL  for now
		altitude = THIS.position.alt
		kalt = self.parameters.ctrlGains['kalt']
		
		altError = altitude-desiredAltitude
		accAltError = CS.accAltError
		pitchLimit=self.parameters.ctrlGains['pitchLimit']
		print " AltSpeed: " + str(THIS.velocity[2])


		CS.pitchPTerm = -kalt.kp* altError
		CS.pitchITerm = - kalt.ki*accAltError
		CS.pitchDTerm = - kalt.kd * THIS.velocity[2] #indexed from 0 (x)
		thisCommand.pitchCMD = CS.pitchPTerm + CS.pitchITerm + CS.pitchDTerm

		accAltError = antiWindup(thisCommand.pitchCMD,-pitchLimit,pitchLimit, accAltError,altError*Ts)
		accAltError = saturate(accAltError,-GAINS['maxEAlt'],GAINS['maxEAlt']) 
		CS.accAltError  = accAltError
		thisCommand.pitchCMD=saturate(thisCommand.pitchCMD,-pitchLimit,pitchLimit)
		print "ErrorNorm: " + str(np.linalg.norm(eqil,2))

		print '\n\n\n'
		
		thisCommand.timestamp = datetime.now()
		THIS.command = thisCommand
def saturate(value, minimum, maximum):
	#print "minimum: " + str(minimum)
	out = max(value,minimum)
	#print "maximum: " + str(maximum)
	out = min(out,maximum)
	#print "out2: " + str(out)
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

def antiWindup(value, lowLimit,highLimit, accumulator, toAdd):
	
	if(value>highLimit): #Saturation and anti-windup
		if(toAdd>0):
			accumulator =accumulator+toAdd
	if(value<lowLimit):
		if(toAdd < 0):
			accumulator =accumulator+toAdd		
	else:
		accumulator =accumulator+toAdd
	return accumulator

def antiWindupVec(value, lowLimit,highLimit, accumulator, toAdd):
#	print "value" + str(value)
#	print 'llim' + str(lowLimit)
#	print 'hlim' + str(highLimit)
#	print 'acc' + str(accumulator)
#	print 'toAdd' + str(toAdd)
	for i in range(0,len(value)):
		if(value[i]>highLimit): #Saturation and anti-windup
			if(toAdd[i]>0):
				accumulator[i] =accumulator[i]+toAdd[i]
		if(value[i]<lowLimit):
			if(toAdd[i] < 0):
				accumulator[i] =accumulator[i]+toAdd[i]			
		else:
			accumulator[i] =accumulator[i]+toAdd[i]
	return accumulator





