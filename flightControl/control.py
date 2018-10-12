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
		self.vehicleState.ID = int(self.vehicle.parameters['SYSID_THISMAV'])
		self.vehicleState.startTime = datetime.now()
		self.vehicleState.counter = 0
		self.trimThrottle= self.vehicle.parameters['TRIM_THROTTLE'] 
		self.rollToThrottle = self.vehicle.parameters['TECS_RLL2THR'] 
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime


	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		#signal.signal(signal.SIGINT, signal.SIG_IGN) #not needed because this is a thread in the same process as flightProgram.py
		print "AccelFirst: " + str(self.vehicleState.fwdAccel)
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
			#print "RelTime: " + str((datetime.now() - self.startTime).total_seconds())
			#print "counter: " + str(self.vehicleState.counter)

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
#			print "received from:" + str(ID)
			self.stateVehicles[ID] = msg.content
			#self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime
			self.vehicleState.timeout.peerLastRX[ID]=datetime.now()
			if(not self.vehicleState.isFlocking): #reset accumulated position error if not flocking
				self.vehicleState.controlState.accPosError[msg.content.ID] = np.matrix([[0],[0]])
			
	def scaleAndWriteCommands(self):
		print "Writing RC commands"
	#	print str(self.vehicleState.command.heading.rate)
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
		self.vehicleState.timeout.peerLastRX[self.vehicleState.ID]=datetime.now()
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()

		if self.vehicleState.timestamp is None: #startup calculation of Ts
			self.vehicleState.time = datetime.now() 
			Ts = self.parameters.Ts
		else:
			Ts = (datetime.now() - self.vehicleState.time).total_seconds()

		#Record sample time 
		self.vehicleState.counter+=1
		self.vehicleState.propagated = 0
		self.vehicleState.time = datetime.now()
		self.thisTS = Ts

		#Cache old values for filters
		lastHeading = self.vehicleState.heading.value
		lastSpeed = self.vehicleState.groundspeed
		lastHeadingRate = self.vehicleState.heading.rate
		lastHeadingAccel = self.vehicleState.heading.accel
		
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.heading.value = m.atan2(self.vehicleState.velocity[0],self.vehicleState.velocity[1])
		self.vehicleState.groundspeed = np.linalg.norm(np.matrix([[self.vehicleState.velocity[1]],[self.vehicleState.velocity[0]]]),2)

	#copy other states over
		self.vehicleState.airspeed=self.vehicle.airspeed
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
		self.vehicleState.position = self.vehicle.location.global_relative_frame
		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.acceleration = self.vehicle.acceleration
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.parameters = self.parameters
		self.vehicleState.servoOut = self.vehicle.servoOut

	#Heading Rates
		#Filter startup handling
		if self.vehicleState.heading.rate is None:
			lastHeading = self.vehicleState.heading.value
			self.vehicleState.heading.rate = 0
			lastHeadingRate = 0
			self.vehicleState.heading.accel = 0
			lastHeadingAccel = 0
		if self.vehicleState.fwdAccel is None:	
			lastSpeed=self.vehicleState.groundspeed
			self.vehicleState.fwdAccel = 0
		aHdg = self.parameters.gains['aFilterHdg']
		deltaHeading = wrapToPi(self.vehicleState.heading.value -lastHeading)

		#self.vehicleState.heading.rate = -1* 9.81*self.vehicleState.attitude.roll/ self.vehicleState.groundspeed  #Use roll for heading rate
		if(self.vehicleState.groundspeed >5):
			self.vehicleState.heading.rate = -9.81/self.vehicleState.groundspeed * m.tan(self.vehicleState.attitude.roll * m.cos(self.vehicleState.attitude.pitch))
		else: #low speed condition; don't divide by small groundspeed
			self.vehicleState.heading.rate = -self.vehicle.attitude.yawspeed
		#self.vehicleState.heading.rate = (1- aHdg) * lastHeadingRate +aHdg/Ts * (deltaHeading)	

	#heading Accel
		self.vehicleState.heading.accel = (1- aHdg) * lastHeadingAccel + aHdg/Ts * (
		self.vehicleState.heading.rate -lastHeadingRate) 	 #Use filter for heading accel		

		ATT=self.vehicleState.attitude
		s = self.vehicleState.groundspeed;
		if(s>5):
			self.vehicleState.heading.accel = np.asscalar( 9.81/ s**2 *(s*ATT.pitchspeed*m.sin(ATT.pitch)*m.tan(ATT.roll)
				-s*m.cos(ATT.pitch)*ATT.rollspeed*1/(m.cos(ATT.roll)**2) 
				+ m.cos(ATT.pitch)*m.tan(ATT.roll)*self.vehicleState.fwdAccel) )#use heuristic for heading acceleration
		else:
			self.vehicleState.heading.accel = 0

	#FwdAcceleration
	#Filter startup handling
		aSpd = self.parameters.gains['aFilterSpd']	
		deltaSpd = self.vehicleState.groundspeed - lastSpeed
		lastFwdAccel = self.vehicleState.fwdAccel
		self.vehicleState.fwdAccel =  (1- aSpd) * lastFwdAccel +aSpd/Ts *(deltaSpd)

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
		self.lastLogged = self.vehicleState.counter
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


	#using PID for inertial velocity commands, for heading comand, and for altitude
	def computeControlPID(self):
		#overhead
		thisCommand  = Command()
		#LEADER = self.stateVehicles[(self.parameters.leaderID)]
		LEADER = self.stateVehicles[2]
		THIS = self.vehicleState
		CS = THIS.controlState
		GAINS = THIS.parameters.gains

		qi = np.matrix(THIS.position).transpose()
		qi_gps = np.matrix([THIS.position.lat, THIS.position.lon])
		print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		print "ql_gps" + str(ql_gps)
		ID = THIS.ID
		n = THIS.parameters.expectedMAVs
		Ts =self.thisTS


		#print "leader roll:" + str(LEADER.attitude.roll)

		vx = THIS.velocity[1]
		vy = THIS.velocity[0]
		pi = np.matrix([[vx],[vy]])
		pl = np.matrix([[LEADER.velocity[1]],[LEADER.velocity[0]]]).transpose()
		sl =np.linalg.norm(pl,2);

		#print 'pl = ' + str(pl)

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

		phi = LEADER.heading
		phiDot = LEADER.heading.rate #need to estimate these better
		phiDDot = LEADER.heading.accel #need to estimate these better

		#phi = m.pi / 2  #makes this an inertial frame relative position problem
		#phiDot = 0

		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		print 'Time: = ' + str(THIS.time)
	
	#Compute from leader 
		eqil = qil - Obi.transpose()* qdil
		Eqil=CS.accPosError[(self.parameters.leaderID)]
		pil = pi - (pl + phiDot * gamma * qdil) #in plane relative velocity (inertial)

		CS.plTerm = pl
		CS.phiDotTerm = phiDot * gamma * Obi.transpose()*qdil
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
#			print "in loop for plane:" + str(j)
			if(ID != j and j !=THIS.parameters.leaderID):
#				print "Computing peer control based on plane " + str(j) + "\n\n"
#				print self.stateVehicles.keys()
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
#				print 'qj_gps' + str(qj_gps)
				qij = getRelPos(qj_gps,qi_gps).transpose()
#				print 'qij: ' + str(qij)
				qdjl = qd[j-2,0:2]
				qdjl.shape=(2,1)
#				print 'qdjl: ' + str(qdjl)
				pj = np.matrix([[JPLANE.velocity[1]],[JPLANE.velocity[0]]])
			#	print "pj: " + str(pj)
				qdij = (qdil-qdjl )
				eqij = qij-Obi.transpose()*qdij
			#	print "eqij: " + str(eqij)
				Eqij=CS.accPosError[j]
			#	print "Eqij: " + str(Eqij)
								
				ui = ui-ka.kp * eqij  - ka.ki * Eqij
				Eqij = antiWindupVec(ui, -vMax,vMax, Eqij, eqij*Ts)
				Eqij = Eqij / (max(1,np.linalg.norm(Eqij,2)/100)) #more saturation
				CS.accPosError[j] = Eqij
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					print 'F Repel:' + str(frepel)
					ui = ui - frepel * qij 

		qldd = LEADER.acceleration.x * np.matrix([[m.cos(phi)],[m.sin(phi)]]) + phiDot * np.matrix([[-m.sin(phi)],[m.cos(phi)]]) * sl 
		pdiDot = ( qldd + phiDDot * gamma * Obi.transpose() * qdil 
			-phiDot**2 *Obi.transpose()*qdil 
			-kl.kp * ( pi- pl - phiDot*gamma*Obi.transpose()*qdil))

	#Heading Control:		
		ktheta = self.parameters.gains['ktheta']
		theta = THIS.heading
		thetaD = m.atan2(ui[1,0],ui[0,0])

		#for step response:
		pdiDot = np.matrix([[0],[0]])
		qldd = 0
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100):
			thetaD = 0
		else:
			thetaD = 0.8
		ui = np.matrix([[m.cos(thetaD)],[m.sin(thetaD)]]) * 20		
		
		lastThetaDDotApprox = CS.thetaDDotApprox
		a = GAINS['aFilterThetaDDot']
	
		if not CS.thetaD:
			CS.thetaD=thetaD #Handle startup with zero thetaDDotApprox

		thetaDLast = CS.thetaD
		print "thetaDLast: " + str(thetaDLast)  + " " + str(thetaD)

	#Desired Heading Rate
		#thetaDDotApprox  = (1- a) * lastThetaDDotApprox +a/Ts *wrapToPi(thetaD-thetaDLast)  #Desired heading rate from numerical differentiation
		thetaDDotApprox = np.asscalar( 1.0/(1+ui[1]**2/ui[0]**2) * (ui[0]*pdiDot[1]-ui[1]*pdiDot[0])/ui[0]**2 )
		theteaDDotApprox = 0
		CS.thetaDDotApprox = thetaDDotApprox 
		CS.thetaD=thetaD

		print "qil Leader Body: " + str( Obi * qil)
#		print "qil Follower Body: " + str(np.matrix([[m.cos(theta),m.sin(theta)],[-m.sin(theta),m.cos(theta)]])*qil)
		print "qil Intertial: " + str(qil)

		etheta = wrapToPi(theta-thetaD)
		CS.etheta=etheta

		print "etheta: " + str(etheta)		

		eq = Obi*qil-qdil #this is in leader body, only want the first 2 elements
		eq.shape=(2,1)

	#heading control
		calcTurnRate = THIS.heading.rate 
		groundspd = np.linalg.norm(pi,2)
		accHeadingError= CS.accHeadingError

		CS.rollPTerm=	-ktheta.kp*etheta
		CS.rollITerm=	-ktheta.ki * CS.accHeadingError
		CS.rollDTerm =  -ktheta.kd * (calcTurnRate-thetaDDotApprox)
		CS.rollFFTerm = GAINS['kThetaFF']*m.atan(thetaDDotApprox * groundspd / 9.81 * m.cos(THIS.attitude.pitch))

#		print 'RollPTerm: ' + str(CS.rollPTerm)
#		print 'RollITerm: ' + str(CS.rollITerm)
#		print 'RollDTerm: ' + str(CS.rollDTerm)
#		print "FFTerm " + str(CS.rollFFTerm)

		print "Etheta: " + str(CS.accHeadingError)
		print "thetaDDot: " + str(CS.thetaDDotApprox)
		
		rollCMD =CS.rollPTerm + CS.rollITerm  + CS.rollDTerm +CS.rollFFTerm

#		rollCMD = 0;

		rollLimit=GAINS['rollLimit']
	
		rollCMD = saturate(rollCMD,-rollLimit,rollLimit)
		thisCommand.rollCMD =rollCMD

		print 'RollTarget:' + str(thisCommand.rollCMD)

		accHeadingError = antiWindup(rollCMD,-rollLimit,rollLimit,accHeadingError,etheta*Ts)
		accHeadingError = saturate(accHeadingError,-GAINS['maxETheta'],GAINS['maxETheta'])		
		CS.accHeadingError=accHeadingError

	#speed control
		#speedD = np.linalg.norm(ui,2) * m.cos(theta-thetaD) #reduce commanded velocity based on heading error
		speedD = np.linalg.norm(ui,2)  #Don't reduce commanded velocity based on heading error
		CS.speedD = speedD 
		CS.speedDDot = (ui.transpose() / speedD) * ( pdiDot)

		

#for step response: 
		"""speedAccelStep = 3
		if (datetime.now() - self.startTime).total_seconds() < 20:
			speedD = 18
			speedDDot = 0;
		elif (datetime.now() - self.startTime).total_seconds() < 40:  #Ramp down and hold till 40 sec
			speedD = 18 + speedAccelStep * ((datetime.now() - self.startTime).total_seconds() - 20)
			speedDDot = speedAccelStep
			if(speedD>24):
				speedDDot = 0
				speedD = 24
		else:  #Ramp down and hold low
			speedD = 24 - speedAccelStep * ((datetime.now() - self.startTime).total_seconds() - 40)
			speedDDot = -speedAccelStep
			if(speedD<=18):
				speedD = 18
				speedDDot = 0"""
		
		
#For sinusoidal speed command
		"""t = (datetime.now() - self.startTime).total_seconds()
		amp = 2gitk
		f = 0.1
		speedD = 20 + amp*m.sin(2*m.pi*f*t)
		speedDDot = amp * 2*m.pi*f*m.cos(2*m.pi*f*t)

		CS.speedDDot = speedDDot;
		CS.speedD = speedD"""

		print 'SpeedDDot: ' + str(CS.speedDDot)
		groundspd = THIS.groundspeed
		airspd = THIS.airspeed
		print 'groundspeed: '+str(groundspd)
		print 'airspeed: ' + str(airspd)
		eSpeed = groundspd - speedD
		
		sej = np.matrix([[0],[0]]); #TODO

		fi = np.matrix([[m.cos(THIS.heading)],[m.sin(THIS.heading)]])
		#fi = np.matrix([[m.cos(thetaD)],[m.sin(thetaD)]])
		
		#asTarget = speedD + (airspd-groundspd) #the basic one

		CS.backstepSpeed = speedD
		CS.backstepSpeedError =  1/GAINS['aSpeed']* -GAINS['gamma'] * eSpeed
		CS.backstepSpeedRate = 1/GAINS['aSpeed'] * CS.speedDDot
		CS.backstepPosError =  1/GAINS['aSpeed'] * -eqil.transpose()*fi*1/GAINS['lambda']
		
		asTarget = CS.backstepSpeed + CS.backstepSpeedRate + CS.backstepSpeedRate + CS.backstepPosError

		print "asTarget " + str(asTarget)
		
		print 'eSpeed ' + str(eSpeed)
		
		asTarget=max(vMin,min(vMax,asTarget)) #saturate to limit
		
		CS.asTarget = asTarget

		kspeed = GAINS['kSpeed']
		rollAngle = THIS.attitude.roll
		eSpeed = airspd - asTarget

		accAirspeedError=CS.accAirspeedError

		CS.throttlePTerm = -kspeed.kp * eSpeed
		CS.throttleITerm = -kspeed.ki * accAirspeedError
		CS.throttleDTerm = -kspeed.kd * THIS.fwdAccel
		CS.throttleFFTerm = self.trimThrottle + 1/m.pow(m.cos(rollAngle),2) #TODO: use FF gain

		print "\n\n\n"
	#	print "Speed P: "+str(CS.throttlePTerm)
	#	print "Speed I: "+str(CS.throttleITerm)
	#	print "Speed D: "+str(CS.throttleDTerm)
	#	print "Speed FF: "+str(CS.throttleFFTerm)

		thisCommand.throttleCMD = CS.throttlePTerm + CS.throttleITerm +CS.throttleDTerm+CS.throttleFFTerm

		accAirspeedError = antiWindup(thisCommand.throttleCMD, 0,100,accAirspeedError, eSpeed*Ts)
		accAirspeedError = saturate(accAirspeedError,-GAINS['maxESpeed'],GAINS['maxESpeed'])		
		CS.accAirspeedError=accAirspeedError
		print "ESpeed: " + str(accAirspeedError)

	#altitude control
		desiredAltitude = qd[ID-2,2] #this is AGL  for now
		altitude = THIS.position.alt
		kalt = self.parameters.gains['kalt']
		
		altError = altitude-desiredAltitude
		accAltError = CS.accAltError
		pitchLimit=self.parameters.gains['pitchLimit']
		print "AltSpeed: " + str(THIS.velocity[2])

		CS.pitchPTerm = -kalt.kp* altError
		CS.pitchITerm = - kalt.ki*accAltError
		CS.pitchDTerm = - kalt.kd * THIS.velocity[2] #indexed from 0 (x)
		thisCommand.pitchCMD = CS.pitchPTerm + CS.pitchITerm + CS.pitchDTerm

		accAltError = antiWindup(thisCommand.pitchCMD,-pitchLimit,pitchLimit, accAltError,altError*Ts)
		accAltError = saturate(accAltError,-GAINS['maxEAlt'],GAINS['maxEAlt']) 
		CS.accAltError  = accAltError
		thisCommand.pitchCMD=saturate(thisCommand.pitchCMD,-pitchLimit,pitchLimit)
		print "ErrorNorm: " + str(np.linalg.norm(eqil,2))
		#print "RelTime: " + str((datetime.now() - self.startTime).total_seconds())
		print '\n\n\n'
		
		thisCommand.timestamp = datetime.now()
		THIS.command = thisCommand
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

def antiWindup(value, lowLimit,highLimit, accumulator, toAdd):
	if(value>=highLimit): #Saturation and anti-windup
		if(toAdd>0):
			accumulator =accumulator+toAdd
	elif(value<=lowLimit):
		if(toAdd < 0):
			accumulator =accumulator+toAdd		
	else:
		accumulator =accumulator+toAdd
	return accumulator

def antiWindupVec(value, lowLimit,highLimit, accumulator, toAdd):
	for i in range(0,len(value)):
		if(value[i]>=highLimit): #Saturation and anti-windup
			if(toAdd[i]>0):
				accumulator[i] =accumulator[i]+toAdd[i]
		if(value[i]<=lowLimit):
			if(toAdd[i] < 0):
				accumulator[i] =accumulator[i]+toAdd[i]			
		else:
			accumulator[i] =accumulator[i]+toAdd[i]
	return accumulator

def propagateVehicleState(state, dt): #assumes heading rate and fwdAccel are constant
	psiDot = saturate(state.heading.rate,-2,2) 
	sDot = state.fwdAccel #TODO sometimes get math domain error on this
	psi = state.heading
#	print "propagating vehicle state"
	
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
	
#	print "T1: " + str(state.time)	
	state.time = state.time + timedelta(seconds = dt)
#	print "T2: " + str(state.time)		
	state.propagated = 1
	
