from dronekit import connect, VehicleMode
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

acceptableControlMode = VehicleMode("FBWB")

logging.basicConfig(level=logging.WARNING)

class Controller(threading.Thread):
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams):
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
		# print "Constructor \n\n"
		# print type(self.vehicleState)
		self.command = Command()
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		while(not self.stoprequest.is_set()):#not self.kill_received):
			# print "executing control"
			while(not self.receiveQueue.empty()):
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
					self.receiveQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
			self.getVehicleState() #Get update from the Pixhawk

			if(not self.vehicleState.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.vehicleState.isFlocking):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
#					print "Would write commands"
					self.computeControl() #writes the control values to self.vehicleState
#					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			time.sleep(0.05)
			
				#TODO: find a way to clear timeouts, if necessary
		self.stop()
		self.releaseControl()			
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
			
	def scaleAndWriteCommands(self):
		xPWM = self.vehicleState.command.headingRate * self.parameters.headingGain+self.parameters.headingOffset
		yPWM = self.vehicleState.command.climbRate*self.parameters.climbGain + self.parameters.climbOffset
		zPWM = self.vehicleState.command.airSpeed*self.parameters.speedGain + self.parameters.speedOffset


		
		xPWM = 1600+100*m.sin(time.time())
		yPWM = 1600+100*m.cos(time.time())
		zPWM = 1510 #This is throttle off



		xPWM = saturate(xPWM,1000,2000)
		yPWM = saturate(yPWM,1000,2000)
		zPWM = saturate(zPWM,1510,2000)
		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		
	def checkAbort(self): #only call if flocking!!
		# print "in checkAbort" + str(time.time())
		if(self.checkTimeouts()): #If we had a timeout
			"Abort - Timeout" + str(datetime.now())
			self.vehicleStateabortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
#			self.commenceRTL()
#			self.releaseControl()
			self.commands = {0,0,0}			
			return True
		print "Flight Mode: " + str(self.vehicle.mode)
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Abort - control mode" + str(datetime.now())
			self.vehicleState.RCLatch = True			
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			# print "About to RTL" + str(time.time())
#			self.commenceRTL()
			# print "returned from RTL function" + str(time.time())
			self.commands = {0,0,0}
			return True
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 1900):
			print "Abort - Geofence not enabled"
			self.vehicelState.RCLatch = True
			self.vehicelState.isFlocking = False
			self.vehicleState.abortReason = "Geofence"
#			self.commenceRTL()
			self.commands = {0,0,0}
			return True
		if (self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True			
			self.abortReason = "RC Disable"
			print "RC Disable" + str(time.time())
#			self.commenceRTL()
#			self.releaseControl()
			self.commands = {0,0,0}
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
		if(len(self.stateVehicles) != self.parameters.expectedMAVs-1):
			print "Won't engage; Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys())
			self.vehicleState.RCLatch = True
			return False	

		#Check RC enable
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Won't engage - control mode" 
			self.vehicleState.RCLatch = True			
			return False			
		if(self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 1900): #Geofence
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
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = self.vehicle.channels.items() #necessary to be able to serialize it
#		print	str(time.time())  +"\t" + str(self.vehicle.attitude.roll) + "\t" + str((self.vehicleState.timeout.peerTimeoutTime)) + "\t" + str((self.vehicleState.timeout.peerLastRX))
		self.vehicleState.position = self.vehicle.location.global_frame
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()
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
		msg.type = "UAV"
		msg.sendTime=time.time()
		msg.content = self.vehicleState
		self.loggingQueue.put(msg)
	def commenceRTL(self):
		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
#		self.vehicle.mode = VehicleMode("RTL")
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

	def computeControl(self):
		thisCommand  = Command
		thisCommand. headingRate =1
		thisCommand.climbRate =1
		thisCommand.airSpeed = 1
		thisCommand.timestamp = datetime.now()
		self.vehicleState.command = thisCommand
def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out
	
