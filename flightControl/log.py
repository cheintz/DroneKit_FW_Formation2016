import collections
from vehicleState import *
import socket
import Queue
import logging
import multiprocessing
import jsonpickle
import cPickle
import os
import time
import mutil
from datetime import datetime
import signal

class Logger(multiprocessing.Process):

	def __init__(self,logQueue,logPath,n,startTime):
		multiprocessing.Process.__init__(self)
		self.logQueue=logQueue
		self.stoprequest = multiprocessing.Event()
		self.expectedMAVs=n
#		self.file=open(datetime.now().strftime("%y_%m_%d_%H_%M_%S_log.csv"),'w')
		#self.file=open(os.path.join("/home/pi/logs" ,datetime.now().strftime("log_%Y_%m_%d__%H_%M_%S.csv")),'w')
		self.startTime=startTime
		self.file=open(os.path.join(logPath ,self.startTime.strftime("%Y_%m_%d__%H_%M_%S_log.csv")),'w')
		self.headerWritten = False
		self.lastLogged = 0
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Log"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
			while( not self.stoprequest.is_set()):
				if(self.logQueue.qsize()>5):
					print "Log Queue Size: " + str(self.logQueue.qsize())
				try:
					msg = self.logQueue.get(True, 0.5)
					self.logMessage(msg)
					#print "Sent a message"
				#	self.logQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					time.sleep(0.001)
#					print "Log Sleeping"
					break #no more messages.
		self.file.flush()
		os.fsync(self.file.fileno())
		self.file.close()
		print "Log Stopped"
					
	def logMessage(self, msg):
		stateVehicles = msg.content['stateVehicles']
		thisState = msg.content['thisState']
		self.lastState = thisState

		if(thisState.counter<= self.lastLogged):
			print "Error: Logged states out of order: Last Logged: " + str(self.lastLogged)+ ", This Counter: " + str(thisState.counter) 
			#print "last state: "  + str(self.lastState.time)
	#		#print "this state: " + str(thisState.time)
	#		raise ValueError('Attempt to log vehicleStates out of sequence!!') 
		#print "lastDiff" + str(thisState.counter - self.lastLogged)
#		print "\t\t" + str(thisState.counter) + "\t" + str(self.logQueue.qsize())	
		self.lastLogged = thisState.counter
		self.lastTransmittedState = thisState

		if not self.headerWritten: #only do this once
			myOrderedDict = mutil.vsToLogPrep(thisState)
			self.writeHeaderString(myOrderedDict.keys())
			self.numItemsPerMav = len(myOrderedDict.keys())
			self.headerWritten = True

		
		outString = ''

		outString += str(thisState.time) + ','
		outString+= str((thisState.time - thisState.startTime).total_seconds())+',' #relative time
		for i in range(1,thisState.parameters.expectedMAVs+1):
			#print "logging: " + str(i)
			try:
				if(True):#i!=thisState.ID):
					stateToWrite= (stateVehicles[i])
				else:
					stateToWrite =thisState
				
				myOrderedDict = mutil.vsToLogPrep(stateToWrite)
				outString += ','.join(map(str, myOrderedDict.values()))
			except KeyError:
#				print "Attempted to log nonexistant vehicle: " + str(i)
				outString += str(i)+','
				for j in range(0,self.numItemsPerMav-2): #write blanks to save the space
					outString += ', '
			if(i!=thisState.parameters.expectedMAVs):
				outString+=', '
		self.file.write(outString)
		self.file.write("\n")
		#print "Send complete"ader
	def writeHeaderString(self,sourceList):
		headerString=''
		headerString+='Time, RelTime,'
		
		n=self.expectedMAVs
		for i in range(1,n+1):
			prefix = "v"+str(i)+"_"
			tempList = [prefix + s for s in sourceList] #append the mavID to each header
	
			headerString+= ','.join(map(str, tempList)) 
			if(i!=n):
				headerString+=','
		headerString+='\n'		
		self.file.write(headerString)

		
		
		
		

