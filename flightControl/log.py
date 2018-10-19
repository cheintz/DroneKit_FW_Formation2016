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
from datetime import datetime
import signal
import copy

class Logger(multiprocessing.Process):

	def __init__(self,logQueue,logPath,n,startTime,fileSuffix):
		multiprocessing.Process.__init__(self)
		self.logQueue=logQueue
		self.stoprequest = multiprocessing.Event()
		self.expectedMAVs=n
#		self.file=open(datetime.now().strftime("%y_%m_%d_%H_%M_%S_log.csv"),'w')
		#self.file=open(os.path.join("/home/pi/logs" ,datetime.now().strftime("log_%Y_%m_%d__%H_%M_%S.csv")),'w')
		self.startTime=startTime
		self.file=open(os.path.join(logPath,  self.startTime.strftime("%Y_%m_%d__%H_%M_%S_log") + fileSuffix+ '.csv'),'w' ) 
		self.headerWritten = False
		self.lastLogged = 0
		self.numItemsPerSelf=0
		self.numItemsPerOther=0
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
		thisDict = thisState.getCSVLists()	

		if not self.headerWritten: #only do this once	
			#Assume same type of logging as this vehicle
			self.writeHeaderString(thisState)
			self.headerWritten = True

		outString = str(thisState.time) + ','
		outString+= str((thisState.time - thisState.startTime).total_seconds())+',' #relative time
		for i in range(1,thisState.parameters.expectedMAVs+1):
			try:
				if i!=thisState.ID: #if other UAV
					stateToWrite= (stateVehicles[i])
				else:
					stateToWrite =thisState
				
				myOrderedDict = stateToWrite.getCSVLists()
				outString += ','.join(map(str, myOrderedDict.values()))
			except KeyError:
				outString += str(i)+','
				for j in range(0,self.numItemsPerOther-2): #write blanks to save the space
					outString += ','
			if(i!=thisState.parameters.expectedMAVs):
				outString+=','
		self.file.write(outString)
		self.file.write("\n")
	def writeHeaderString(self,thisState):
		headerString=''
		headerString+='Time, RelTime,'
		n=self.expectedMAVs

		thisList =thisState.getCSVLists().keys()
		self.numItemsPerSelf = len(thisList)
		if(thisState.parameters.txStateType == 'basic'):
			temp = BasicVehicleState(copy.deepcopy(thisState))
			otherList = temp.getCSVLists()
			self.numItemsPerOther = len(otherList)
		else:
			otherList = thisList

		for i in range(1,n+1):
			prefix = "v"+str(i)+"_"
			if not thisState.ID == i: #other vehicles
				tempList = [prefix + s for s in otherList] #append the mavID to each header
				headerString+= ','.join(map(str, tempList)) 
			else: 
				tempList = [prefix + s for s in thisList] #append my mavID to each header
				headerString+= ','.join(map(str, tempList)) 
			if(i!=n):
				headerString+=',' #no trailing comma, but commas between MAVs
		headerString+='\n'		
		self.file.write(headerString)

		
		
		
		

