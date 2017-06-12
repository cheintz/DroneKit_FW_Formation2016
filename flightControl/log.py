import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import jsonpickle
import cPickle
import os
import time
from datetime import datetime

class Logger(threading.Thread):

	def __init__(self,logQueue,logPath):
		threading.Thread.__init__(self)
		self.logQueue=logQueue
		self.stoprequest = threading.Event()
#		self.file=open(datetime.now().strftime("log_%y_%m_%d_%H_%M_%S.json"),'w')
		self.file=open(os.path.join(logPath ,datetime.now().strftime("log_%Y_%m_%d__%H_%M_%S.json")),'w')
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Log"
	def run(self):
		while( not self.stoprequest.is_set()):
			while( not self.logQueue.empty()):
				if(self.logQueue.qsize()>5):
					print "Log Queue Size: " + str(self.logQueue.qsize())
				try:
					msg = self.logQueue.get(True, 0.5)
					self.logMessage(msg)
					#print "Sent a message"
					self.logQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					thread.sleep(0.001)
					break #no more messages.
		self.file.flush()
		os.fsync(self.file.fileno())
		self.file.close()
		print "Log Stopped"
					
	def logMessage(self, msg):
#		print "About to transmit" + str(msg.content.attitude.roll)
		mp = jsonpickle.encode(msg)
#		mp = cPickle.dumps(msg)
#		print "Length: " + str(len(mp))		
#		print "Encoded is" + mp
		self.file.write(mp)
		self.file.write("\n")
		#print "Send complete"
		
		
		

