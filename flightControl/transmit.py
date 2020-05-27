import collections
from vehicleState import *
import socket
import logging
import multiprocessing
import Queue
import jsonpickle
import cPickle
import zlib
import time
import signal

class Transmitter(multiprocessing.Process):

	def __init__(self,transmitQueue,AdHocIP,port,sendAddress):
		multiprocessing.Process.__init__(self)
		self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		myAddr = (AdHocIP,port)
		self.s.bind(myAddr)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.transmitQueue=transmitQueue
		self.sendAddress = sendAddress
		self.stoprequest = multiprocessing.Event()
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Transmit"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
			if(self.transmitQueue.qsize()>1):
				print "TX Queue size: " + str(self.transmitQueue.qsize())
			try:
				msg = self.transmitQueue.get(True, 0.5)
				self.sendMessage(msg)
#					print "sending message"
			#	self.transmitQueue.task_done() #May or may not be helpful
			except Queue.Empty:
				time.sleep(0.001)
				break #no more messages.

		print "Transmit Stopped"
					
	def sendMessage(self, msg):
		#print msg
		mp = cPickle.dumps(msg,cPickle.HIGHEST_PROTOCOL)
#		mp = cPickle.dumps(msg,1)
#		print len(str(msg.content.__dict__))
#		print len(mp)
		mp = zlib.compress(mp)
		#print len(mp)

	#	print "packetLength: " + str(len(mp))
		print self.sendAddress
	#	print "Length zlib: "+str(len(zlib.compress(mp)))	
#		print "Encoded is" + mp
		self.s.sendto(mp,self.sendAddress);
		#print "Send complete"
		
		
		

