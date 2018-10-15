import collections
from vehicleState import *
import socket
import Queue
import logging
import multiprocessing
import jsonpickle
import cPickle
import zlib
import signal

class Receiver(multiprocessing.Process):
	def __init__(self,receiveQueue,AdHocIP, port):
		multiprocessing.Process.__init__(self)
		self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		self.AdHocIP = AdHocIP
		self.port = port
		ip = ""
		self.s.bind((ip,self.port))
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.receiveQueue=receiveQueue
		self.stoprequest = multiprocessing.Event()
		self.s.settimeout(1)
	def stop(self):
		self.stoprequest.set()
		print "Stop flag set - Receive"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
#			print "Processing any received"
			try:
				self.receiveMessage()
			except Queue.Empty:
#				thread.sleep(0.001) #not necessary because receiveMessage() is blocking (with timeout)
				break #no more messages.
		print "Receive Stopped"
	def receiveMessage(self):
		try:
			mp = self.s.recvfrom(4096)
#			if(False):
			if(mp[1] == (self.AdHocIP,self.port)):
				print "received my own message"
				pass
			else:
				mp=mp[0]
				mp = zlib.decompress(mp)
#				print mp
				try:
				#	print mp + "\n\n\n"
			#		print type(mp)
#					msg = jsonpickle.decode(mp)
					msg=cPickle.loads(mp)
			#		print msg
			#		msg = { str(key):value for key,value in msg.items() }
			#		print "\n\n\n"
			#		print msg
#					print msg.keys()
#					print type(msg)
#					print msg.content
#					print "Received valid packet from" + str(msg.content.ID) + " With Roll: " + str(msg.content.attitude.roll)
					print "received message from: " + str(msg.content.ID)
					self.receiveQueue.put(msg)
					pass
				except ValueError:
					print "received invalid packet"
				if(self.receiveQueue.qsize()>5):
					print "Receive Queue Size" + str(self.receiveQueue.qsize())
			#	print "Received, did nothing"
		except socket.error, e:
			if not e.args[0] == 'timed out':
				raise e
			else:
				print "timeout"
		
		
		
