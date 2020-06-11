from dronekit import connect, VehicleMode
import time
import socket
import os
import Queue
import multiprocessing	
import transmit, control, receive, log
import vehicleState

import numpy as np
import argparse 
from datetime import datetime
from servovehicle import ServoVehicle
 
#get enviromental variables
AdHocIP = os.environ['ADHOCIP']
AdHocIP = "192.168.0.108" #override if the environmental variable is wrong
peerReadPort = int(os.environ['PORT'])
myAddr = (AdHocIP, peerReadPort)
logPath = os.environ["LOGPATH"]

try:
	sitlFlag = os.environ["SITL"]
	sitlFlag=True
except:
	sitlFlag = False


if(sitlFlag):
	broadcastIP= "10.0.2.255"
	import defaultConfig_SITL as defaultConfig
else:
	broadcastIP= os.environ["BROADCASTIP"]
	import defaultConfig


#use nothing for the hardware in the planes

#broadcastIP= "10.0.2.255" #for virual machine
broadcastIP= "192.168.0.255"  #this is for the physical machine


transmitAddress = (broadcastIP,peerReadPort)





#set up socket for UDP broadcast
#s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#s.bind(myAddr)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#create message queues
loggingQueue= multiprocessing.Queue()
transmitQueue = multiprocessing.Queue()
receiveQueue = multiprocessing.Queue()

startTime=datetime.now()



#Parse the connection arg and connect to the vehicle
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

defaultParams = defaultConfig.getParams()

# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True, rate=1/defaultParams.Ts, baud=1500000,vehicle_class=ServoVehicle)

vehicle.wait_ready('autopilot_version')

receiveThread = receive.Receiver(receiveQueue,AdHocIP,peerReadPort,defaultParams.config['ignoreSelfPackets'])
transmitThread = transmit.Transmitter(transmitQueue,AdHocIP,peerReadPort,transmitAddress)

fileSuffix =   '_v' + str(int(vehicle.parameters['SYSID_THISMAV']))
logThread = log.Logger(loggingQueue,logPath,defaultParams.expectedMAVs,startTime,fileSuffix)

controlThread = control.Controller(loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime,sitlFlag)

print "default params" + str(defaultParams)

threads = []
threads.append(controlThread)
threads.append(receiveThread)
threads.append(transmitThread)
threads.append(logThread)

receiveThread.start()
print "Started Receive"

transmitThread.start()
print"Started Transmit"

logThread.start()
print "Started Logging"

controlThread.start()
print "Started Control"

def hasLiveThreads(threads):
	return True in [t.is_alive() for t  in threads]


while hasLiveThreads(threads):
	try:
		[t.join(1) for t in threads
		if t is not None and t.is_alive()]
		
	except KeyboardInterrupt:
		print "killing threads"
		for t in threads:
			t.stop()

		# Shut down simulator if it was started.
		if sitl is not None:
		    sitl.stop()
	
print "exiting Main"

	
