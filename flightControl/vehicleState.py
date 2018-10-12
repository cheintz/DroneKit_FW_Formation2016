from recordtype import recordtype
import numpy as np

zeroVect = np.matrix([[0],[0]])


KPID = recordtype('KPID', ['kp','ki','kd'])

PIDTerms = recordtype('PIDTerms',['p','i','d','ff'],default = 0.0)

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter',['receivedTime','desiredPosition','gains', 'Ts', 'GCSTimeout', 'peerTimeout', 'leaderID', 'expectedMAVs', 'rollGain', 		'rollOffset', 'pitchGain', 'pitchOffset', 'throttleGain', 'throttleMin',('txStateType','basic')], default = None)

Command = recordtype('Command',['SpeedD','asTarget','thetaD','thetaDDot','rollCMD','pitchCMD','throttleCMD','timestamp'], default = 0.0)#,use_slots=False)

CourseAngle = recordtype('CourseAngle',['value','rate','accel'],default=0.0)	

ControlState = recordtype('ControlState',[('plTerm',zeroVect),('kplTerm',zeroVect),('kpjTerm',{}),'qldd'
	,('phiDotTerm',zeroVect),('kilTerm',zeroVect), ('uiTarget',zeroVect),'etheta','accHeadingError',('rollTerms',PIDTerms()),'accAirspeedError'
	,('throttleTerms',PIDTerms()),'accAltError',('pitchTerms',PIDTerms())
	,'backstepSpeed','backstepSpeedError','backstepSpeedRate','backstepPosError']
	, default = 0.0)

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.

class BasicVehicleState(object):
	def __init__(self):
		self.ID = None
		self.timestamp = None
		self.position = 0.0
		self.velocity = 0.0
		self.fwdAccel = 0.0
		self.heading = CourseAngle()
		self.pitch = CourseAngle()
		self.isPropagated = False
		self.counter = 0
		self.isFlocking = False
		
class FullVehicleState(BasicVehicleState):
	def __init__(self):
		super(FullVehicleState, self).__init__()
		self.startTime = None
		self.attitude = None
		self.imuAccel = None
		self.chanels = None
		self.mode = None
		self.command = Command()
		self.controlState = ControlState()
		self.RCLatch = True
		self.airspeed = 0.0
		self.groundspeed = 0.0
		self.windEstimate = {'vx':None,'vy':None,'vz':None}
		self.timeout= Timeout()
