from recordtype import recordtype
import numpy as np

zeroVect = np.matrix([[0],[0]])


KPID = recordtype('KPID', ['kp','ki','kd'])

PIDTerms = recordtype('PIDTerms',['p','i','d','ff'],default = 0.0)

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter',['receivedTime','desiredPosition','gains', 'Ts', 'GCSTimeout', 'peerTimeout', 'leaderID', 'expectedMAVs', 'rollGain', 		'rollOffset', 'pitchGain', 'pitchOffset', 'throttleGain', 'throttleMin',('txStateType','basic')], default = None)

Command = recordtype('Command',['SpeedD','asTarget','thetaD','thetaDDot','rollCMD','pitchCMD','throttleCMD','timestamp'], default = 0.0)#,use_slots=False)

CourseAngle = recordtype('CourseAngle',['heading','headingRate','headingAccel','pitch','pitchRate','pitchAccel'],default=0.0)	

BasicVehicleState = recordtype('BasicVehicleState',['ID',('timestamp',None),'position','velocity','fwdAccel',('courseAngle',CourseAngle()),
	('propagated',0),('counter',0)], default=0.0)

ControlState = recordtype('ControlState',[('plTerm',zeroVect),('kplTerm',zeroVect),('kpjTerm',{}),'qldd'
	,('phiDotTerm',zeroVect),('kilTerm',zeroVect), ('uiTarget',zeroVect),'etheta','accHeadingError',('rollTerms',PIDTerms()),'accAirspeedError'
	,('throttleTerms',PIDTerms()),'accAltError',('pitchTerms',PIDTerms())
	,'backstepSpeed','backstepSpeedError','backstepSpeedRate','backstepPosError']
	, default = 0.0)

FullVehicleState = recordtype('FullVehicleState', [ ('startTime',None),['bvs',BasicVehicleState()], 'attitude','imuAccel'
	, 'channels', 'mode', ('command',Command()), ('controlState',ControlState()), ('isFlocking',False), ('RCLatch', True)
	, ('abortReason',{}), ('timeout', Timeout()), ('parameters',Parameter()),('servoOut',{'1':None,'2':None,'3':None})
	, ('airspeed',0.0),('groundspeed',0.0) ,('wind_estimate',{'vx':None,'vy':None,'vz':None})
	], default = None )
		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.

		
		
		


