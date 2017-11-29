from recordtype import recordtype
import numpy as np

zeroVect = np.matrix([[0],[0]])

KPID = recordtype('KPID', ['kp','ki','kd'])

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter','receivedTime,isComplete,desiredPosition,ctrlGains, Ts GCSTimeout peerTimeout, leaderID, expectedMAVs, cruiseSpeed, rollGain, rollOffset, pitchGain, pitchOffset, throttleGain, throttleMin', default = None)

Command = recordtype('Command',['rollCMD','pitchCMD','throttleCMD','timestamp'], default = None)#,use_slots=False)	

ControlState = recordtype('ControlState',[('accPosError',{1:zeroVect,2:zeroVect,3:zeroVect}),('plTerm',zeroVect),('kplTerm',zeroVect)
	,('phiDotTerm',zeroVect),('kilTerm',zeroVect), ('kdlTerm',zeroVect),('uiTarget',zeroVect),('thetaD',None),'thetaDDotApprox'
	,'etheta','speedD','asTarget','accHeadingError','rollPTerm','rollITerm','rollDTerm','rollFFTerm','accAirspeedError'
	,'throttlePTerm','throttleITerm','throttleDTerm','throttleFFTerm','accAltError','pitchPTerm','pitchITerm','pitchDTerm'
	, 'e1','u1'],default = 0.0)

VehicleState = recordtype('VehicleState', [ ('startTime',None),('isArmable' , False) ,'ID', 'time',  'attitude','acceleration'
	, 'channels', 'position', 'velocity',('heading',0.0),('headingRate',0.0), 'mode', ('command',Command())
	,('controlState',ControlState()), ('isFlocking',False), ('RCLatch', True), ('abortReason',None), ('timeout', Timeout())
	,('parameters',Parameter()),('servoOut',{'1':None,'2':None,'3':None}),('airspeed',0.0)
	,('wind_estimate',{'vx':None,'vy':None,'vz':None}),('fwdAccel',0)], default = None )
		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.


