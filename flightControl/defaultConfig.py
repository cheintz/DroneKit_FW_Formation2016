from vehicleState import Parameter, KPID
import time
import numpy as np
import math as m

defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[-10,0,120]])#,[-20,0,140]])  #Agent, amount forward, amount right, absolute altitude, meters
defaultParams.ctrlGains = {'kl':KPID(.35,0, 0.1), 'ka': KPID(0.1,0*.002,0.01),'alpha1': 0.001,'alpha2':100,'d':0.01
	,'vMin': 10,'vMax':30,'kBackstep':0,'aFilterHdg':0.1,'aFilterSpd':.02, 'aFilterThetaDDot': 0.1,'ktheta':KPID(0.11,0*0.005,0*1)
	,'kSpeed':KPID(6.7,1.6,4),'rollLimit':65/(180/m.pi),'kalt':KPID(0.05,.001,.005),'pitchLimit':30/(180/m.pi)
	,'maxEqil':100, 'maxETheta':10,'maxEAlt':50,'kThetaFF':1}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1.5 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leader
defaultParams.rollGain= 500/(65/(180/m.pi)) #degrees per PWM
defaultParams.rollOffset=1500
defaultParams.pitchGain = -500/(20/(180/m.pi)) #m/s per PWM
defaultParams.pitchOffset = 1500
defaultParams.throttleMin = 1000
defaultParams.throttleGain = (2000-1000)/100
defaultParams.Ts = 0.05

