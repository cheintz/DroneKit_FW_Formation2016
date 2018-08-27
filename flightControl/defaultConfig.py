
from vehicleState import Parameter, KPID
import time
import numpy as np
import math as m

defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[-10,10,80]])#,[-10,-10,120]])  #Agent, amount forward, amount right, absolute altitude, meters
defaultParams.ctrlGains = {'kl':KPID(.35,0.005*0, 0), 'ka': KPID(0.1,0*.002,0.01*0),'alpha1': 0.001,'alpha2':100,'d':0.01
	,'vMin': 13,'vMax':24,'kBackstep':0,'aFilterHdg':0.1,'aFilterSpd':.02, 'aFilterThetaDDot': .8,'ktheta':KPID(1,0.1,0.5)
	,'kSpeed':KPID(6.7,1.2,4*0),'rollLimit':50/(180/m.pi),'kalt':KPID(0.05,.001,.005),'pitchLimit':20/(180/m.pi)
	,'maxEqil':100, 'maxETheta':5,'maxEAlt':50,'maxESpeed':20,'kThetaFF':1, 'aSpeed':0.4,'Ks':2.5}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 4 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leaders
defaultParams.rollGain= -500/(50/(180/m.pi)) #degrees per PWM
defaultParams.rollOffset=1500
defaultParams.pitchGain = -500/(20/(180/m.pi)) #m/s per PWM
defaultParams.pitchOffset = 1500
defaultParams.throttleMin = 1000
defaultParams.throttleGain = (2000-1000)/100
defaultParams.Ts = 0.1

