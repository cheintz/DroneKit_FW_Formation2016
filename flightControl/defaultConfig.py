from vehicleState import Parameter
import time
import numpy as np


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[-10,0,120]])  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':0.25, 'ka': 0.0001,'alpha1': 0.001,'alpha2':100,'d':0.01,'vMin': 15,'vMax':24,'kBackstep':0,'aFilter':0.1, 'aFilterThetaDDot': 0.1,'ktheta':1.0, 'headingRateLimit':0.6,'kpAlt':0.15,'kiAlt':0.02,'climbLimit':2.0}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1.5 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leader
defaultParams.headingGain= 500/0.6 #rad/sec per PWM
defaultParams.headingOffset=1500
defaultParams.climbGain = -500/2.0 #m/s per PWM
defaultParams.climbOffset = 1500
defaultParams.speedGain = 500/9
defaultParams.speedOffset = 1677
defaultParams.cruiseSpeed = 17
defaultParams.Ts = 0.05

