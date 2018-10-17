import sys
from vehicleState import *
from datetime import datetime
from collections import OrderedDict

attitudeKeys = ['roll','pitch','yaw','rollspeed','pitchspeed','yawspeed']
positionKeys = ['lat','lon','alt','time']
velocityKeys = [0,1,2]
courseAngleKeys = ['value','rate','accel']

def vsToLogPrep(vs):
#	print vs.ID
#	print vs.command

	headers=[]
	values=[]
	
	out=''
	
	headers.append('ID')
	values.append(vs.ID)
	headers.append('Counter')
	values.append(vs.counter)
	
	for k in attitudeKeys:
		headers.append(k)
		values.append(vs.attitude.__dict__[k])
	
	for k in positionKeys:
		headers.append(k)
		values.append(vs.position.__dict__[k])
	
	headers+=['latSpd','lonSpd','altSpd']
	values+=vs.velocity
	
	headers.append('airspeed')
	values.append(vs.airspeed)
	headers.append('groundspeed')
	values.append(vs.groundspeed)

	headers +=['wind_vx','wind_vy','wind_vz']
	values += [vs.wind_estimate['vx'],vs.wind_estimate['vy'],vs.wind_estimate['vz']]
	
	headers+=['heading','headingRate','headingAccel']
	values+=[vs.heading.value,vs.heading.rate,vs.heading.accel]

	headers+=['pitch','pitchRate','pitchAccel']
	values += [vs.pitch.value,vs.pitch.rate,vs.pitch.accel]
	
	
	d = vs.controlState._asdict()
	for k in d.keys():
#		print k
		item = d[k]
		if isinstance(item,np.matrix):
			(h,v)=handleMatrix(item,k)
			headers+=h
			values+=v
		elif isinstance(item,dict):
			for k2 in item.keys():
				if isinstance(item[k2],np.matrix):
					(h,v) = handleMatrix(item[k2],str(k)+str(k2))
					headers+=h
					values+=v
				else:
					headers.append(k2)
					values.append(item[k2])
		else:
			headers.append(k)
			values.append(item)


	d = vs.command._asdict()
	for k in d.keys():
		item = d[k]
		if isinstance(item,np.matrix):
			(h,v)=handleMatrix(item,k)
			headers+=h
			values+=v
		elif isinstance(item,dict):
			for k2 in item.keys():
				if isinstance(item[k2],np.matrix):
					(h,v) = handleMatrix(item[k2],str(k)+str(k2))
					headers+=h
					values+=v
				else:
					headers.append(k2)
					values.append(item[k2])
		else:
			headers.append(k)
			values.append(item)

	headers.append('GCSLastRX')		
	
	try:
		values.append(str((vs.timeout.GCSLastRx-epoch).total_seconds()))
	except: 
		values.append(' ')

	headers.append('GCSLastRX_1')
	try:
		values.append(str((vs.timeout.peerLastRX[1]-epoch) . total_seconds()))
	except: 
		values.append(' ')

	headers.append('GCSLastRX_2')
	try:
		values.append(str((vs.timeout.peerLastRX[2]-epoch) . total_seconds()))
	except: 
		values.append(' ')

	headers.append('GCSLastRX_3')
	try:
		values.append(str((vs.timeout.peerLastRX[3]-epoch) . total_seconds()))
	except: 
		values.append(' ')

	headers.append('abortReason')
	try:
		values.append(str(vs.abortReason))
	except:
		values.append(' ')
	
	for i in range(1,8):
		headers.append("ch"+str(i))
		values.append(vs.channels[str(i)])

	for i in range(1,4):
		headers.append("servoOut"+str(i))
		values.append(vs.servoOut[str(i)])
	
	out = OrderedDict(zip(headers,values))
	return out

def handleMatrix(mat,basename):
	outKey = []
	outValue = []		
	for j in range(0,len(mat)):
		outKey.append(basename+'_'+str(j))
		outValue.append(mat[j,0])
	return (outKey,outValue)

			
	

