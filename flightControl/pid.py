from  vehicleState import *
import copy

class PIDController:
	def __init__(self,gains, lowerOutSat, upperOutSat, lowerIntSat,upperIntSat):
		self.integrator = 0.0
		self.gains = copy.deepcopy(gains)
		self._lowerIntSat = lowerIntSat
		self._upperIntSat = upperIntSat
		self._lowerOutSat = lowerOutSat
		self._upperOutSat = upperOutSat
	def update(self,e,eDot,Ts,ff=0):
		terms = PIDTerms()
		terms.p = -e *self.gains.kp
		terms.i = -self.integrator * self.gains.ki
		terms.d = -eDot * self.gains.kd
		terms.ff = ff  
		output = terms.p+terms.d+terms.i+terms.ff
		output = saturate(output,self._lowerOutSat,self._upperOutSat)
		antiWindup(output,self._lowerOutSat,self._upperOutSat,self.integrator,e*Ts)
		self.integrator = saturate(self.integrator,self._lowerIntSat,self._upperIntSat)

		return [output, terms]
	def reset(self):
		self.terms=PIDTerms()
		self.integrator = 0.0

def antiWindup(value, lowLimit,highLimit, accumulator, toAdd):
	if(value>=highLimit): #Saturation and anti-windup
		if(toAdd>0):
			accumulator =accumulator+toAdd
	elif(value<=lowLimit):
		if(toAdd < 0):
			accumulator =accumulator+toAdd		
	else:
		accumulator =accumulator+toAdd
	return accumulator

def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out
