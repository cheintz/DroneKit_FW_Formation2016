from control import evalTF
from collections import deque
from numpy import arange
import math as m
import matplotlib.pyplot as plt



numCoeffs = [    0 ,   4.5223 , -17.6471 ,  27.3281 , -21.2052 ,   8.5294,   -1.6469 ,   0.1194]
denCoeffs = [  1.0000 ,  -4.2738 ,   7.6179 ,  -7.3013 ,   4.0300 ,  -1.2651,   0.2063,   -0.0137]

numTerms = deque([0,0,0,0,0,0,0,0],8)
denTerms = deque([0,0,0,0,0,0,0,0],8)


t=arange(0,10,.075)


O = len(numCoeffs)

u = []
y = []

#u[:O] = [0]*(O)
#y[:O] = [0]*(O)
#print u
#print t
for i in range(0,len(t)):
#	print i
	#u.append( 0.2 * m.sin(2*m.pi*0.3*t[i]))
	u.append(1)
	y.append(0)
	#numTerms.appendleft(0.2 * m.sin(2*m.pi*0.3*t[i]))
	numTerms.appendleft(1)
	denTerms.appendleft(0)

	n = len(u)
#	print y[:-O:-1]
	y[i] = evalTF(numCoeffs,list(numTerms),denCoeffs,list(denTerms))
	denTerms.popleft()
	denTerms.appendleft(y[i])
print len(t)
print len(u)
print len(y)
plt.plot(t,u,'k',t,y,'b')
plt.show()








