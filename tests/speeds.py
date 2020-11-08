
#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

#parameters
goal=10
v=-2
x1=0.3#do not set to 0 or a division by 0 will occur
x2=0.3

#internal variables
v=abs(v)
if goal<0:
    v*=-1

t3=goal/(v*(1-0.5*x1-0.5*x2))
t1=x1*t3
t2=t3*(1-x2)

a1=v/t1
a2=v/(t3-t2)

x1=0.5*a1*(t1**2)
x2=v*(t2-t1)+x1

print((t1,t2,t3))

time=np.linspace(0,t3,100)
x=np.zeros(100)
vt=np.zeros(100)
vx=np.zeros(100)

for i,t in enumerate(time):
    if t<t1:
        x[i]=0.5*a1*(t**2)
        vt[i] = a1 * t
        vx[i] = np.sqrt(2*x[i]*a1)
    elif t<t2:
        x[i]=v*(t-t1)+x1
        vt[i] = v
        vx[i] = v
    else:
        x[i]=-0.5*a2*(t-t2)**2+v*(t-t2)+x2
        vt[i] = -a2 * t + v + a2*t2
        tmp = 2*a2*(goal-x[i])
        if tmp < 0:
            vx[i] = 0
        else:
            vx[i] = np.sqrt(tmp)


plt.plot(time,x)
plt.plot(time,vt)

plt.figure()
plt.plot(x, vt)

plt.figure()
plt.plot(x, vx)

plt.show()
