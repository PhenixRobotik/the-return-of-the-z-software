
#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

#parameters
goal=-10
v=-2
T1=0.5#do not set to 0 or a division by 0 will occur
T2=0.5

#internal variables
v=abs(v)
goal=abs(goal)

t3=goal/(v*(1-0.5*T1-0.5*T2))
t1=T1*t3
t2=t3*(1-T2)

a1=v/t1
a2=v/(t3-t2)

x1=0.5*a1*(t1**2)
x2=v*(t2-t1)+x1

print((t1,t2,t3))

poses=np.linspace(0,goal,100)
dt=np.zeros(100)
dx_step = goal/len(poses)


for i,x in enumerate(poses):
    if x<x1:
        dt[i] = dx_step / np.sqrt(2*(x+dx_step)*a1)
    elif x<x2:
        dt[i] = dx_step / v
    else:
        tmp = 2*a2*(goal-x+dx_step)
        dt[i] = dx_step / np.sqrt(tmp)


plt.figure()
plt.plot(dt)

print(poses)
print(dt)

print(t3)
print(np.sum(dt))

plt.show()
