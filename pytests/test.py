#!/usr/bin/env python3

from time import process_time_ns
from pymxsrs import MXS

mass = 1.0
inertia = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]]
position = [0.0,0.0,0.0]
velocity = [11.0,0.0,0.0]
attitude = [0.0,0.0,0.0,1.0]
rates = [0.0,0.0,0.0]

vehicle = MXS(mass,inertia,position,velocity,attitude,rates)

# deltaT = 0.01
# time = 0
# time_limit = 5#*2000
# while time < time_limit:
#     print(vehicle.statevector)
#     vehicle.step(deltaT,[0,0,0,0])
#     time += deltaT

# print(f"Steps: {time_limit/deltaT}")
# print(f"SimTime: {time}")
# print(f"Final state: {vehicle.statevector}")


samples = 30

times = [0]*samples

for i in range(30):
    start = process_time_ns()
    
    vehicle.statevector = [0.0,0.0,0.0, 11.0,0.0,0.0, 0.0,0.0,0.0,1.0, 0.0,0.0,0.0]

    deltaT = 0.01
    time = 0
    time_limit = 5*2000
    while time < time_limit:
        #print(vehicle.statevector)
        vehicle.step(deltaT,[0,0,0.2,0])
        time += deltaT
    
    end = process_time_ns()
    times[i] = end - start

print(f"Steps: {time_limit/deltaT}")
print(f"SimTime: {time}")
print(f"Final state: {vehicle.statevector}")

s_times = [0.0]*samples
for i,time in enumerate(times):
    s_times[i] = time/(1000**3)

print(f"Mean time: {sum(s_times) / samples}")
