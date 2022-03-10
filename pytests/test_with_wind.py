#!/usr/bin/env python3

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

class WindModel:
    def get_wind(self,position):
        return [0,0,0]
    def step(self,dt):
        pass

vehicle = MXS(mass,inertia,position,velocity,attitude,rates,WindModel())

deltaT = 0.01
time = 0
time_limit = 5#*2000
while time < time_limit:
    print(vehicle.statevector)
    vehicle.step(deltaT,[0,0,0,0])
    time += deltaT

print(f"Steps: {time_limit/deltaT}")
print(f"SimTime: {time}")
print(f"Final state: {vehicle.statevector}")


vehicle.statevector = [0.0,0.0,0.0, 11.0,0.0,0.0, 0.0,0.0,0.0,1.0, 0.0,0.0,0.0]

deltaT = 0.01
time = 0
time_limit = 5#*2000
while time < time_limit:
    print(vehicle.statevector)
    vehicle.step(deltaT,[0,0,0,0])
    time += deltaT

print(f"Steps: {time_limit/deltaT}")
print(f"SimTime: {time}")
print(f"Final state: {vehicle.statevector}")
