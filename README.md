# `pymxsrs`

`pymxsrs` is a Python wrapper around `mxsrs`, a model of the MXS V2 aircraft implemented in Rust.

## Usage

Some examples can be found in the `pytests` folder. There is only one real export from `pymxsrs`, the `MXS` class. The
constructor takes the mass, inertia, position, velocity, attitude and rates.

```py
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
```

A custom Python-driven wind model can also be specified. The wind model must be an object with two methods: `get_wind()`
and `step()`:
- `get_wind()` should is called with the world-frame position and returns the [N,E,D] components of the wind
- `step()` takes a timestep and can be used to update internal wind model state. Use `pass` if not needed.

```py
class WindModel:
    def get_wind(self,position):
        return [0,0,0]
    def step(self,dt):
        pass

vehicle = MXS(mass,inertia,position,velocity,attitude,rates,WindModel())
```

Step the model forward using the `step()` method. The state vector, and some other useful properties are exposed as
properties of the MXS object.

```py
deltaT = 0.01
time = 0
time_limit = 5*2000

control_input = [0,0,0.2,0]

while time < time_limit:
    print(vehicle.statevector)
    vehicle.step(deltaT,control_input)
```

# Python-style definition

```py
class MXS:
    def __init__(self,mass,inertia,position,velocity,attitude,rates):
        # Implemented in rust
        pass

    def step(self,delta_t,control_input):
        # Implemented in rust
        pass

    
    @property
    def position(self):
        # Implemented in rust
        pass
    
    @property
    def velocity(self):
        # Implemented in rust
        pass
    
    @property
    def position(self):
        # Implemented in rust
        pass
    
    @property
    def attitude(self):
        # Implemented in rust
        pass
    
    @property
    def rates(self):
        # Implemented in rust
        pass
    
    @property
    def statevector(self):
        # Implemented in rust
        pass
    
    @statevector.setter
    def statevector(self,value):
        # Implemented in rust
        pass 
```