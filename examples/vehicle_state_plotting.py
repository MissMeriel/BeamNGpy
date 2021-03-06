
import time

import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
#import seaborn as sns

from beamngpy import BeamNGpy, Vehicle, Scenario
from beamngpy.sensors import Electrics

#sns.set()  # Make seaborn set matplotlib styling

# Instantiate a BeamNGpy instance the other classes use for reference & communication
beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')  # This is the host & port used to communicate over

# Create a vehile instance that will be called 'ego' in the simulation
# using the etk800 model the simulator ships with
vehicle = Vehicle('ego', model='etk800', licence='PYTHON', colour='Green')
# Create an Electrics sensor and attach it to the vehicle
electrics = Electrics()
vehicle.attach_sensor('electrics', electrics)

# Create a scenario called vehicle_state taking place in the west_coast_usa map the simulator ships with
scenario = Scenario('west_coast_usa', 'vehicle_state')
# Add the vehicle and specify that it should start at a certain position and orientation.
# The position & orientation values were obtained by opening the level in the simulator,
# hitting F11 to open the editor and look for a spot to spawn and simply noting down the
# corresponding values.
scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=(0, 0, 45))  # 45 degree rotation around the z-axis

# The make function of a scneario is used to compile the scenario and produce a scenario file the simulator can load
scenario.make(beamng)


bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()  # After loading, the simulator waits for further input to actually start

vehicle.ai_set_mode('span')

positions = list()
directions = list()
wheel_speeds = list()
throttles = list()
brakes = list()


vehicle.update_vehicle()
sensors = bng.poll_sensors(vehicle)

print('The vehicle position is:')
display(vehicle.state['pos'])

print('The vehicle direction is:')
display(vehicle.state['dir'])

print('The wheel speed is:')
display(sensors['electrics']['wheelspeed'])

print('The throttle intensity is:')
display(sensors['electrics']['throttle'])

print('The brake intensity is:')
display(sensors['electrics']['brake'])

for _ in range(240):
    time.sleep(0.1)
    vehicle.update_vehicle()  # Synchs the vehicle's "state" variable with the simulator
    sensors = bng.poll_sensors(vehicle)  # Polls the data of all sensors attached to the vehicle
    positions.append(vehicle.state['pos'])
    directions.append(vehicle.state['dir'])
    wheel_speeds.append(sensors['electrics']['wheelspeed'])
    throttles.append(sensors['electrics']['throttle'])
    brakes.append(sensors['electrics']['brake'])

bng.close()

print(sensors.keys())
for k in sensors.keys():
    print("{} : {}".format(k, sensors[k]))

plt.plot(throttles, 'b-', label='Throttle')
plt.plot(brakes, 'r-', label='Brake')
plt.legend()
plt.show()
plt.clf()

x = [p[0] for p in positions]
y = [p[1] for p in positions]
plt.plot(x, y, '.')
plt.axis('square')
plt.title("Vehicle position")
plt.show()
plt.clf()


angles = [np.arctan2(d[1], d[0]) for d in directions]
r = wheel_speeds  # We simply use the speed as the radius in the radial plot
plt.subplot(111, projection='polar')
plt.scatter(angles, r)
plt.title("Wheel speed")
plt.show()