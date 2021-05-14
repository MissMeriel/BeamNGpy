import mmap
import random
import sys
from time import sleep
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from PIL import Image
import PIL
import cv2
import scipy.misc
import copy
import statistics, math
import json
from scipy.spatial.transform import Rotation as R

# globals
home = 'H:/BeamNG.research.v1.7.0.1clean'
default_color = 'White'
default_scenario = 'industrial'
default_spawnpoint = 'racetrackleft'
spawn = {'pos': (216.578, -28.1725, 42.7788), 'rot': None, 'rot_quat': (-0.0051, -0.003147, -0.67135, 0.74112)}
setpoint = 60.0 #50.0 #53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country
lanewidth = 3.75
steps_per_sec = 100

def setup_sensors(vehicle):
    # Set up sensors
    # pos = (-0.3, 1, 1.0) # default
    pos = (-0.5, 2, 1.0) #center edge of hood
    # pos = (-0.5, 1, 1.0)  # center middle of hood
    # pos = (-0.5, 0.4, 1.0)  # dashboard
    # pos = (-0.5, 0.38, 1.5) # roof
    # pos = (-0.5, 0.38, 1.1) # rearview?
    direction = (0, 0, 0) #(0, 0.75, -1.5) #(0, 0.75, 0) #(0,1,0)
    fov = 120
    resolution = (512, 512)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    pos = (-0.5, 0.38, 1.1) # rearview
    direction = (0, 1, 0)
    fov = 120
    resolution = (512, 512)
    back_camera = Camera(pos, direction, fov, resolution,
                         colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    #lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('headlight_cam', front_camera)
    vehicle.attach_sensor('rearview_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle

def euler_from_quaternion(x, y, z, w):
    r = R.from_quat((x,y,z,w))
    return r.as_euler('xyz', degrees=True)

def run_scenario_spawns_correctly(vehicle_model='etk800'):
    global base_filename, default_color, spawn, steps_per_sec, home

    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home=home)
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('vehicle', model=vehicle_model,
                      licence='CORRECT', color='Red')
    vehicle = setup_sensors(vehicle)

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)
    bng = beamng.open(launch=True)

    bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(steps_per_sec)

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()

    return_str = "SPAWNED VEHICLE STATE:\n{}".format(vehicle.state)
    print(return_str)

    bng.close()
    return return_str

def run_scenario_spawns_backwards(vehicle_model='etk800'):
    global base_filename, default_color, spawn, steps_per_sec, home

    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home=home)
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('vehicle', model=vehicle_model,
                      licence='CORRECT', color='Red')
    vehicle = setup_sensors(vehicle)

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)
    bng = beamng.open(launch=True)

    bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(steps_per_sec)

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()
    bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])


    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()

    return_str = "SPAWNED VEHICLE STATE:\n{}".format(vehicle.state)
    print(return_str)

    bng.close()
    return return_str

def main():
    v = 'hopper'
    results_correct = run_scenario_spawns_correctly(vehicle_model=v)
    results_backwards = run_scenario_spawns_correctly(vehicle_model=v)
    print("\n\nCORRECTLY SPAWNING VEHICLE STATE:\n{}\nBACKWARDS SPAWNING VEHICLE STATE:\n{}".format(results_correct, results_backwards))


if __name__ == '__main__':
    main()
