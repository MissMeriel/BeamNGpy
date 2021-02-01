"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random
import sys

from time import sleep

import numpy as np

#from matplotlib import pyplot as plt
#from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, Config
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer
#from beamngpy import get_default
import beamngpy

#def load_config():
    # load sensor values
    # load positional values
#    return

def setup_sensors(vehicle):
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
    fov = 120
    resolution = (512, 512)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    pos = (0.0, 3, 1.0)
    direction = (0, -1, 0)
    fov = 90
    resolution = (512, 512)
    back_camera = Camera(pos, direction, fov, resolution,
                         colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    damage.encode_vehicle_request()
    lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle


def main():
    random.seed(1703)
    setup_logging()
    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')

    state_cfg = Config()
    parts_cfg = Config()
    # Create a scenario in west_coast_usa

    scenario = Scenario('west_coast_usa', 'research_test',
                        description='Random driving for research')

    # Set up first vehicle, with two cameras, gforces sensor, lidar, electrical
    # sensors, and damage sensors
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='RED', color='Red')
    vehicle = setup_sensors(vehicle)

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))

    # Compile the scenario and place it in BeamNG's map folder
    #scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)
    try:
        #bng.hide_hud()
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        # Load and start the scenario
        bng.load_scenario(scenario)
        bng.start_scenario()
        # Put simulator in pause awaiting further inputs
        bng.pause()
        pcfg = vehicle.get_part_config()
        assert vehicle.skt

        # Send random inputs to vehicle and advance the simulation 20 steps
        #for _ in range(1024):
        for _ in range(30):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            # bng.step(20)
            bng.step(20)

            # Retrieve sensor data and show the camera data.
            sensors = bng.poll_sensors(vehicle)

            print('\n{} seconds passed.'.format(sensors['timer']['time']))
            print("step in loop {}".format(_))

    finally:
        sensors = bng.poll_sensors(vehicle)
        for s in sensors.keys():
            print("{} : {}".format(s, sensors[s]))
        damage_dict = sensors['damage']
        state_cfg.update(sensors['damage'])
        state_cfg.update(vehicle.state)
        state_cfg.save('{}/state_cfg.json'.format(beamng.home))
        vehicle.annotate_parts()
        vehicle.update_vehicle()
        #part_options = vehicle.get_part_options()
        parts_cfg_dict = vehicle.get_part_config()


        parts_cfg.load_values(vehicle.get_part_config())
        #parts_cfg.update(vehicle.get_part_config())
        parts_cfg.save('{}/parts_cfg.json'.format(beamng.home))
        vehicle.save()
        bng.close()

    # reload scenario with saved config
    random.seed(1703)
    setup_logging()
    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    loaded_config = state_cfg.load("{}/state_cfg.json".format(beamng.home))
    scenario = Scenario('west_coast_usa', 'research_test',
                        description='Random driving for research')
    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='RED', color='Red')
    vehicle = setup_sensors(vehicle)
    vehicle.set_part_config(parts_cfg)

    scenario.add_vehicle(vehicle, pos=tuple(state_cfg.pos), rot=None,
                         rot_quat=beamngpy.angle_to_quat(state_cfg.dir))
    scenario.make(beamng)
    bng = beamng.open(launch=True)
    bng.spawn_vehicle(vehicle, pos=tuple(state_cfg.pos), rot=None,
                         rot_quat=beamngpy.angle_to_quat(state_cfg.dir))
    # TODO: debug scenario restart
    # scenario.restart()

    try:
        bng.hide_hud()
        bng.set_deterministic()  # Set simulator to be deterministic
        bng.set_steps_per_second(60)  # With 60hz temporal resolution

        # Load and start the scenario
        bng.load_scenario(scenario)
        bng.start_scenario()
        # Put simulator in pause awaiting further inputs
        bng.pause()

        assert vehicle.skt
        vehicle.load()
        # Send random inputs to vehicle and advance the simulation 20 steps
        #for _ in range(1024):
        for _ in range(30):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            # bng.step(20)
            bng.step(20)

            # Retrieve sensor data and show the camera data.
            sensors = bng.poll_sensors(vehicle)

            print('\n{} seconds passed.'.format(sensors['timer']['time']))
            print("step in loop {}".format(_))

    finally:
        sensors = bng.poll_sensors(vehicle)
        for s in sensors.keys():
            print("{} : {}".format(s, sensors[s]))
        state_cfg.update(sensors['damage'])
        state_cfg.update(vehicle.state)
        state_cfg.save('{}/state_cfg.json'.format(beamng.home))
        parts_cfg.update(vehicle.get_part_config())
        parts_cfg.save('{}/parts_cfg.json'.format(beamng.home))
        bng.close()

if __name__ == '__main__':
    main()