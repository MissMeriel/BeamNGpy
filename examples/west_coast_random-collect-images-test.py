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
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from PIL import Image
import PIL
import cv2
import scipy.misc

# globals
default_model = 'pickup'
default_scenario = 'automation_test_track' #'west_coast_usa' #'cliff' # smallgrid
dt = 20
camera_pos = (-0.3, 1, 1.0)
camera_direction = (0, 1, -1.5)
base_filename = 'H:/camera_mount/{}_{}_'.format(default_model, default_scenario.replace("_", ""))


def spawn_point(scenario_locale):
    if scenario_locale is 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale is 'west_coast_usa':
        #return {'pos':(-717.121, 101, 118.675), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-717.121, 101, 118.675), 'rot': None, 'rot_quat': (0, 0, 0.918812, -0.394696)}
    #906, 118.78 rot:
    elif scenario_locale is 'smallgrid':
        return {'pos':(0.0, 0.0, 0.0), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}

def setup_sensors(vehicle):
    # Set up sensors
    global camera_pos, camera_direction
    fov = 120
    resolution = (512, 512)
    front_camera = Camera(camera_pos, camera_direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    pos = (0.0, 2.5, 1.0)
    direction = (0, -1, 0)
    fov = 90
    resolution = (512, 512)
    back_camera = Camera(pos, direction, fov, resolution,
                         colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    #lidar = Lidar(visualized=False)
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
    global base_filename
    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    scenario = Scenario('west_coast_usa', 'research_test')
    vehicle = Vehicle('ego_vehicle', model='etk800',
                      licence='RED', color='Red')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.918812, -0.394696))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)
    images = []

    bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(60)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.ai_set_mode('span')
    vehicle.ai_drive_in_lane(True)

    assert vehicle.skt

    # Send random inputs to vehice and advance the simulation 20 steps
    for _ in range(1024):
        throttle = 1.0 #random.uniform(0.0, 1.0)
        #steering = random.uniform(-1.0, 1.0)
        #brake = random.choice([0, 0, 0, 1])
        vehicle.control(throttle=throttle)

        # collect images
        image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
        images.append(image)
        filename = "{}{}.bmp".format(base_filename, len(images))

        # save the image
        plt.imshow(np.asarray(image))
        plt.pause(1)
        image.save(filename)
        bng.step(20)

        # Retrieve sensor data and show the camera data.
        sensors = bng.poll_sensors(vehicle)
        print('{} seconds passed.'.format(sensors['timer']['time']))
    bng.close()


if __name__ == '__main__':
    main()
