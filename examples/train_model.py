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
training_dir = 'training_images_utah5'
default_model = 'etk800' #'pickup'
default_scenario = 'utah' #'hirochi_raceway' #'automation_test_track' #'west_coast_usa' #'cliff' # smallgrid
dt = 20
base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))

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
    elif scenario_locale == 'automation_test_track':
        # starting line
        # return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # 30m down track from starting line
        return {'pos': (530.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # handling circuit
        #return {'pos': (-294.031, 10.4074, 118.518), 'rot': None, 'rot_quat': (0, 0, 0.708103, 0.706109)}
        # rally track
        #return {'pos': (-374.835, 84.8178, 115.084), 'rot': None, 'rot_quat': (0, 0, 0.718422, 0.695607)}
        # highway
        #return {'pos': (-294.791, -255.693, 118.703), 'rot': None, 'rot_quat': (0, 0, -0.704635, 0.70957)}
        # default
        #return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
    elif scenario_locale == 'hirochi_raceway':
        # default
        #return {'pos': (-453.309, 373.546, 25.3623), 'rot': None, 'rot_quat': (0, 0, -0.277698, 0.960669)}
        # starting point
        return {'pos': (-408.48, 260.232, 25.4231), 'rot': None, 'rot_quat': (0, 0, -0.279907, 0.960027)}
    elif scenario_locale == 'utah':
        # west highway
        #return {'pos': (-922.158, -929.868, 135.534), 'rot': None, 'rot_quat': (0, 0, -0.820165, 0.572127)}
        # building site
        #return {'pos': (-910.372, 607.927, 265.059), 'rot': None, 'rot_quat': (0, 0, 0.913368, -0.407135)}
        # tourist area
        #return {'pos': (-528.44, 283.886, 298.365), 'rot': None, 'rot_quat': (0, 0, 0.77543, 0.631434)}
        # auto repair zone
        #return {'pos': (771.263, -149.268, 144.291), 'rot': None, 'rot_quat': (0, 0, -0.76648, 0.642268)}
        # campsite
        return {'pos': (566.186, -530.957, 135.291), 'rot': None, 'rot_quat': ( -0.0444918, 0.0124419, 0.269026, 0.962024)}
        # default (do not use for training)
        # return {'pos': ( 771.263, -149.268, 144.291), 'rot': None, 'rot_quat': (0, 0, -0.76648, 0.642268)}
        # parking lot (do not use for training)
        # return {'pos': (907.939, 773.502, 235.878), 'rot': None, 'rot_quat': (0, 0, -0.652498, 0.75779)}


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

def setup_dir(training_dir):
    d = "{}/{}".format(os.path.dirname(os.path.realpath(__file__)), training_dir)
    if not os.path.isdir(d):
        os.mkdir(d)
    return "{}/data.csv".format(d)

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

def main():
    global base_filename, training_dir, default_model

    f = setup_dir(training_dir)
    spawn_pt = spawn_point(default_scenario)
    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=default_model,
                      licence='RED', color='Red')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=spawn_pt['pos'], rot=None, rot_quat=spawn_pt['rot_quat'])

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
    vehicle.ai_set_aggression(0.1)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt

    # Send random inputs to vehice and advance the simulation 20 steps
    imagecount = 0
    wheelvel = [0.1, 0.1, 0.1]
    with open(f, 'w') as datafile:
        datafile.write('filename,timestamp,steering_input,throttle_input,brake_input,driveshaft,engine_load,fog_lights,fuel,'
                       'lowpressure,oil,oil_temperature,parkingbrake,rpm,water_temperature\n')
        #for _ in range(1024):
        for _ in range(32768):
            #throttle = 1.0 #random.uniform(0.0, 1.0)
            #steering = random.uniform(-1.0, 1.0)
            #brake = random.choice([0, 0, 0, 1])
            #vehicle.control(throttle=throttle)

            # collect images
            sensors = bng.poll_sensors(vehicle)
            image = sensors['front_cam']['colour'].convert('RGB')
            imagecount += 1
            filename = "{}{}.bmp".format(base_filename, imagecount)

            # collect ancillary data
            datafile.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(filename,
                                                str(round(sensors['timer']['time'], 2)),
                                                sensors['electrics']['steering_input'],
                                                sensors['electrics']['throttle_input'],
                                                sensors['electrics']['brake_input'],
                                                sensors['electrics']['driveshaft'],
                                                sensors['electrics']['engine_load'],
                                                sensors['electrics']['fog_lights'],
                                                sensors['electrics']['fuel'],
                                                sensors['electrics']['lowpressure'],
                                                sensors['electrics']['oil'],
                                                sensors['electrics']['oil_temperature'],
                                                sensors['electrics']['parkingbrake'],
                                                sensors['electrics']['rpm'],
                                                sensors['electrics']['water_temperature'],))

            # save the image
            image.save(filename)

            # step sim forward
            bng.step(20)
            print('{} seconds passed.'.format(str(round(sensors['timer']['time'], 2))))

            # check for stopping condition
            for i in range(len(wheelvel)-1):
                wheelvel[i] = wheelvel[i+1]
            wheelvel[2] = float(sensors['electrics']['wheelspeed'])
            print('wheelvel = {}'.format(sum(wheelvel) / 3.0 ))
            if sum(wheelvel) / 3.0 == 0.0:
                print("avg wheelspeed is zero; exiting...")
                bng.close()
                break




if __name__ == '__main__':
    main()
