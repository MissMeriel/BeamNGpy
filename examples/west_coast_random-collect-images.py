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
#training_dir = 'training_images_hirochi_remove'
default_scenario = 'automation_test_track'
training_dir = 'BeamNG_DAVE2/training_images_{}5'.format(default_scenario)
default_model = 'etk800' #'pickup'
setpoint = 40

dt = 20
#base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))
base_filename = 'H:/{}/{}_{}_'.format(training_dir, default_model, default_scenario.replace("_", ""))

def spawn_point(scenario_locale):
    if scenario_locale == 'automation_test_track':
        # starting line
        #return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # 30m down track from starting line
        #return {'pos': (530.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # handling circuit
        #return {'pos': (-294.031, 10.4074, 118.518), 'rot': None, 'rot_quat': (0, 0, 0.708103, 0.706109)}
        # rally track
        #return {'pos': (-374.835, 84.8178, 115.084), 'rot': None, 'rot_quat': (0, 0, 0.718422, 0.695607)}
        # highway
        return {'pos': (-294.791, -255.693, 118.703), 'rot': None, 'rot_quat': (0, 0, -0.704635, 0.70957)}
        # default
        return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
    elif scenario_locale == 'hirochi_raceway':
        # figure 8 oval
        return {'pos': (181.513, 4.62607, 20.6226), 'rot': None, 'rot_quat': (0, 0, 0.432016, 0.901866)}
        # pit lane
        #return {'pos': (-457.309, 373.546, 25.3623), 'rot': None, 'rot_quat': (0, 0, -0.277698, 0.960669)}
        # paddock
        #return {'pos': (-256.046, 273.232, 25.1961), 'rot': None, 'rot_quat': (0, 0, 0.741246, 0.671234)}
        # starting line (done in traffic mode)
        #return {'pos': (-408.48, 260.232, 25.4231), 'rot': None, 'rot_quat': (0, 0, -0.279907, 0.960027)}
        # rock crawling course
        #return {'pos': (-179.674, -50.6751, 27.6237), 'rot': None, 'rot_quat': (0.0734581, 0.00305369, 0.0414223, 0.996433)}
        #return {'pos': (-183.674, -38.6751, 25.6237), 'rot': None, 'rot_quat': (0.0734581, 0.0305369, 0.0414223, 0.996433)}
        # default
        #return {'pos': (-453.309, 373.546, 25.3623), 'rot': None, 'rot_quat': (0, 0, -0.277698, 0.960669)}
    elif scenario_locale == 'utah':
        # west highway
        return {'pos': (-922.158, -929.868, 135.534), 'rot': None, 'rot_quat': (0, 0, -0.820165, 0.572127)}
        # west highway 2
        #COLLECTED UTAH10 return {'pos': (-151.542, -916.292, 134.561), 'rot': None, 'rot_quat': (0.017533652484417, 0.01487538497895, -0.68549990653992, 0.72770953178406)}
        # building site
        #return {'pos': (-910.372, 607.927, 265.059), 'rot': None, 'rot_quat': (0, 0, 0.913368, -0.407135)}
        # on road near building site
        #COLLECTED UTAH7 #return {'pos': (-881.524, 611.674, 264.266), 'rot': None, 'rot_quat': (0, 0, 0.913368, -0.407135)}
        # tourist area
        #return {'pos': (-528.44, 283.886, 298.365), 'rot': None, 'rot_quat': (0, 0, 0.77543, 0.631434)}
        # auto repair zone
        #return {'pos': (771.263, -149.268, 144.291), 'rot': None, 'rot_quat': (0, 0, -0.76648, 0.642268)}
        # campsite
        #return {'pos': (566.186, -530.957, 135.291), 'rot': None, 'rot_quat': ( -0.0444918, 0.0124419, 0.269026, 0.962024)}
        # default
        #return {'pos': ( 771.263, -149.268, 144.291), 'rot': None, 'rot_quat': (0, 0, -0.76648, 0.642268)} #(do not use for training)
        #COLLECTED UTAH8 return {'pos': (835.449, -164.877, 144.57), 'rot': None, 'rot_quat': (-0.003, -0.0048, -0.172, 0.985)}
        # parking lot (do not use for training)
        #return {'pos': (907.939, 773.502, 235.878), 'rot': None, 'rot_quat': (0, 0, -0.652498, 0.75779)} #(do not use for training)
        #COLLECTED UTAH9 return {'pos': (963.22,707.785,235.583), 'rot': None, 'rot_quat': (-0.027, 0.018, -0.038, 0.999)}
    elif scenario_locale == 'industrial':
        # western industrial area -- didnt work with AI Driver
        return {'pos': (237.131, -379.919, 34.5561), 'rot': None, 'rot_quat': (-0.035, -0.0181, 0.949, 0.314)}
        # open industrial area -- didnt work with AI Driver
        #return {'pos':, 'rot': None, 'rot_quat':}
        # drift course (dirt and paved)
        # COLLECTED INDUSTRIAL1 return {'pos':(20.572, 161.438, 44.2149), 'rot': None, 'rot_quat': (-0.003, -0.005, -0.636, 0.771)}
        # rallycross course/default
        # COLLECTED INDUSTRIAL2 return {'pos': (4.85287, 160.992, 44.2151), 'rot': None, 'rot_quat': (-0.0032, 0.003, 0.763, 0.646)}
        # racetrack
        # COLLECTED INDUSTRIAL3 return {'pos':(184.983, -41.0821, 42.7761), 'rot': None, 'rot_quat':(-0.005, 0.001, 0.299, 0.954)}
    elif scenario_locale == 'derby':
        # the big 8
        #COLLECTED DERBY1
        return {'pos': (-174.882, 61.4717, 83.5583), 'rot': None, 'rot_quat': (-0.119, -0.001, 0.002, 0.993)}
    elif scenario_locale == 'east_coast_usa':
        # town industrial area
        #COLLECTED EAST_COAST_USA1 return {'pos':(736.768, -20.8732, 52.127), 'rot': None, 'rot_quat':(-0.006, -0.004, -0.186, 0.983)}
        # farmhouse
        #COLLECTED EAST_COAST_USA2 return {'pos':(-607.898, -354.424, 34.5097), 'rot': None, 'rot_quat':(-0.0007, 0.0373, 0.960, -0.279)}
        # gas station parking lot
        #COLLECTED EAST_COAST_USA3 return {'pos':(-758.764, 480.25, 23.774), 'rot': None, 'rot_quat':(-0.001, -0.010, -0.739, 0.673)}
        # sawmill
        #COLLECTED EAST_COAST_USA4 return {'pos':(261.326, -774.902, 46.2887), 'rot': None, 'rot_quat':(-0.005, 0.008, 0.950, -0.311)}
        # highway/default
        # COLLECTED EAST_COAST_USA5
        return {'pos':(900.643, -226.266, 40.191), 'rot': None, 'rot_quat':(-0.004, -0.0220, -0.0427, 0.99)}
    elif scenario_locale == 'driver_training': #etk driver experience center
        # north
        # COLLECTED DRIVER_TRAINING1 return {'pos':(-195.047, 253.654, 53.019), 'rot': None, 'rot_quat':(-0.006, -0.006, -0.272, 0.962)}
        # west
        # COLLECTED DRIVER_TRAINING2 return {'pos': (-394.541, 69.052, 51.2327), 'rot': None, 'rot_quat': (-0.0124, 0.0061, -0.318, 0.948)}
        # default -- weird behavior, havent used yet
        return {'pos':(60.6395, 70.8329, 38.3048), 'rot': None, 'rot_quat':(0.015, 0.006, 0.884, 0.467)}
        #return {'pos': (32.3209, 89.8991, 39.135), 'rot': None, 'rot_quat': (0.0154, -0.007, 0.794, 0.607)}
    elif scenario_locale == 'jungle_rock_island':
        # industrial site -- weird behavior, redo
        #return {'pos': (38.0602, 559.695, 156.726), 'rot': None, 'rot_quat': (-0.004, 0.005, 0.1, -0.005)}
        return {'pos': (-9.99082, 580.726, 156.72), 'rot': None, 'rot_quat': (-0.0066664000041783, 0.0050910739228129, 0.62305396795273, 0.78213387727737)}
        # observatory
        # COLLECTED JUNGLE_ROCK_ISLAND2 return {'pos':(-842.505, 820.688, 186.139), 'rot': None, 'rot_quat':(0.0003, 0.0122, 0.994, 0.113)}
        # hangar
        # COLLECTED JUNGLE_ROCK_ISLAND3 return {'pos':(818.098, -676.55, 160.034), 'rot': None, 'rot_quat':(-0.027693340554833, 0.011667124927044, -0.19988858699799, 0.97935771942139)}
        # peninsula
        # COLLECTED JUNGLE_ROCK_ISLAND4 return {'pos':(355.325, -775.203, 133.412), 'rot': None, 'rot_quat':(0.0243, -0.0422, -0.345, 0.937)}
        # port
        # COLLECTED JUNGLE_ROCK_ISLAND5 return {'pos':(-590.56, 312.523, 130.215), 'rot': None, 'rot_quat':(-0.0053834365680814, 0.00023860974761192, 0.013710686005652, 0.99989157915115)}
        # hill/default
        return {'pos':(124.232, -78.7489, 158.735), 'rot': None, 'rot_quat':(0.005, 0.0030082284938544, 0.96598142385483, 0.25854349136353)}
    elif scenario_locale == 'small_island':
        # north road/default
        # COLLECTED SMALL_ISLAND1 return {'pos': (254.77, 233.82, 39.5792), 'rot': None, 'rot_quat': (-0.013, 0.008, -0.0003, 0.1)}
        # south road
        # COLLECTED SMALL_ISLAND2 return {'pos':(-241.908, -379.478, 31.7824), 'rot': None, 'rot_quat':(0.008, 0.006, -0.677, 0.736)}
        # industrial area
        return {'pos':(126.907, 272.76, 40.0575), 'rot': None, 'rot_quat':(-0.0465, -0.0163, -0.0546, 0.997)}


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
    #d = "{}/{}".format(os.path.dirname(os.path.realpath(__file__)), training_dir)
    d = "H:/{}".format( training_dir)
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
    global base_filename, training_dir, default_model, setpoint

    f = setup_dir(training_dir)
    spawn_pt = spawn_point(default_scenario)
    random.seed(1703)
    setup_logging()

    #beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean', user='H:/BeamNG.research')
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

    #bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(60)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.ai_set_mode('traffic')
    vehicle.ai_drive_in_lane(True)
    vehicle.ai_set_aggression(0.3)
    vehicle.ai_set_speed(11.5, mode='set')

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()

    # Send random inputs to vehice and advance the simulation 20 steps
    imagecount = 0
    wheelvel = [0.1, 0.1, 0.1]
    vels = []
    with open(f, 'w') as datafile:
        datafile.write('filename,timestamp,steering_input,throttle_input,brake_input,driveshaft,engine_load,fog_lights,fuel,'
                       'lowpressure,oil,oil_temperature,parkingbrake,rpm,water_temperature,wheelspeed\n')
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
            datafile.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(filename,
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
                                                sensors['electrics']['water_temperature'],
                                                sensors['electrics']['wheelspeed']))

            # save the image
            image.save(filename)
            vels.append(sensors['electrics']['wheelspeed'])
            # step sim forward
            bng.step(20)
            print('{} seconds passed.'.format(str(round(sensors['timer']['time'], 2))))

            # check for stopping condition
            for i in range(len(wheelvel)-1):
                wheelvel[i] = wheelvel[i+1]
            wheelvel[2] = float(sensors['electrics']['wheelspeed'])
            #print("wheelspeed:{}".format(sensors['electrics']['wheelspeed']))
            #print('wheelvel = {}'.format(sum(wheelvel) / 3.0 ))
            print("average velocity: {}".format(sum(vels) / float(len(vels))))
            if sum(wheelvel) / 3.0 == 0.0:
                print("avg wheelspeed is zero; exiting...")
                bng.close()
                break





if __name__ == '__main__':
    main()
