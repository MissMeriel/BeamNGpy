"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random, math
import sys, time
from time import sleep
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, StaticObject
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from scipy.spatial.transform import Rotation as R
from PIL import Image
import PIL
import cv2
import scipy.misc

# globals
#training_dir = 'training_images_hirochi_remove'
default_scenario = 'industrial'
spawnpoint = 'racetrackstartinggate'
training_dir = 'H:/BeamNG_DAVE2_racetracks_all/training_images_{}-{}12/'.format(default_scenario, spawnpoint)
default_model = 'hopper'
setpoint = 40

dt = 20
#base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))
base_filename = '{}_{}_'.format(default_model, default_scenario.replace("_", ""))

def spawn_point(scenario_locale, spawn_point):
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
        if spawnpoint == 'west':
        # western industrial area -- didnt work with AI Driver
            return {'pos': (237.131, -379.919, 34.5561), 'rot': None, 'rot_quat': (-0.035, -0.0181, 0.949, 0.314)}
        # open industrial area -- didnt work with AI Driver
        # drift course (dirt and paved)
        elif spawnpoint == 'driftcourse':
            return {'pos':(20.572, 161.438, 44.2149), 'rot': None, 'rot_quat': (-0.003, -0.005, -0.636, 0.771)}
        # rallycross course/default
        elif spawnpoint == 'rallycross':
            return {'pos': (4.85287, 160.992, 44.2151), 'rot': None, 'rot_quat': (-0.0032, 0.003, 0.763, 0.646)}
        # racetrack
        elif spawnpoint == 'racetrackright':
            return {'pos':(184.983, -41.0821, 42.7761), 'rot': None, 'rot_quat':(-0.005, 0.001, 0.299, 0.954)}
        elif spawnpoint == 'racetrackleft':
            return {'pos':(216.578, -28.1725, 42.7788), 'rot': None, 'rot_quat':(-0.0051003773696721, -0.0031468099914491, -0.67134761810303, 0.74111843109131)}
        elif spawnpoint == 'racetrackstartinggate':
            return {'pos':(160.905, -91.9654, 42.8511), 'rot': None, 'rot_quat':(-0.0036226876545697, 0.0065293218940496, 0.92344760894775, -0.38365218043327)}
    elif scenario_locale == 'derby':
        # the big 8
        if spawnpoint == 'big8':
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
    # pos = (-0.3, 1, 1.0) # default
    #pos = (-0.5, 2, 1.0) #center edge of hood
    # pos = (-0.5, 1, 1.0)  # center middle of hood
    # pos = (-0.5, 0.4, 1.0)  # dashboard
    # pos = (-0.5, 0.38, 1.5) # roof
    pos = (-0.5, 0.38, 1.3) # windshield
    # direction = (0, 1, 0)
    direction = (0, 1.0, 0)
    fov = 50
    resolution = (1280,960) #(512, 512)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    # pos = pos
    # direction = (0, 1, 0)
    fov = 120
    # resolution = (1280,960)
    back_camera = Camera(pos, direction, fov, resolution,
                         colour=True, depth=True, annotation=True)

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    #lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    # vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle

def setup_dir(training_dir):
    #d = "{}/{}".format(os.path.dirname(os.path.realpath(__file__)), training_dir)
    d = training_dir
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

def turn_90(rot_quat):
    r = R.from_quat(list(rot_quat))
    r = r.as_euler('xyz', degrees=True)
    r[2] = r[2] + 90
    r = R.from_euler('xyz', r, degrees=True)
    return tuple(r.as_quat())

#return distance between two 3d points
def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def add_barriers(scenario):
    barrier_locations = []
    with open('industrial_racetrack_barrier_locations.txt', 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            line = line.split(' ')
            pos = line[0].split(',')
            pos = tuple([float(i) for i in pos])
            rot_quat = line[1].split(',')
            rot_quat = tuple([float(j) for j in rot_quat])
            rot_quat = turn_90(rot_quat)
            # barrier_locations.append({'pos':pos, 'rot_quat':rot_quat})
            # add barrier to scenario
            ramp = StaticObject(name='barrier{}'.format(i), pos=pos, rot=None, rot_quat=rot_quat, scale=(1, 1, 1),
                                shape='levels/Industrial/art/shapes/misc/concrete_road_barrier_a.dae')
            scenario.add_object(ramp)

def add_barrier_cars(scenario):
    barrier_locations = []
    with open('industrial_racetrack_car_locations.txt', 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            line = line.split(' ')
            pos = line[0].split(',')
            pos = tuple([float(i) for i in pos])
            rot_quat = line[1].split(',')
            rot_quat = tuple([float(j) for j in rot_quat])
            v = Vehicle('barrier_vehicle{}'.format(i), model=default_model, licence='BLK{}'.format(i), color='Cream')
            scenario.add_vehicle(v, pos=pos, rot=None, rot_quat=rot_quat)

# def parse_output_file():
def from_val_get_key(d, v):
    key_list = list(d.keys())
    val_list = list(d.values())
    position = val_list.index(v)
    return key_list[position]

def testrun(speed=11, risk=0.6):
    global base_filename, training_dir, default_model, setpoint
    global spawnpoint

    f = setup_dir(training_dir)
    spawn_pt = spawn_point(default_scenario, spawnpoint)
    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean', user='H:/BeamNG.research')
    scenario = Scenario(default_scenario, 'research_test')
    # add barriers and cars to get the ego vehicle to avoid the barriers
    add_barriers(scenario)
    add_barrier_cars(scenario)
    vehicle = Vehicle('ego_vehicle', model=default_model, licence='RED', color='White')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=spawn_pt['pos'], rot=None, rot_quat=spawn_pt['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    #bng.hide_hud()
    bng.set_nondeterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(36)  #

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()
    bng.switch_vehicle(vehicle)

    vehicle.ai_set_mode('traffic')
    vehicle.ai_drive_in_lane(False)
    vehicle.ai_set_speed(speed, mode='set')
    vehicle.ai_set_aggression(risk)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    # bng.resume()
    start_time = time.time()
    # Send random inputs to vehicle and advance the simulation 20 steps
    imagecount = 0
    with open(f, 'w') as datafile:
        datafile.write('filename,timestamp,steering_input,throttle_input,brake_input,driveshaft,engine_load,fog_lights,fuel,'
                       'lowpressure,oil,oil_temperature,parkingbrake,rpm,water_temperature,wheelspeed\n')
        reached_start = False
        vels = []
        vel_dict = {}
        while imagecount < 10000 :
            # collect images
            sensors = bng.poll_sensors(vehicle)
            image = sensors['front_cam']['colour'].convert('RGB')
            full_filename = "{}{}{}.jpg".format(training_dir, base_filename, imagecount)
            qualified_filename = "{}{}.jpg".format(base_filename, imagecount)

            # collect ancillary data
            datafile.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(qualified_filename,
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
            if sensors['timer']['time'] > 10:
                kph = sensors['electrics']['wheelspeed'] * 3.6
                vels.append(kph)
                vel_dict[sensors['timer']['time']] = kph

            if distance(spawn_pt['pos'], vehicle.state['pos']) < 5 and sensors['timer']['time'] > 10:
                reached_start = True
                break

            if sensors['damage']['damage'] > 0:
                print("CRASHED; QUITTING")
                break

            # save the image
            image.save(full_filename)
            imagecount += 1

            # plt.title("90 degrees FOV")
            # plt.imshow(image)
            # plt.pause(0.01)
            # plt.title("120 degrees FOV")
            # plt.imshow(sensors['back_cam']['colour'].convert('RGB'))
            # plt.pause(0.01)
            # step sim forward
            bng.step(1, wait=True)
        wheelspeed_avg = round((sum(vels) / float(len(vels))), 3)
        wheelspeed_var = round(np.var(vels), 3)
        return_str = ''
        if sensors['damage']['damage'] > 0:
            # print("QUIT DUE TO CRASH VALUE {}".format(sensors['damage']['damage']))
            return_str = "QUIT DUE TO CRASH VALUE {}".format(sensors['damage']['damage'])
            print(return_str)
        maxtime = from_val_get_key(vel_dict, max(vels))
        mintime = from_val_get_key(vel_dict, min(vels))
        info = "IMAGE COUNT:{}\nSIM TIME:{} WALLCLOCK TIME:{}\nWHEELSPEED AVG:{} VAR:{} \nMAX:{} @ timestep {} MIN:{} @ timestep {} ".format(imagecount, str(round(sensors['timer']['time'], 2)), time.time() - start_time, wheelspeed_avg, wheelspeed_var, round(max(vels), 3), round(maxtime,3), round(min(vels),3), round(mintime,3))
        print("SPEED:{} RISK:{}".format(speed, risk))
        print(info)
        return_str = "{}\n{}".format(return_str, info)
        bng.close()
        return return_str

def collection_run(speed=11, risk=0.6, num_samples=10000):
    global base_filename, training_dir, default_model, setpoint
    global spawnpoint

    f = setup_dir(training_dir)
    spawn_pt = spawn_point(default_scenario, spawnpoint)
    random.seed(1703)
    setup_logging()

    home = 'H:/BeamNG.research.v1.7.0.1clean' #'H:/BeamNG.tech.v0.21.3.0' #
    beamng = BeamNGpy('localhost', 64256, home=home, user='H:/BeamNG.research')
    scenario = Scenario(default_scenario, 'research_test')
    # add barriers and cars to get the ego vehicle to avoid the barriers
    add_barriers(scenario)
    add_barrier_cars(scenario)
    vehicle = Vehicle('ego_vehicle', model=default_model, licence='RED', color='White')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=spawn_pt['pos'], rot=None, rot_quat=spawn_pt['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    #bng.hide_hud()
    bng.set_nondeterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(36)  #

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()
    bng.switch_vehicle(vehicle)

    vehicle.ai_set_mode('traffic')
    vehicle.ai_drive_in_lane(False)
    vehicle.ai_set_speed(speed, mode='set')
    vehicle.ai_set_aggression(risk)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    # bng.resume()
    start_time = time.time()
    # Send random inputs to vehicle and advance the simulation 20 steps
    imagecount = 0
    timer = 0
    with open(f, 'w') as datafile:
        datafile.write('filename,timestamp,steering_input,throttle_input,brake_input,driveshaft,engine_load,fog_lights,fuel,'
                       'lowpressure,oil,oil_temperature,parkingbrake,rpm,water_temperature,wheelspeed\n')
        return_str = ''
        while imagecount < num_samples:
            sensors = bng.poll_sensors(vehicle)
            image = sensors['front_cam']['colour'].convert('RGB')
            full_filename = "{}{}{}.png".format(training_dir, base_filename, imagecount)
            qualified_filename = "{}{}.png".format(base_filename, imagecount)
            timer = sensors['timer']['time']
            steering = sensors['electrics']['steering_input']
            if timer > 10 and abs(steering) >= 0.1:
                # collect ancillary data
                datafile.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(qualified_filename,
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
                print(full_filename)
                image.save(full_filename)
                imagecount += 1

                # if sensors['timer']['time'] > 10:
                #     kph = sensors['electrics']['wheelspeed'] * 3.6
                #     vels.append(kph)
                #     vel_dict[sensors['timer']['time']] = kph
                #
                # if distance(spawn_pt['pos'], vehicle.state['pos']) < 5 and sensors['timer']['time'] > 10:
                #     reached_start = True
                #     break

            if sensors['damage']['damage'] > 0:
                return_str = "CRASHED at timestep {} speed {}; QUITTING".format(round(sensors['timer']['time'], 2), round(sensors['electrics']['wheelspeed']*3.6, 3))
                print(return_str)
                break

            bng.step(1, wait=True)

        bng.close()
        return return_str

def main():
    risks = [0.3, 0.6, 0.9]
    speeds = [16]
    results_string = ''
    # for r in risks:
    #     for s in speeds:
    #         results = testrun(speed=s, risk=r)
            # results_string = "{}\n\nSPEED:{}, RISK={}\n{}".format(results_string, s, r, results)
    # print(results_string)
    collection_run(speed=12, risk=0.7 , num_samples=10000)

if __name__ == '__main__':
    main()
