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
import os, copy
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, StaticObject
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from scipy.spatial.transform import Rotation as R
from PIL import Image
import PIL
import cv2
import scipy.misc
from scipy import interpolate
from ast import literal_eval

# globals
#training_dir = 'training_images_hirochi_remove'
default_scenario = 'industrial'
spawnpoint = 'racetrackstartinggate'
training_dir = 'H:/BeamNG_DAVE2_racetracks_all/training_images_{}-{}16/'.format(default_scenario, spawnpoint)
default_model = 'hopper'
throttle_setpoint = 40
integral=0; prev_error=0
steer_integral=0; steer_prev_error=0; steer_prev_setpoint = 0
remaining_centerline = []
centerline = []
avg_error = []
dt = 20
#base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))
base_filename = '{}_{}_'.format(default_model, default_scenario.replace("_", ""))
steps_per_second = 100
tight_curve = False

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
            # return {'pos':(216.775, -209.382, 45.4226), 'rot': None, 'rot_quat':(0.0031577029731125, 0.023062458261847, 0.99948292970657, 0.022182803601027)} #after hairpin turn
            # return {'pos': (271.936, -205.19, 44.0371), 'rot': None, 'rot_quat': (-0.014199636876583, 0.0051083415746689, 0.4541027545929, 0.89082151651382)}  # before hairpin turn
            # return {'pos': (241.368, -228.844, 45.1149), 'rot': None, 'rot_quat': (0.009736854583025, 0.0039774458855391, 0.24705672264099, 0.9689439535141)}  # before hairpin turn
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
    resolution = (200, 150) #(1280,960) #(512, 512)
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

def rpy_from_quaternion(rot_quat):
    r = R.from_quat(list(rot_quat))
    r = r.as_euler('xyz', degrees=True)
    return r

def rpy_from_vec(vec):
    r = R.from_rotvec(list(vec))
    r = r.as_euler('xyz', degrees=True)
    return r

#return distance between two 3d points
def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def distance2D(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

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
    # add_barrier_cars(scenario)
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

def throttle_PID(kph, dt):
    global integral, prev_error, throttle_setpoint
    # kp = 0.001; ki = 0.00001; kd = 0.0001
    # kp = .3; ki = 0.01; kd = 0.1
    # kp = 0.15; ki = 0.0001; kd = 0.008 # worked well but only got to 39kph
    kp = 0.19; ki = 0.0001; kd = 0.008
    error = throttle_setpoint - kph
    deriv = (error - prev_error) / dt
    integral = integral + error * dt
    w = kp * error + ki * integral + kd * deriv
    prev_error = error
    return w

def steering_PID(curr_steering,  steer_setpoint, dt):
    global steer_integral, steer_prev_error, steer_prev_setpoint
    global tight_curve, avg_error
    # kp = 1.1; ki = 0.05; kd = 0.001 #0.075
    if tight_curve:
        # kp = 50; ki = 0.0; kd = 0.1 #0.075
        print("returning ", 1 * np.sign(steer_setpoint), "\n")
        return 1 * np.sign(steer_setpoint)
    else:
        # kp = 1.1; ki = 0.05; kd = 0.001
        # kp = 1.7; ki = 1.1; kd = 0.0
        kp = 170; ki = 50; kd = 0.1 # works with 40-30 kph
        # kp = 0.167; ki = 0.11; kd = 0.14
        print("steer_setpoint", steer_setpoint)
        print("curr_steering ", curr_steering)
        print("tigh_curve", tight_curve)
        # print("dt:", dt)
        error = steer_setpoint - curr_steering
        deriv = (error - steer_prev_error) / dt
        integral = steer_integral + error * dt
        w = kp * error + ki * integral + kd * deriv
        steer_prev_error = error
        print("error term", kp * error)
        print("deriv term", kd * deriv)
        print("integral term", ki * integral)
        print("steering w", w)
        # print("return val", curr_steering + w)
        # print("\n")
        print("returning ", w+curr_steering, "\n")
    return w + curr_steering

def road_analysis(bng):
    global centerline
    roads = bng.get_roads()
    colors = ['b','g','r','c','m','y','k']
    symbs = ['-','--','-.',':','.',',','v','o','1',]
    for road in roads:
        road_edges = bng.get_road_edges(road)
        x_temp = []
        y_temp = []
        dont_add = False
        for edge in road_edges:
            if edge['middle'][0] < 100:
                dont_add = True
                break
            if edge['middle'][1] < -300 or edge['middle'][1] > 0:
                dont_add = True
                break
            if not dont_add:
                x_temp.append(edge['middle'][0])
                y_temp.append(edge['middle'][1])
        if not dont_add:
            symb = '{}{}'.format(random.choice(colors), random.choice(symbs))
            plt.plot(x_temp, y_temp, symb, label=road)
    plt.legend()
    plt.show()
    plt.pause(0.001)
    # get relevant road
    edges = bng.get_road_edges('7983')
    middle = [edge['middle'] for edge in edges]
    centerline = middle
    return middle

def plot_trajectory(traj, title="Trajectory", label1="AI behavior"):
    global centerline
    x = [t[0] for t in traj]
    y = [t[1] for t in traj]
    plt.plot(x,y, 'bo', label=label1)
    # plt.gca().set_aspect('equal')
    # plt.axis('square')
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.plot([t[0] for t in centerline], [t[1] for t in centerline], 'r+', label="AI line script")
    # Set a title of the current axes.
    plt.title(title)
    plt.legend()
    plt.draw()
    # Display a figure.
    plt.show()
    plt.pause(0.1)

def intake_ai_lap_poses(filename="ai_lap_data.txt"):
    global centerline
    lap_traj = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        # lap_traj =
        for line in lines:
            line = line.replace("\n", "")
            # print(line)
            line = literal_eval(line)
            lap_traj.append(line)
    centerline = lap_traj
    return lap_traj

def create_ai_line(bng, filename="ai_lap_data.txt"):
    line = []; points = []; point_colors = []; spheres = []; sphere_colors = []
    poses = intake_ai_lap_poses(filename)
    elapsed_time = 0
    for i,p in enumerate(poses[:-1]):
        if p[1] > -50:
            p[1] += 3
        if p[0] > 300:
            p[0] += 3
        if p[1] < -225:
            p[0] -= 3
        # if i % 5 == 0:
        if True:
            elapsed_time += distance(p, poses[i+1]) / 12
            line.append({"x":p[0], "y":p[1], "z":p[2], "t":elapsed_time})
            points.append(p)
            point_colors.append([0, 1, 0, 0.1])
            spheres.append([p[0], p[1], p[2], 0.25])
            sphere_colors.append([np.sin(np.radians(10)), 0, 0, 0.8])
    bng.add_debug_line(points, point_colors,
                       spheres=spheres, sphere_colors=sphere_colors,
                       cling=True, offset=1)
    return line, bng

def create_ai_line_from_road(spawn, bng):
    global centerline, remaining_centerline
    line = []; points = []; point_colors = []; spheres = []; sphere_colors = []; traj = []
    middle = road_analysis(bng)
    splitindex = 3 #70 #62 #115 #3
    middle_end = middle[:splitindex] #middle[:3]
    middle = middle[splitindex:]
    temp = [list(spawn['pos'])]; temp.extend(middle); middle = temp
    middle.extend(middle_end)
    centerline = copy.deepcopy(middle)
    remaining_centerline = copy.deepcopy(middle)
    elapsed_time = 0; count = 0
    with open("centerline_lap_data.txt", 'w') as f:
        for i,p in enumerate(middle[:-1]):
            f.write("{}\n".format(p))
            elapsed_time += distance(p, middle[i+1]) / 12
            traj.append([p[0],p[1]])
            linedict = {"x": p[0], "y": p[1], "z": p[2], "t": elapsed_time}
            print(linedict)
            line.append(linedict)
            points.append([p[0], p[1], p[2]])
            point_colors.append([0, 1, 0, 0.1])
            spheres.append([p[0], p[1], p[2], 0.25])
            sphere_colors.append([1, 0, 0, 0.8])
            count += 1
        # plot_trajectory(traj, "Points on Script So Far")
    # ai_line = create_ai_line(bng)
    # print("points in ai line:{}".format(len(ai_line)))
    # print("spawn point:{}".format(spawn))
    # print("beginning of script:{}".format(middle[0]))
    plot_trajectory(traj, "Points on Script (Final)", "AI debug line")
    bng.add_debug_line(points, point_colors,
                       spheres=spheres, sphere_colors=sphere_colors,
                       cling=True, offset=0.1)
    return line, bng

def create_ai_line_from_road_with_interpolation(spawn, bng):
    global centerline, remaining_centerline
    line = []; points = []; point_colors = []; spheres = []; sphere_colors = []; traj = []
    middle = road_analysis(bng)
    middle_end = middle[:3]
    middle = middle[3:]
    temp = [list(spawn['pos'])]; temp.extend(middle); middle = temp
    middle.extend(middle_end)
    centerline = copy.deepcopy(middle)
    remaining_centerline = copy.deepcopy(middle)
    timestep = 0.1; elapsed_time = 0; count = 0
    with open("centerline_lap_data.txt", 'w') as f:
        for i,p in enumerate(middle[:-1]):
            f.write("{}\n".format(p))
            # interpolate at 1m distance
            if distance(p, middle[i+1]) > 1:
                y_interp = interpolate.interp1d([p[0], middle[i+1][0]], [p[1], middle[i+1][1]])
                num = int(distance(p, middle[i+1]))
                xs = np.linspace(p[0], middle[i+1][0], num=num, endpoint=True)
                ys = y_interp(xs)
                for x,y in zip(xs,ys):
                    traj.append([x,y])
                    line.append({"x":x, "y":y, "z":p[2], "t":i * timestep})
                    points.append([x, y, p[2]])
                    point_colors.append([0, 1, 0, 0.1])
                    spheres.append([x, y, p[2], 0.25])
                    sphere_colors.append([1, 0, 0, 0.8])
            else:
            # if True:
            # # if count < 2:
                elapsed_time += distance(p, middle[i+1]) / 12
                traj.append([p[0],p[1]])
                linedict = {"x": p[0], "y": p[1], "z": p[2], "t": elapsed_time}
                print(linedict)
                line.append(linedict)
                points.append([p[0], p[1], p[2]])
                point_colors.append([0, 1, 0, 0.1])
                spheres.append([p[0], p[1], p[2], 0.25])
                sphere_colors.append([1, 0, 0, 0.8])
                count += 1
            # plot_trajectory(traj, "Points on Script So Far")
    # print("points in centerline:{}".format(len(middle)))
    # ai_line = create_ai_line(bng)
    # print("points in ai line:{}".format(len(ai_line)))
    print("spawn point:{}".format(spawn))
    print("beginning of script:{}".format(middle[0]))
    plot_trajectory(traj, "Points on Script (Final)", "AI debug line")
    centerline = copy.deepcopy(traj)
    remaining_centerline = copy.deepcopy(traj)
    for i in range(4):
        centerline.extend(copy.deepcopy(centerline))
        remaining_centerline.extend(copy.deepcopy(remaining_centerline))
    bng.add_debug_line(points, point_colors,
                       spheres=spheres, sphere_colors=sphere_colors,
                       cling=True, offset=0.1)
    return line, bng

def law_of_cosines(A, B, C):
    dist_AB = distance2D(A[:2], B[:2])
    dist_BC = distance2D(B[:2], C[:2])
    # p23 = distance2D(left, right)
    ab = np.array(B[:2]) - np.array(A[:2])
    bc = np.array(C[:2]) - np.array(B[:2])
    # return math.acos((math.pow(p12,2) + math.pow(p13,2) - math.pow(p23,2))/(2*p12*p13))
    return math.acos(np.dot(ab, bc) / (dist_AB * dist_BC))

def angle_between(next_waypoint, vehicle_state):
    # print("remaining centerline", remaining_centerline)
    # yaw = math.radians(vehicle_state['yaw'][0]) #
    # yaw = rpy_from_vec(vehicle_state['dir']); yaw = yaw[2]
    # reference_pt = [vehicle_state['pos'][0] + 5 * math.cos(yaw), vehicle_state['pos'][1] + 5 * math.sin(yaw)]
    # inner_angle = law_of_cosines(next_waypoint, vehicle_state['pos'], reference_pt)
    # waypoint_angle = math.tan(next_waypoint[1] / next_waypoint[0])
    # vehicle_angle = math.tan(vehicle_state['pos'][1] / vehicle_state['pos'][0])
    # inner_angle = waypoint_angle - vehicle_angle
    vehicle_angle = math.atan2(vehicle_state['front'][1]-vehicle_state['pos'][1], vehicle_state['front'][0]-vehicle_state['pos'][0])
    waypoint_angle = math.atan2((next_waypoint[1]-vehicle_state['pos'][1]),(next_waypoint[0]-vehicle_state['pos'][0]))
    inner_angle = -(waypoint_angle - vehicle_angle)
    # print("next_waypoint",next_waypoint)
    # print("vehicle_state", vehicle_state)
    # # print("vehicle_state['pos']", vehicle_state['pos'])
    # print("vehicle_angle (degrees):", math.degrees(vehicle_angle))
    # print("waypoint_angle (degrees):",math.degrees(waypoint_angle))
    # # print("inner_angle (degrees):",math.degrees(inner_angle))
    # print("inner_angle (degrees):", math.degrees(math.atan2(math.sin(inner_angle), math.cos(inner_angle))))
    return math.atan2(math.sin(inner_angle), math.cos(inner_angle))

def steering_setpoint(vehicle_state, traj=None):
    global remaining_centerline, centerline, throttle_setpoint
    global steer_integral, steer_prev_error, steer_prev_setpoint
    global tight_curve, avg_error
    # update next waypoint if you've reached your current one
    # normal driving circumstances
    # is there an upcoming tight curve?
    # shallow curve coming up soon
    if not tight_curve and abs(angle_between(remaining_centerline[0], vehicle_state)) > (math.pi / 6):
        print(abs(angle_between(remaining_centerline[0], vehicle_state)), "> (math.pi / 6)")
        tight_curve = True
    else:
        tight_curve = False

    # steep curve coming up in near future
    i = 0
    while  distance2D(vehicle_state['pos'][:2], remaining_centerline[i]) < 11: #10:
        i += 1
        a = angle_between(remaining_centerline[i], vehicle_state)
        print("lookahead angle:", a)
        if abs(a) > (math.pi / 4):
            print(abs(a), "> math.pi/4")
            tight_curve = True
            print("TIGHT CURVE!!!!!\nTIGHT CURVE!!!!!\nTIGHT CURVE!!!!!")
            break

    # while not tight_curve and distance2D(vehicle_state['pos'][:2], remaining_centerline[i]) < 13:
    #     i += 1
    #     a = angle_between(remaining_centerline[i], vehicle_state)
    #     print("lookahead angle:", a)
    #     if abs(a) > (math.pi / 4):
    #         tight_curve = True
    #         print("TIGHT CURVE!!!!!\nTIGHT CURVE!!!!!\nTIGHT CURVE!!!!!")
    #         break
    if tight_curve:
        throttle_setpoint = 30
        steer_integral = 0; steer_prev_error = 0
    else:
        i = 0
        throttle_setpoint = 40
        while distance2D(vehicle_state['pos'][:2], remaining_centerline[i]) < 7.5:
            i += 1
        if i > 0:
            steer_integral = 0; steer_prev_error = 0
    remaining_centerline = remaining_centerline[i:]
    next_waypoint = remaining_centerline[0]
    inner_angle = angle_between(next_waypoint, vehicle_state)
    # print("distance to next waypoint:",distance2D(vehicle_state['pos'][:2], next_waypoint[:2]))
    # plt.figure(figsize=(8,8))
    # plt.scatter(next_waypoint[0], next_waypoint[1], marker="o", label="next_waypoint")
    # plt.scatter(vehicle_state['pos'][0], vehicle_state['pos'][1], marker="v", label="vehicle pos")
    # plt.scatter(0, 0, marker="X", label="origin")
    # plt.plot([t[0] for t in centerline], [t[1] for t in centerline], 'r+', label="AI line script")
    # plt.axis('square')
    # plt.legend()
    # plt.show()
    # plt.pause(0.01)
    return math.atan2(math.sin(inner_angle), math.cos(inner_angle))

def plot_inputs(timestamps, inputs, title="Inputs by Time"):
    plt.plot(timestamps, inputs)
    plt.title(title)
    plt.show()
    plt.pause(0.01)

def collection_run(speed=11, risk=0.6, num_samples=10000):
    global base_filename, training_dir, default_model, setpoint
    global spawnpoint, steps_per_second

    f = setup_dir(training_dir)
    spawn_pt = spawn_point(default_scenario, spawnpoint)
    random.seed(1703)
    setup_logging()

    home = 'H:/BeamNG.research.v1.7.0.1clean' #'H:/BeamNG.tech.v0.21.3.0' #
    beamng = BeamNGpy('localhost', 64256, home=home, user='H:/BeamNG.research')
    scenario = Scenario(default_scenario, 'research_test')
    # add barriers and cars to get the ego vehicle to avoid the barriers
    add_barriers(scenario)
    # add_barrier_cars(scenario)
    vehicle = Vehicle('ego_vehicle', model=default_model, licence='RED', color='White')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=spawn_pt['pos'], rot=None, rot_quat=spawn_pt['rot_quat'])
    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)
    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)
    # bng.hide_hud()
    bng.set_nondeterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(steps_per_second)  #
    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()
    bng.switch_vehicle(vehicle)

    # ai_line, bng = create_ai_line_from_road(spawn_pt, bng)
    ai_line, bng = create_ai_line_from_road_with_interpolation(spawn_pt, bng)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    # bng.resume()
    start_time = time.time()
    # Send random inputs to vehicle and advance the simulation 20 steps
    imagecount = 0
    timer = 0; traj = []; steering_inputs = []; timestamps = []; kphs = []
    with open(f, 'w') as datafile:
        datafile.write('filename,timestamp,steering_input,throttle_input,brake_input,driveshaft,engine_load,fog_lights,fuel,'
                       'lowpressure,oil,oil_temperature,parkingbrake,rpm,water_temperature,wheelspeed\n')
        return_str = ''
        while imagecount < num_samples:
            sensors = bng.poll_sensors(vehicle)
            image = sensors['front_cam']['colour'].convert('RGB')
            full_filename = "{}{}{}.jpg".format(training_dir, base_filename, imagecount)
            qualified_filename = "{}{}.jpg".format(base_filename, imagecount)
            steering = sensors['electrics']['steering_input']
            dt = sensors['timer']['time'] - timer
            if dt > 0:
                throttle = throttle_PID(sensors['electrics']['wheelspeed'] * 3.6, dt)
            print("current kph:{} dt:{} throttle:{}".format(sensors['electrics']['wheelspeed'] * 3.6, dt, throttle))
            vehicle.update_vehicle()
            # points=[vehicle.state['front']]
            # point_colors = [[0, 1, 0, 0.1]]
            # spheres=[[vehicle.state['front'][0],vehicle.state['front'][1],vehicle.state['front'][1],0.25]]
            # sphere_colors=[[1, 0, 0, 0.8]]
            # bng.add_debug_line(points, point_colors,
            #                    spheres=spheres, sphere_colors=sphere_colors,
            #                    cling=True, offset=0.1)
            try:
                steering_setpt = steering_setpoint(vehicle.state, traj)
                if dt > 0:
                    steering = steering_PID(steering, steering_setpt, dt)
                vehicle.control(steering=steering, throttle=throttle)
                timer = sensors['timer']['time']
                traj.append(vehicle.state['pos'])
                timestamps.append(timer)
                steering_inputs.append(steering)
                kphs.append(sensors['electrics']['wheelspeed'] * 3.6)
            except:
                plot_trajectory(traj, "Car Behavior using AI Script")
                plot_inputs(timestamps, steering_inputs, title="Steering Inputs by Time")
                plot_inputs(timestamps, kphs, title="KPHs by Time")
                exit(0)

            if timer > 10: # and abs(steering) >= 0.1:
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
                 image.save(full_filename)
                 imagecount += 1

            if sensors['damage']['damage'] > 0:
                return_str = "CRASHED at timestep {} speed {}; QUITTING".format(round(sensors['timer']['time'], 2), round(sensors['electrics']['wheelspeed']*3.6, 3))
                print(return_str)
                break
            if timer > 300:
                break

            bng.step(1, wait=True)
    plot_trajectory(traj, "Car Behavior using AI Script")
    plot_inputs(timestamps, steering_inputs, title="Steering Inputs by Time")
    plot_inputs(timestamps, kphs, title="KPHs by Time")
    bng.close()
    plot_trajectory(traj, "Car Behavior using AI Script")
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
    collection_run(speed=12, risk=0.3, num_samples=10000)

if __name__ == '__main__':
    main()
