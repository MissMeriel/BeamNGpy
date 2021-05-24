import mmap
import random
import sys
from time import sleep
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, StaticObject
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from PIL import Image
import PIL
import cv2
import scipy.misc
import copy
from DAVE2 import Model
import statistics, math
import json
from scipy.spatial.transform import Rotation as R

# globals
default_color = 'White' #'Red'
default_scenario = 'industrial' #'automation_test_track'
default_spawnpoint = 'racetrackstartinggate'
dt = 20
integral = 0.0
prev_error = 0.0
setpoint = 40 #50.0 #53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country
lanewidth = 3.75 #2.25
centerline = []
steps_per_sec = 36

def spawn_point(scenario_locale, spawn_point ='default'):
    global lanewidth
    if scenario_locale == 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale == 'west_coast_usa':
        if spawn_point == 'midhighway':
            # mid highway scenario (past shadowy parts of road)
            return {'pos': (-145.775, 211.862, 115.55), 'rot': None, 'rot_quat': (0.0032586499582976, -0.0018308814615011, 0.92652350664139, -0.37621837854385)}
        # actually past shadowy parts of road?
        #return {'pos': (95.1332, 409.858, 117.435), 'rot': None, 'rot_quat': (0.0077012465335429, 0.0036200874019414, 0.90092438459396, -0.43389266729355)}
        # surface road (crashes early af)
        elif spawn_point == 'surfaceroad1':
            return {'pos': (945.285, 886.716, 132.061), 'rot': None, 'rot_quat': (-0.043629411607981, 0.021309537813067, 0.98556911945343, 0.16216005384922)}
        # surface road 2
        elif spawn_point == 'surfaceroad2':
            return {'pos': (900.016, 959.477, 127.227), 'rot': None, 'rot_quat': (-0.046136282384396, 0.018260028213263, 0.94000166654587, 0.3375423848629)}
        # surface road 3 (start at top of hill)
        elif spawn_point == 'surfaceroad3':
            return {'pos':(873.494, 984.636, 125.398), 'rot': None, 'rot_quat':(-0.043183419853449, 2.3034785044729e-05, 0.86842048168182, 0.4939444065094)}
        # surface road 4 (right turn onto surface road) (HAS ACCOMPANYING AI DIRECTION AS ORACLE)
        elif spawn_point == 'surfaceroad4':
            return {'pos': (956.013, 838.735, 134.014), 'rot': None, 'rot_quat': (0.020984912291169, 0.037122081965208, -0.31912142038345, 0.94675397872925)}
        # surface road 5 (ramp past shady el)
        elif spawn_point == 'surfaceroad5':
            return {'pos':(166.287, 812.774, 102.328), 'rot': None, 'rot_quat':(0.0038638345431536, -0.00049926445353776, 0.60924011468887, 0.79297626018524)}
        # entry ramp going opposite way
        elif spawn_point == 'entryrampopp':
            return {'pos': (850.136, 946.166, 123.827), 'rot': None, 'rot_quat': (-0.030755277723074, 0.016458060592413, 0.37487033009529, 0.92642092704773)}
        # racetrack
        elif spawn_point == 'racetrack':
            return {'pos': (395.125, -247.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif scenario_locale == 'smallgrid':
        return {'pos':(0.0, 0.0, 0.0), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        # right after toll
        return {'pos': (-852.024, -517.391 + lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
        # return {'pos':(-717.121, 101, 118.675), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-717.121, 101, 118.675), 'rot': None, 'rot_quat': (0, 0, 0.918812, -0.394696)}
    elif scenario_locale == 'automation_test_track':
        if spawn_point == 'startingline':
            # starting line
            return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        elif spawn_point == 'starting line 30m down':
            # 30m down track from starting line
            return {'pos': (530.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        elif spawn_point == 'handlingcircuit':
            # handling circuit
            return {'pos': (-294.031, 10.4074, 118.518), 'rot': None, 'rot_quat': (0, 0, 0.708103, 0.706109)}
        elif spawn_point == 'handlingcircuit2':
            return {'pos': (-280.704, -25.4946, 118.794), 'rot': None, 'rot_quat': (-0.00862686, 0.0063203, 0.98271, 0.184842)}
        elif spawn_point == 'handlingcircuit3':
            return {'pos': (-214.929, 61.2237, 118.593), 'rot': None, 'rot_quat': (-0.00947676, -0.00484788, -0.486675, 0.873518)}
        elif spawn_point == 'handlingcircuit4':
            # return {'pos': (-180.663, 117.091, 117.654), 'rot': None, 'rot_quat': (0.0227101, -0.00198367, 0.520494, 0.853561)}
            # return {'pos': (-171.183,147.699,117.438), 'rot': None, 'rot_quat': (0.001710215350613,-0.039731655269861,0.99312973022461,-0.11005393415689)}
            return {'pos': (-173.009,137.433,116.701), 'rot': None,'rot_quat': (0.0227101, -0.00198367, 0.520494, 0.853561)}
            return {'pos': (-166.679, 146.758, 117.68), 'rot': None,'rot_quat': (0.075107827782631, -0.050610285252333, 0.99587279558182, 0.0058960365131497)}
        elif spawn_point == 'rally track':
            # rally track
            return {'pos': (-374.835, 84.8178, 115.084), 'rot': None, 'rot_quat': (0, 0, 0.718422, 0.695607)}
        elif spawn_point == 'highway':
            # highway (open, farm-like)
            return {'pos': (-294.791, -255.693, 118.703), 'rot': None, 'rot_quat': (0, 0, -0.704635, 0.70957)}
        elif spawn_point == 'highwayopp':
            # highway (open, farm-like)
            return {'pos': (-542.719,-251.721,117.083), 'rot': None, 'rot_quat': (0.0098941307514906,0.0096141006797552,0.72146373987198,0.69231480360031)}
        elif spawn_point == 'default':
            # default
            return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
    elif scenario_locale == 'industrial':
        if spawn_point == 'west':
            # western industrial area -- didnt work with AI Driver
            return {'pos': (237.131, -379.919, 34.5561), 'rot': None, 'rot_quat': (-0.035, -0.0181, 0.949, 0.314)}
        # open industrial area -- didnt work with AI Driver
        # drift course (dirt and paved)
        elif spawn_point == 'driftcourse':
            return {'pos': (20.572, 161.438, 44.2149), 'rot': None, 'rot_quat': (-0.003, -0.005, -0.636, 0.771)}
        # rallycross course/default
        elif spawn_point == 'rallycross':
            return {'pos': (4.85287, 160.992, 44.2151), 'rot': None, 'rot_quat': (-0.0032, 0.003, 0.763, 0.646)}
        # racetrack
        elif spawn_point == 'racetrackright':
            return {'pos': (184.983, -41.0821, 42.7761), 'rot': None, 'rot_quat': (-0.005, 0.001, 0.299, 0.954)}
        elif spawn_point == 'racetrackleft':
            return {'pos': (216.578, -28.1725, 42.7788), 'rot': None, 'rot_quat': (-0.0051, -0.003147, -0.67135, 0.74112)}
        elif spawn_point == 'racetrackstartinggate':
            return {'pos':(160.905, -91.9654, 42.8511), 'rot': None, 'rot_quat':(-0.0036226876545697, 0.0065293218940496, 0.92344760894775, -0.38365218043327)}
        elif spawn_point == "racetrackstraightaway":
            return {'pos':(262.328, -35.933, 42.5965), 'rot': None, 'rot_quat':(-0.010505940765142, 0.029969356954098, -0.44812294840813, 0.89340770244598)}
        elif spawn_point == "racetrackcurves":
            return {'pos':(215.912,-243.067,45.8604), 'rot': None, 'rot_quat':(0.029027424752712,0.022241719067097,0.98601061105728,0.16262225806713)}


def setup_sensors(vehicle):
    # Set up sensors
    # pos = (-0.3, 1, 1.0) # default
    # pos = (-0.5, 1, 1.0)  # center middle of hood
    # pos = (-0.5, 0.4, 1.0)  # dashboard
    # pos = (-0.5, 0.38, 1.5) # roof
    # pos = (-0.5, 0.38, 1.1) # rearview?
    pos = (-0.5, 0.38, 1.3) # windshield
    # direction = (0, 1, 0)
    direction = (0, 1.0, 0)
    fov = 50
    resolution = (200,150)
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

def ms_to_kph(wheelspeed):
    return wheelspeed * 3.6

def throttle_PID(kph, dt):
    global integral, prev_error, setpoint
    kp = 0.001; ki = 0.00001; kd = 0.0001
    error = setpoint - kph
    deriv = (error - prev_error) / dt
    integral = integral + error * dt
    w = kp * error + ki * integral + kd * deriv
    prev_error = error
    return w

def diff_damage(damage, damage_prev):
    new_damage = 0
    if damage is None or damage_prev is None:
        return 0
    new_damage = damage['damage'] - damage_prev['damage']
    return new_damage

# takes in 3D array of sequential [x,y]
# produces plot
def plot_deviation(trajectories, model, deflation_pattern, centerline):
    i = 0; x = []; y = []
    for point in centerline:
        x.append(point[0])
        y.append(point[1])
    plt.plot(x, y, label="Centerline")
    for t in trajectories:
        x = []; y = []
        for point in t:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, label="Run {}".format(i))
        i += 1
    # plt.xlabel('x - axis')
    # plt.ylabel('y - axis')
    # Set a title of the current axes.
    plt.title('Trajectories with {} {}'.format(model, deflation_pattern))
    # show a legend on the plot
    plt.legend()
    # Display a figure.
    plt.show()
    plt.pause(0.1)
    return

def lineseg_dists(p, a, b):
    """Cartesian distance from point to line segment
    Edited to support arguments as series, from:
    https://stackoverflow.com/a/54442561/11208892
    Args:
        - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
        - a: np.array of shape (x, 2)
        - b: np.array of shape (x, 2)
    """
    # normalized tangent vectors
    d_ba = b - a
    d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1]).reshape(-1, 1)))

    # signed parallel distance components
    # rowwise dot products of 2D vectors
    s = np.multiply(a - p, d).sum(axis=1)
    t = np.multiply(p - b, d).sum(axis=1)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(s))])

    # perpendicular distance component
    # rowwise cross products of 2D vectors
    d_pa = p - a
    c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

    return np.hypot(h, c)

def threeD_to_twoD(arr):
    return [[x[0],x[1]] for x in arr]

#return distance between two 3d points
def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def dist_from_line(centerline, point):
    a = threeD_to_twoD(centerline[:-1])
    b = threeD_to_twoD(centerline[1:])
    a = np.array(a)
    b = np.array(b)
    dist = lineseg_dists([point[0], point[1]], a, b)
    return dist

def calc_deviation_from_center(centerline, trajectories):
    dists = []
    stddev = 0
    for t in trajectories:
        x = []; y = []
        for point in t:
            dist = dist_from_line(centerline, point)
            # print("dist:{}".format(dist))
            dists.append(min(dist))
    stddev = statistics.stdev(dists)
    return stddev

def perturb_part_config(model, var, val):
    pristine_pc = "C:/Users/merie/Documents/BeamNG.research/vehicles/{}/pristine.pc".format(model)
    perturbed_pc = "C:/Users/merie/Documents/BeamNG.research/vehicles/{}/{}{}.pc".format(model, var, val)
    with open(pristine_pc) as f:
        data = json.load(f)
        data['vars'][var] = val
        with open(perturbed_pc) as f2:
            json.dump(data, f2)
    perturbed_pc = ""
    return perturbed_pc

def run_scenario_ai_version(vehicle_model='etk800', deflation_pattern=[0,0,0,0], parts_config='vehicles/hopper/custom.pc'):
    global base_filename, default_color, default_scenario, default_spawnpoint, setpoint, steps_per_sec
    global prev_error

    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model, licence='AI', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, default_spawnpoint)
    # scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])


    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(steps_per_sec)  # With 100hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # create vehicle to be chased
    chase_vehicle = Vehicle('chase_vehicle', model='miramar', licence='CHASEE', color='Red')
    bng.spawn_vehicle(chase_vehicle, pos=(469.784, 346.391, 144.982), rot=None,
                      rot_quat=(-0.0037852677050978, -0.0031219546217471, -0.78478640317917, 0.61974692344666))

    bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'], partConfig=parts_config)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    #bng.resume()

    # perturb vehicle
    print("vehicle position before deflation via beamstate:{}".format(vehicle.get_object_position()))
    print("vehicle position before deflation via vehicle state:{}".format(vehicle.state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    plt.imshow(image)
    plt.pause(0.01)
    vehicle.deflate_tires(deflation_pattern)
    bng.step(steps_per_sec * 6)
    vehicle.update_vehicle()
    # print("vehicle position after deflation via beamstate:{}".format(vehicle.get_object_position()))
    # print("vehicle position after deflation via vehicle state:{}".format(vehicle.state))
    pitch = vehicle.state['pitch'][0]
    roll = vehicle.state['roll'][0]
    z = vehicle.state['pos'][2]
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    plt.imshow(image)
    plt.pause(0.01)
    bng.resume()

    vehicle.ai_set_mode('chase')
    vehicle.ai_set_target('chase_vehicle')
    vehicle.ai_drive_in_lane(True)
    damage_prev = None
    runtime = 0.0
    traj = []; kphs = []
    for _ in range(650):
        image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
        damage = bng.poll_sensors(vehicle)['damage']
        wheelspeed = bng.poll_sensors(vehicle)['electrics']['wheelspeed']
        new_damage = diff_damage(damage, damage_prev)
        damage_prev = damage
        runtime = bng.poll_sensors(vehicle)['timer']['time']
        vehicle.update_vehicle()
        traj.append(vehicle.state['pos'])
        kphs.append(ms_to_kph(wheelspeed))
        if new_damage > 0.0:
            break
        bng.step(5)
    bng.close()
    results = {'runtime': round(runtime,3), 'damage': damage, 'kphs':kphs, 'traj':traj, 'pitch': round(pitch,3), 'roll':round(roll,3), "z":round(z,3), 'final_img':image }
    return results

def plot_input(timestamps, input, input_type):
    plt.plot(timestamps, input)
    plt.xlabel('Timestamps')
    plt.ylabel('{} input'.format(input_type))
    # Set a title of the current axes.
    plt.title("{} over time".format(input_type))
    plt.show()
    plt.pause(0.1)

def plot_trajectory(traj):
    x = [t[0] for t in traj]
    y = [t[1] for t in traj]
    plt.plot(x,y)
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    # Set a title of the current axes.
    plt.title('Trajectory')
    # Display a figure.
    plt.show()
    plt.pause(0.1)

def run_scenario(vehicle_model='etk800', deflation_pattern=[0,0,0,0], parts_config='vehicles/hopper/custom.pc'):
    global base_filename, default_color, default_scenario,default_spawnpoint, setpoint, steps_per_sec
    global integral, prev_error
    integral = 0.0
    prev_error = 0.0

    # setup DNN model + weights
    sm = Model()
    steering_model = Model().define_model_BeamNG("BeamNGmodel-racetracksteering8.h5")
    throttle_model = Model().define_model_BeamNG("BeamNGmodel-racetrackthrottle8.h5")

    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean', user='H:/BeamNG.research')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model,
                      licence='EGO', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, default_spawnpoint)
    scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat']) #, partConfig=parts_config)
    add_barriers(scenario)
    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    #bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(steps_per_sec)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()
    # bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'], partConfig=parts_config)

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()

    # perturb vehicle
    print("vehicle position before deflation via beamstate:{}".format(vehicle.get_object_position()))
    print("vehicle position before deflation via vehicle state:{}".format(vehicle.state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    plt.imshow(image)
    plt.pause(0.01)
    vehicle.deflate_tires(deflation_pattern)
    bng.step(steps_per_sec * 6)
    vehicle.update_vehicle()
    # print("vehicle position after deflation via beamstate:{}".format(vehicle.get_object_position()))
    # print("vehicle position after deflation via vehicle state:{}".format(vehicle.state))
    pitch = vehicle.state['pitch'][0]
    roll = vehicle.state['roll'][0]
    z = vehicle.state['pos'][2]
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    plt.imshow(image)
    plt.pause(0.01)

    # bng.resume()
    #vehicle.break_all_breakgroups()
    #vehicle.break_hinges()
    #vehicle_loadfile = 'vehicles/etk800/fronttires_0psi.pc'
    #vehicle_loadfile = 'vehicles/etk800/backtires_0psi.pc'
    # vehicle_loadfile = 'vehicles/etk800/chassis_forcefeedback201.pc'
    #vehicle.load_pc(vehicle_loadfile, False)

    wheelspeed = 0.0; throttle = 0.0; prev_error = setpoint; damage_prev = None; runtime = 0.0
    kphs = []; traj = []; pitches = []; rolls = []; steering_inputs = []; throttle_inputs = []; timestamps = []
    damage = None
    final_img = None
    # Send random inputs to vehice and advance the simulation 20 steps
    for _ in range(1024):
        # collect images
        image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
        img = sm.process_image(np.asarray(image))
        steering_prediction = steering_model.predict(img)
        throttle_prediction = throttle_model.predict(img)
        runtime = bng.poll_sensors(vehicle)['timer']['time']

        # control params
        kph = ms_to_kph(wheelspeed)
        brake = 0
        steering = float(steering_prediction[0][0]) #random.uniform(-1.0, 1.0)
        throttle = float(throttle_prediction[0][0])
        if runtime < 10:
            throttle = throttle_PID(kph, dt)
            if throttle > 1:
                throttle = 1
            # if setpoint < kph:
            #     brake = 0.0 #throttle / 10000.0
            #     throttle = 0.0
        vehicle.control(throttle=throttle, steering=steering, brake=brake)

        steering_inputs.append(steering)
        throttle_inputs.append(throttle)
        timestamps.append(runtime)

        steering_state = bng.poll_sensors(vehicle)['electrics']['steering']
        steering_input = bng.poll_sensors(vehicle)['electrics']['steering_input']
        avg_wheel_av = bng.poll_sensors(vehicle)['electrics']['avg_wheel_av']
        wheelspeed = bng.poll_sensors(vehicle)['electrics']['wheelspeed']

        damage = bng.poll_sensors(vehicle)['damage']
        new_damage = diff_damage(damage, damage_prev)
        damage_prev = damage
        vehicle.update_vehicle()
        traj.append(vehicle.state['pos'])
        pitches.append(vehicle.state['pitch'][0])
        rolls.append(vehicle.state['roll'][0])


        kphs.append(ms_to_kph(wheelspeed))
        if new_damage > 0.0:
            final_img = image
            break
        bng.step(1)

        # if distance(spawn_pt['pos'], vehicle.state['pos']) < 5 and sensors['timer']['time'] > 10:
        #     reached_start = True
        #     break

    #     print("runtime:{}".format(round(runtime, 2)))
    # print("time to crash:{}".format(round(runtime, 2)))
    bng.close()
    # avg_kph = float(sum(kphs)) / len(kphs)
    plt.imshow(final_img)
    plt.pause(0.01)
    plot_input(timestamps, steering_inputs, "Steering")
    plot_input(timestamps, throttle_inputs, "Throttle")
    plot_trajectory(traj)
    print("Number of steering_inputs:", len(steering_inputs))
    print("Number of throttle inputs:", len(throttle_inputs))
    results = {'runtime': round(runtime,3), 'damage': damage, 'kphs':kphs, 'traj':traj, 'pitch': round(pitch,3), 'roll':round(roll,3), "z":round(z,3), 'final_img':final_img }

    return results

def get_distance_traveled(traj):
    dist = 0.0
    for i in range(len(traj[:-1])):
        dist += math.sqrt(math.pow(traj[i][0] - traj[i+1][0],2) + math.pow(traj[i][1] - traj[i+1][1],2) + math.pow(traj[i][2] - traj[i+1][2],2))
    return dist

def turn_90(rot_quat):
    r = R.from_quat(list(rot_quat))
    r = r.as_euler('xyz', degrees=True)
    r[2] = r[2] + 90
    r = R.from_euler('xyz', r, degrees=True)
    return tuple(r.as_quat())

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


def main():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline

    deflation_patterns = [[0,0,0,0]
                          # [1,1,1,1],
                          # [0,1,0,1]  # (PASSENGER SIDE  TIRES)
                          #  [1, 0, 1, 0]  # (DRIVER SIDE TIRES)
                          # [1, 0, 0, 1],  # (REAR TIRES)
                          # [0, 1, 1, 0]  # (FRONT TIRES)
                          ]

    vehicles = ['hopper'] #, 'roamer', 'pickup', 'van', 'miramar', 'semi', 'etk800']
    partsconfigs = [None
                    # 'vehicles/etk800/etk800default_15x7front18x9rear.pc'
                    # 'vehicles/hopper/custom.pc' ,
                    # 'vehicles/hopper/classic.pc',
                    # 'vehicles/hopper/crawler.pc',
                    # 'vehicles/hopper/hopperdefault_14x6front19x9rear.pc'
                    # 'vehicles/hopper/hoppercustom_15x9front17x9rear.pc',
                    # 'vehicles/hopper/hoppercrawler_15x7silverrear.pc'
                    ]
    AI = [False]
    all_results = {}
    outputstr = ''
    with open('H:/temp/pitchtraces.txt', 'w') as f:
        for ai in AI:
            for v in vehicles:
                for df in deflation_patterns:
                    for pc in partsconfigs:
                    # centerline = run_scenario_ai_version(vehicle_model='hopper', deflation_pattern=df)
                    # print("centerline:{}".format(centerline))
                        r = 0; a = 0; d = 0
                        grouped_results = {}
                        for i in range(1):
                            if ai:
                                results = run_scenario_ai_version(vehicle_model=v, deflation_pattern=df, parts_config=pc)
                            else:
                                results = run_scenario(vehicle_model=v, deflation_pattern=df, parts_config=pc)

                            key = '{}-{}-{}-AI:{}-{}'.format(v, df, pc, ai, i)
                            f.write("{}:\n".format(key))
                            results['distance'] = get_distance_traveled(results['traj'])
                            f.write("{}\n\n".format(results))
                            all_results[key] = copy.deepcopy(results)
                            grouped_results[i] = copy.deepcopy(results)
                        outputstr = "{}\n\n{}-{}-{}-AI:{}".format(outputstr, v, df, pc, ai)
                        avgd_dist = 0; avgd_runtime = 0; avgd_pitch = 0; avgd_roll = 0; avgd_z = 0; avgd_damage = 0; avgd_damageExt = 0
                        print(grouped_results)
                        for key in grouped_results.keys():
                             for k in grouped_results[key]:
                                 if k == 'distance':
                                     avgd_dist += grouped_results[key][k] / len(grouped_results.keys())
                                 elif k == 'runtime':
                                     avgd_runtime += grouped_results[key][k] / len(grouped_results.keys())
                                 elif k == "pitch":
                                     avgd_pitch += grouped_results[key][k] / len(grouped_results.keys())
                                 elif k == 'z':
                                     avgd_z += grouped_results[key][k] / len(grouped_results.keys())
                                 elif k == 'damage':
                                     avgd_damage += grouped_results[key][k]['damage'] / len(grouped_results.keys())
                                     avgd_damageExt += grouped_results[key][k]['damage'] / len(grouped_results.keys())


                        outputstr = "{}\nAVG DIST:{}\nAVG RUNTIME:{}\nAVG PITCH:{}\nAVG ROLL:{}\nAVG Z:{}\nAVG DAMAGE:{}\nAVG DAMAGE EXT:{}".format(outputstr, avgd_dist, avgd_runtime, math.degrees(avgd_pitch), math.degrees(avgd_roll), avgd_z, avgd_damage, round(avgd_damageExt, 3))
                        # key = '{}-{}-{}-avg'.format(v, df, pc)
                        # all_results[key] = copy.deepcopy()
                        print("OUTPUT STRING SO FAR:{}".format(outputstr))
        print("\n\nFINAL OUTPUT STRING!!!:\n{}".format(outputstr))

        # for k in all_results.keys():
        #     print("{}\n".format(k))
        #     print(all_results[k])
        #     for key in all_results[k]:
        #         if key == 'pitches':
        #             pitches = all_results[k][key]
        #             print('PITCH MEAN:{} STD:{}'.format(float(sum(pitches))/len(pitches), np.std(pitches)))
        #         elif key == 'rolls':
        #             rolls = all_results[k][key]
        #             print('ROLL MEAN:{} STD:{}'.format(float(sum(rolls))/len(rolls), np.std(rolls)))
        #         elif key == 'traj':
        #             zs = [a[2] for a in all_results[k][key]]
        #             print('Z MEAN:{} STD:{}'.format(float(sum(zs))/len(zs), np.std(zs)))
        #         elif key == 'runtime':
        #             print("{} sec to crash".format(all_results[k][key]))
        #     f.write(k)
        #     f.write("{}\n".format(all_results[k]))
            #     r += rt / 10.0
            #     a += av / 10.0
            #     d += d / 10.0
            #     all_kphs = all_kphs + kphs
            #     points.append(copy.deepcopy(traj))
            #     print("RUN {}, runtime:{} avg_kph:{}, std dev kphs:{}".format(i, rt, av, statistics.stdev(kphs)))
            # plot_deviation(points, v, df, centerline)
            # dist_from_centerline = calc_deviation_from_center(centerline, points)
            # print("ALL RUNS for vehicle {} with deflation pattern {}:"
            #       "\n\taverage runtime:{} average kph:{}, std dev kph:{}, average damage:{}, stdev from centerline:{}"
            #       .format(v, df, r, a, statistics.stdev(all_kphs), d, dist_from_centerline))


if __name__ == '__main__':
    main()
