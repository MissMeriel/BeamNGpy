import mmap
import random, math, datetime
import sys, time
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow
from shutil import copyfile

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from PIL import Image
import PIL
import cv2
import scipy.misc
import copy
from DAVE2 import Model
import statistics
import json
from scipy.spatial.transform import Rotation as R
from skimage.measure import compare_ssim
from pyquaternion import Quaternion

# globals
default_color = 'White' #'Red'
default_scenario = 'west_coast_usa' #'automation_test_track'
dt = 20
integral = 0.0
prev_error = 0.0
setpoint = 40.0 #50.0 #53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country
lanewidth = 3.75 #2.25
centerline = []
fcam = None

def spawn_point(scenario_locale, spawn_point ='default'):
    global lanewidth
    if scenario_locale == 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale == 'west_coast_usa':
        # industrial area
        #return {'pos': (-717.124, 100.904, 118.558), 'rot': None, 'rot_quat': (0.0013743385206908, 0.0021804112475365, 0.91880744695663, -0.39469763636589)}
        # chinatown
        return {'pos': (-725.712, 554.327, 120.27), 'rot': None, 'rot_quat': (-0.0061643430963159, 0.0011215945705771, -0.53014314174652, 0.84788501262665)}
        # racetrack
        #return {'pos': (395.125, -247.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
        # mid highway 1 scenario (past shadowy parts of road)
        #return {'pos': (-145.775, 211.862, 115.55), 'rot': None, 'rot_quat': (0.0032586499582976, -0.0018308814615011, 0.92652350664139, -0.37621837854385)}
        # mid highway 2 (actually past shadowy parts of road?)
        return {'pos': (95.1332, 409.858, 117.435), 'rot': None, 'rot_quat': (0.0077012465335429, 0.0036200874019414, 0.90092438459396, -0.43389266729355)}
        # surface road 1 (crashes early af)
        return {'pos': (945.285, 886.716, 132.061), 'rot': None, 'rot_quat': (-0.043629411607981, 0.021309537813067, 0.98556911945343, 0.16216005384922)}
        # surface road 2
        #return {'pos': (900.016, 959.477, 127.227), 'rot': None, 'rot_quat': (-0.046136282384396, 0.018260028213263, 0.94000166654587, 0.3375423848629)}
        # surface road 3 (start at top of hill)
        #return {'pos':(873.494, 984.636, 125.398), 'rot': None, 'rot_quat':(-0.043183419853449, 2.3034785044729e-05, 0.86842048168182, 0.4939444065094)}
        # surface road 4 (right turn onto surface road) (HAS ACCOMPANYING AI DIRECTION AS ORACLE)
        return {'pos': (956.013, 838.735, 134.014), 'rot': None, 'rot_quat': (0.020984912291169, 0.037122081965208, -0.31912142038345, 0.94675397872925)}
        # surface road 5 (ramp past shady el)
        #return {'pos':(166.287, 812.774, 102.328), 'rot': None, 'rot_quat':(0.0038638345431536, -0.00049926445353776, 0.60924011468887, 0.79297626018524)}
        # highway #1 entry ramp going opposite way
        #return {'pos': (850.136, 946.166, 123.827), 'rot': None, 'rot_quat': (-0.030755277723074, 0.016458060592413, 0.37487033009529, 0.92642092704773)}
        # highway #2 (right after toll)
        return {'pos': (-852.024, -517.391 + lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
        #return {'pos':(-717.121, 101, 118.675), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-717.121, 101, 118.675), 'rot': None, 'rot_quat': (0, 0, 0.918812, -0.394696)}
    elif scenario_locale == 'smallgrid':
        return {'pos':(0.0, 0.0, 0.0), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale == 'automation_test_track':
        if spawn_point == 'starting line':
            # starting line
            return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        elif spawn_point == 'starting line 30m down':
            # 30m down track from starting line
            return {'pos': (530.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        elif spawn_point == 'handling circuit':
            # handling circuit
            return {'pos': (-294.031, 10.4074, 118.518), 'rot': None, 'rot_quat': (0, 0, 0.708103, 0.706109)}
        elif spawn_point == 'rally track':
            # rally track
            return {'pos': (-374.835, 84.8178, 115.084), 'rot': None, 'rot_quat': (0, 0, 0.718422, 0.695607)}
        elif spawn_point == 'highway':
            # highway (open, farm-like)
            return {'pos': (-294.791, -255.693, 118.703), 'rot': None, 'rot_quat': (0, 0, -0.704635, 0.70957)}
        elif spawn_point == 'default':
            # default
            return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}

def setup_sensors(vehicle):
    global fcam
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 0.75, 0) #(0, 0.75, -1.5) #(0, 0.75, 0) #(0,1,0)
    fov = 120
    resolution = (512, 512)
    front_camera = Camera(pos, direction, fov, resolution,
                          colour=True, depth=True, annotation=True)
    fcam = front_camera

    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    #lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle

def ms_to_kph(wheelspeed):
    return wheelspeed * 3.6

def throttle_PID(kph, dt):
    global integral, prev_error, setpoint
    kp = 1; ki = 0.3; kd = 0.5
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

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm

def quatFromDir(dir, up=[0, 0, 1]):
    dirNorm = normalize(dir)
    i = np.cross(dirNorm, up)
    i = normalize(i)
    k = np.cross(i, dirNorm)
    k = normalize(k)
    k = [[i[0], dirNorm[0], k[0]], [i[1], dirNorm[1], k[1]], [i[2], dirNorm[2], k[2]]]
    rotation = R.from_matrix(k)
    return rotation.as_quat()

# get world camera pos,orientation from relative pos,orientation
def get_camera_rot_and_pos(camera_pos, camera_dir, vehicle_pos, vehicle_dir, vehicle_up):
    orientation = vehicle_dir
    up = vehicle_up
    orientation = quatFromDir(orientation, up)
    print("orientation:{}".format(orientation))
    offset = vehicle_pos

    direction = camera_dir

    rot = quatFromDir(direction, [0, 0, 1]) * orientation

    pos = camera_pos
    pos = offset + orientation * pos

    return pos, rot

def run_spawn_for_deflation(vehicle_model='etk800', deflation_pattern=[0,0,0,0]):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error
    global fcam

    vehicle_loadfile = 'vehicles/hopper/crawler.pc'
    # setup DNN model + weights
    m = Model()
    model = m.define_model_BeamNG("BeamNGmodel-5.h5")

    random.seed(1703)
    setup_logging()

    # beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model,
                      licence='LOWPRESS', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, 'highway')
    # scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    # bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # load part config
    #pc = vehicle.get_part_config()
    # loadfile = 'C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/front17x8rear17x9.json'
    # vehicle.load(loadfile)

    bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'], partConfig='vehicles/hopper/custom.pc')
    #bng.set_relative_camera()

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    # vehicle.control(throttle=0.0, steering=0.0, brake=1.0)

    # perturb vehicle
    # before_state = copy.deepcopy(vehicle.state)
    before_state = bng.poll_sensors(vehicle)['front_cam']
    before_state['vel'] = vehicle.state['vel']
    before_state['roll'] = vehicle.state['roll']
    before_state['pitch'] = vehicle.state['pitch']
    before_state['yaw'] = vehicle.state['yaw']
    # print("vehicle position before deflation via beamstate:{}".format(vehicle.get_object_position()))
    # print("vehicle position before deflation via vehicle state:{}".format(vehicle.state))
    print("vehicle position before deflation via camera:{}".format(before_state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    # print("camera sensor before deflation: {}".format(bng.poll_sensors(vehicle)['front_cam']))
    #get_camera_rot_and_pos([-0.3, 1, 1.0], [0, 0.75, 0], before_state['pos'], before_state['dir'], before_state['up'])
    img = m.process_image(image)
    before_prediction = model.predict(img)
    before_state["prediction"] = before_prediction
    plt.imshow(image)
    plt.pause(0.01)
    if deflation_pattern != [0,0,0,0]:
        print("deflating according to pattern {}".format(deflation_pattern))
        vehicle.deflate_tires(deflation_pattern)
        time.sleep(1)
    bng.resume()
    bng.set_steps_per_second(100)
    bng.set_deterministic()
    totalsecs = 0.0
    deflation_traj = []
    while totalsecs <= 30:
        vehicle.update_vehicle()
        # vehicle.control(throttle=0.0, steering=0.0, brake=1.0)
        after_state = bng.poll_sensors(vehicle)['front_cam']
        after_state['vel'] = vehicle.state['vel']
        after_state['roll'] = vehicle.state['roll']
        after_state['pitch'] = vehicle.state['pitch']
        after_state['yaw'] = vehicle.state['yaw']
        print("roll:{}, pitch:{}, yaw:{}".format(vehicle.state['roll'], vehicle.state['pitch'], vehicle.state['yaw']))
        deflation_traj.append(round(math.degrees(vehicle.state['pitch'][0]), 4))
        bng.step(100)
        totalsecs += 1
    # after_state = copy.deepcopy(vehicle.state)
    # print("vehicle position after deflation via beamstate:{}".format(vehicle.get_object_position()))
    # print("vehicle position after deflation via vehicle state:{}".format(vehicle.state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    print("vehicle position after deflation via camera:{}".format(after_state))
    #print("camera sensor output: {}".format(bng.poll_sensors(vehicle)['front_cam']['rot']))
    #print("camera pos output: {}".format(bng.poll_sensors(vehicle)['front_cam']['pos']))
    # print("camera rot output: {}".format(bng.poll_sensors(vehicle)['front_cam']['direction']))
    # print("fcam.encode_engine_request() = {}".format(fcam.encode_engine_request()))
    damages = bng.poll_sensors(vehicle)['damage']['deform_group_damage']
    d = ["{}={}".format(k, damages[k]['damage']) for k in damages.keys()]
    print("vehicle pressure after deflation: lowpressure={} damages:".format(bng.poll_sensors(vehicle)['damage']['lowpressure'], d))
    img = m.process_image(image)
    after_state["prediction"] = model.predict(img)
    plt.imshow(image)
    plt.pause(0.01)
    bng.close()
    return before_state, after_state, deflation_traj

def run_spawn_for_parts_config(vehicle_model='etk800', loadfile='C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/front17x8rear17x9.json'):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error
    global fcam

    # setup DNN model + weights
    m = Model()
    model = m.define_model_BeamNG("BeamNGmodel-5.h5")

    random.seed(1703)
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model,
                      licence='LOWPRESS', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, 'highway')
    # scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    # bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(100)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # vehicle.load('C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/pristine.json')
    if loadfile:
        bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'],
                          partConfig=loadfile)
    else:
        bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'], partConfig='vehicles/hopper/custom.pc')
    #bng.set_relative_camera()

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()
    bng.set_steps_per_second(100)
    bng.set_deterministic()
    totalsecs = 0.0
    pitch_traj = []
    while totalsecs <= 30:
        vehicle.update_vehicle()
        camera_state = bng.poll_sensors(vehicle)['front_cam']
        camera_state['roll'] = vehicle.state['roll']
        camera_state['pitch'] = vehicle.state['pitch']
        camera_state['yaw'] = vehicle.state['yaw']
        pitch_traj.append(round(math.degrees(vehicle.state['pitch'][0]), 4))
        print("roll:{}, pitch:{}, yaw:{}".format(vehicle.state['roll'], vehicle.state['pitch'], vehicle.state['yaw']))
        bng.step(100)
        totalsecs += 1
    # print("Camera state:{}".format(camera_state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    #get_camera_rot_and_pos([-0.3, 1, 1.0], [0, 0.75, 0], before_state['pos'], before_state['dir'], before_state['up'])
    img = m.process_image(image)
    before_prediction = model.predict(img)
    camera_state["prediction"] = before_prediction
    plt.imshow(image)
    plt.pause(0.01)
    bng.close()
    return camera_state, vehicle.state, pitch_traj

# returns angle between two vectors in euler angle
def angle_between_vectors(a, b):
    unit_vec_1 = a / np.linalg.norm(a)
    unit_vec_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vec_1, unit_vec_2)
    angle = np.arccos(dot_product)
    return math.degrees(angle)

def distance(a,b):
    return math.sqrt(math.pow(a[0]-b[0],2) + math.pow(a[1]-b[1],2) + math.pow(a[2]-b[2],2))

def distance2(a):
    return math.sqrt(math.pow(a[0],2) + math.pow(a[1],2) + math.pow(a[2],2))

def diff_images(im1, im2):
    gray1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(gray1, gray2, full=True)
    # print("SSIM: {}".format(score))
    difference = cv2.absdiff(gray1, gray2)
    num_diff = np.sum(difference)
    # print("Total pixels in image: {}".format(im1.size))
    # print("number of different (gray) pixels: {}".format(num_diff))
    difference = cv2.absdiff(im1, im2)
    num_diff = np.sum(difference)
    # print("number of different (color) pixels: {}".format(num_diff))
    return float(num_diff) / float(im1.size)

# def angle_between_quaternions(Q1, Q2, a ,b):
#     X1 = Q1 * X * conj(Q1);
#     Y1 = Q1 * Y * conj(Q1);
#     Z1 = Q1 * Z * conj(Q1);
#
#     X2 = Q2 * X * conj(Q2);
#     Y2 = Q2 * Y * conj(Q2);
#     Z2 = Q2 * Z * conj(Q2);
#
#     DiffAngleX = acos(dot(X1, X2));
#     DiffAngleY = acos(dot(Y1, Y2));
#     DiffAngleZ = acos(dot(Z1, Z2));
#     return

# xyz axes rotations in euler angles
def get_euler_angles(q):
    angles = [0,0,0]

    # roll
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    if (abs(sinp) >= 1):
        angles[1] = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        angles[1] = math.asin(sinp)

    #yaw
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[3] * q[3])
    angles[2] = math.atan2(siny_cosp, cosy_cosp)
    return angles

# keys: 'vel' 'pos' 'dir' 'front' 'up' 'prediction'
def diff_states(before_state, after_state):
    diffs = {}
    for k in before_state.keys():
        if k == 'dir' or k == 'up':
            diffs[k] = angle_between_vectors(after_state[k], before_state[k])
        elif k == 'pos' or k == 'front':
            diffs[k] = distance(before_state[k], after_state[k])
        elif k == 'prediction':
            diffs[k] = after_state[k][0][0] - before_state[k][0][0]
        elif k == 'colour' or  k == 'annotation':
            print(k, after_state[k], before_state[k])
            diffs[k] = diff_images(np.array(after_state[k]), np.array(before_state[k]))
        elif k == 'rotation':
            # q_before = Quaternion(before_state[k])
            # q_after = Quaternion(after_state[k])
            # z = q_before.conjugate * q_after
            # theta = 2 * math.acos(z[3])
            # diffs[k] = theta

            before_pitch = get_euler_angles(before_state[k])
            after_pitch = get_euler_angles(after_state[k])
            diffs[k] = [a_i - b_i for a_i, b_i in zip(after_pitch, before_pitch)]

            # continue
        elif k == 'depth':
            continue
        else:
            print(k, after_state[k], before_state[k])
            diffs[k] = [a_i - b_i for a_i, b_i in zip(after_state[k], before_state[k])]
    return diffs

def get_averages(states):
    dirs = [state['dir'] for state in states]
    ups = [state['up'] for state in states]
    poss = [state['pos'] for state in states]
    fronts = [state['front'] for state in states]
    predictions = [state['prediction'] for state in states]
    avgs = {}
    avgs['dir'] = sum(dirs) / float(len(dirs))
    avgs['up'] = sum(ups) / float(len(ups))
    avgs['pos'] = sum(poss) / float(len(poss))
    avgs['front'] = sum(fronts) / float(len(fronts))
    avgs['prediction'] = sum(predictions) / float(len(predictions))
    return avgs

def get_cam_averages(states):
    print(states)
    poss = [distance2(state['position']) for state in states]
    print("poss", poss)
    pitches = [math.degrees(state['pitch'][0]) for state in states]
    predictions = [state['prediction'] for state in states]
    avgs = {}
    std_devs = {}
    avgs['position'] = sum(poss) / float(len(poss))
    avgs['pitch'] = sum(pitches) / float(len(pitches))
    std_devs['pitch'] = np.std(pitches)
    avgs['prediction'] = sum(predictions) / float(len(predictions))
    return avgs

def deflation_test():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline
    deflation_patterns = [[0,0,0,0],
                          [1,1,1,1],
                          # [0,1,0,1],  # (DRIVERS SIDE TIRES)
                          # [1, 0, 1, 0],  # (PASSENGER SIDE TIRES)
                          # [1, 0, 0, 1],  # (REAR TIRES)
                          [0, 1, 1, 0]  # (FRONT TIRES)
                          ]
    vehicles = ['hopper'] #, 'hopper', 'miramar'] #, 'roamer', 'pickup', 'van', 'miramar', 'semi', 'etk800']
    output_string = ""
    for v in vehicles:
        for df in deflation_patterns:
            states = []
            for i in range(1):
                before_state, after_state, deflation_traj = run_spawn_for_deflation(v, df)
                diffs = diff_states(before_state, after_state)
                states.append(copy.deepcopy(diffs))
                deftraj = ["{:.4f}".format(a) for a in deflation_traj]
                tmp = "Vehicle {} with deflation pattern {}:\n\tbefore:{}\n\tafter:{}\n\tdeflation_traj:{}\n\tdiff:{}".format(v, df, before_state, after_state, deftraj, diffs)
                print(tmp)
                "{}\n{}".format(output_string, tmp)
            avgs = get_cam_averages(states)
            diffstr = "{}\n\taveraged diff:{}".format(tmp, avgs)
            output_string = "{}\n{}\n\n".format(output_string, diffstr)
            # src = "C:/Users/merie/Documents/BeamNG.research/beamng.log"
            # dst = "{}.{}".format(src, df)
            # copyfile(src, dst)
            # avgs = get_cam_averages(states)
    return output_string

def parts_config_test():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline
    json_files = [
                    # 'C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/front17x8rear17x9.json',
                    # 'C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/front16x7rear17x9.json'
                # 'vehicles/hopper/17x8front17x9rear.pc',
                'vehicles/hopper/16x7front17x9rear.pc'
                # 'vehicles/hopper/15x9front17x9rear.pc'
        # 'C:/Users/merie/Documents/BeamNG.research/vehicles/hopper/5.5Lengine.pc'
                  ]
    vehicles = ['hopper'] #, 'hopper', 'miramar'] #, 'roamer', 'pickup', 'van', 'miramar', 'semi', 'etk800']
    output_string = ""
    for v in vehicles:
        orig_state, vehicle_state, pitch_traj = run_spawn_for_parts_config(v, 'vehicles/hopper/custom.pc')
        output_string = "Vehicle {} with parts config {}:\n\tcamera_state:{} \n\tvehicle_state:{} \n\tpitch trajectory:{} \n\tcamera diff:{}".format(v, None, orig_state, vehicle_state, pitch_traj, None)
        states = [orig_state]
        for f in json_files:
            for i in range(3):
                camera_state, vehicle_state, pitch_traj = run_spawn_for_parts_config(v, f)
                states.append(copy.deepcopy(camera_state))
                print("state[0]:{}\nstate:{}".format(orig_state, camera_state))
                print(states)
                diffs = diff_states(orig_state, camera_state)
                deftraj = ["{:.4f}".format(a) for a in pitch_traj]
                tmp = "Vehicle {} with parts config {}:\n\tcamera_state:{} \n\tvehicle_state:{} \n\tpitch trajectory:{}\n\tcamera diff:{}".format(v, f, camera_state, vehicle_state, pitch_traj, diffs)
                print(tmp)
                output_string = "{}\n{}".format(output_string, tmp)
            # avgs = get_cam_averages(states)
    return output_string

def main():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline
    # output_string = parts_config_test()
    output_string = deflation_test()
    print("\n\nOUTPUT FOR ALL RUNS:\n{}".format(output_string))
    writefilename = 'H:/vehicle_perturbation_analysis/run{}'.format(str(datetime.datetime.now()).replace(" ", "_"))
    filename = "H:/temp_mmap/run{}.txt".format(str(datetime.datetime.now()).replace(" ", "_").replace(":", "-").replace(".","-"))
    with open(filename, 'w') as file:
        file.write(output_string)


if __name__ == '__main__':
    main()
