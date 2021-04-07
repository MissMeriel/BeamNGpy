import mmap
import random, math
import sys, time
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
from DAVE2 import Model
import statistics
import json

# globals
default_color = 'White' #'Red'
default_scenario = 'west_coast_usa' #'automation_test_track'
dt = 20
integral = 0.0
prev_error = 0.0
setpoint = 40.0 #50.0 #53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country
lanewidth = 3.75 #2.25
centerline = []

def spawn_point(scenario_locale, spawn_point ='default'):
    global lanewidth
    if scenario_locale == 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale == 'west_coast_usa':
        # industrial area
        #return {'pos': (-717.124, 100.904, 118.558), 'rot': None, 'rot_quat': (0.0013743385206908, 0.0021804112475365, 0.91880744695663, -0.39469763636589)}
        # chinatown
        #return {'pos': (-725.712, 554.327, 120.27), 'rot': None, 'rot_quat': (-0.0061643430963159, 0.0011215945705771, -0.53014314174652, 0.84788501262665)}
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
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 0.75, 0) #(0, 0.75, -1.5) #(0, 0.75, 0) #(0,1,0)
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


def run_spawn(vehicle_model='etk800', deflation_pattern=[0,0,0,0]):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error
    # 080.097 | I | BeamNGVehicle::spawnObject | Spawning
    # object
    # 'hopper'
    # with config: vehicles / hopper / crawler.pc
    # 081.199 | I | GELua.main | Player  # 0 vehicle switched from: nil to: id 8496 (vehicles/hopper/)
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
    bng.set_steps_per_second(100)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # load part config
    #pc = vehicle.get_part_config()
    # loadfile = 'vehicles/hopper/crawler.json'
    # vehicle.load(loadfile)
    # vehicle.update_vehicle()

    bng.spawn_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'], partConfig='vehicles/hopper/semi-test.pc')
    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    vehicle.control(throttle=0.0, steering=0.0, brake=1.0)
    # bng.resume()

    # perturb vehicle
    before_state = copy.deepcopy(vehicle.state)
    print("vehicle position before deflation via beamstate:{}".format(vehicle.get_object_position()))
    print("vehicle position before deflation via vehicle state:{}".format(before_state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    img = m.process_image(image)
    before_prediction = model.predict(img)
    before_state["prediction"] = before_prediction
    plt.imshow(image)
    plt.pause(0.01)
    if deflation_pattern != [0,0,0,0]:
        print("deflating according to pattern {}".format(deflation_pattern))
        vehicle.deflate_tires(deflation_pattern)
        time.sleep(1)
    vehicle.update_vehicle()
    after_state = copy.deepcopy(vehicle.state)
    print("vehicle position after deflation via beamstate:{}".format(vehicle.get_object_position()))
    print("vehicle position after deflation via vehicle state:{}".format(after_state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    img = m.process_image(image)
    after_prediction = model.predict(img)
    after_state["prediction"] = after_prediction
    plt.imshow(image)
    plt.pause(0.01)
    bng.close()
    return before_state, after_state


# returns angle between two vectors in euler angle
def angle_between_vectors(a, b):
    unit_vec_1 = a / np.linalg.norm(a)
    unit_vec_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vec_1, unit_vec_2)
    angle = np.arccos(dot_product)
    return math.degrees(angle)

def distance(a,b):
    return math.sqrt(math.pow(a[0]-b[0],2) + math.pow(a[1]-b[1],2) + math.pow(a[2]-b[2],2))

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
        else:
            diffs[k] = [a_i - b_i for a_i, b_i in zip(after_state[k], before_state[k])]
    return diffs

def get_averages(states):
    dirs = [state['dir'] for state in states]
    ups = [state['up'] for state in states]
    poss = [state['pos'] for state in states]
    fronts = [state['front'] for state in states]
    predictions = [state['prediction'] for state in states]
    avgs = {}
    avgs['dir'] = sum(dirs)/len(dirs)
    avgs['up'] = sum(ups) / len(ups)
    avgs['pos'] = sum(poss) / len(poss)
    avgs['front'] = sum(fronts) / len(fronts)
    avgs['prediction'] = sum(predictions) / len(predictions)
    return avgs


def main():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline
    deflation_patterns = [[0,0,0,0],
                          [1,1,1,1],
                          [0,1,0,1],  # (DRIVERS SIDE TIRES)
                          [1, 0, 1, 0],  # (PASSENGER SIDE TIRES)
                          [1, 0, 0, 1],  # (REAR TIRES)
                          [0, 1, 1, 0]  # (FRONT TIRES)
                          ]
    #1171.208|I|BeamNGVehicle::spawnObject|Spawning object 'hopper' with config: vehicles/hopper/crawler.pc
    vehicles = ['hopper'] #, 'hopper', 'miramar'] #, 'roamer', 'pickup', 'van', 'miramar', 'semi', 'etk800']
    output_string = ""
    for v in vehicles:
        for df in deflation_patterns:
            states = []
            for i in range(10):
                before_state, after_state = run_spawn(v, df)
                diffs = diff_states(before_state, after_state)
                states.append(copy.deepcopy(diffs))
                tmp = "Vehicle {} with deflation pattern {}:\n\tbefore:{}\n\tafter:{}\n\tdiff:{}".format(v, df, before_state, after_state, diffs)
                print(tmp)
            avgs = get_averages(states)
            diffstr = "Vehicle {} with deflation pattern {}:\n\taveraged diff:{}".format(v, df, avgs)
            output_string = "{}\n{}".format(output_string, diffstr)
    print("\n\nOUTPUT FOR ALL RUNS:\n{}".format(output_string))



if __name__ == '__main__':
    main()
