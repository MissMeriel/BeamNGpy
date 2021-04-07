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
        # mid highway scenario (past shadowy parts of road)
        #return {'pos': (-145.775, 211.862, 115.55), 'rot': None, 'rot_quat': (0.0032586499582976, -0.0018308814615011, 0.92652350664139, -0.37621837854385)}
        # actually past shadowy parts of road?
        #return {'pos': (95.1332, 409.858, 117.435), 'rot': None, 'rot_quat': (0.0077012465335429, 0.0036200874019414, 0.90092438459396, -0.43389266729355)}
        # surface road (crashes early af)
        #return {'pos': (945.285, 886.716, 132.061), 'rot': None, 'rot_quat': (-0.043629411607981, 0.021309537813067, 0.98556911945343, 0.16216005384922)}
        # surface road 2
        #return {'pos': (900.016, 959.477, 127.227), 'rot': None, 'rot_quat': (-0.046136282384396, 0.018260028213263, 0.94000166654587, 0.3375423848629)}
        # surface road 3 (start at top of hill)
        #return {'pos':(873.494, 984.636, 125.398), 'rot': None, 'rot_quat':(-0.043183419853449, 2.3034785044729e-05, 0.86842048168182, 0.4939444065094)}
        # surface road 4 (right turn onto surface road) (HAS ACCOMPANYING AI DIRECTION AS ORACLE)
        return {'pos': (956.013, 838.735, 134.014), 'rot': None, 'rot_quat': (0.020984912291169, 0.037122081965208, -0.31912142038345, 0.94675397872925)}
        # surface road 5 (ramp past shady el)
        #return {'pos':(166.287, 812.774, 102.328), 'rot': None, 'rot_quat':(0.0038638345431536, -0.00049926445353776, 0.60924011468887, 0.79297626018524)}
        # entry ramp going opposite way
        return {'pos': (850.136, 946.166, 123.827), 'rot': None, 'rot_quat': (-0.030755277723074, 0.016458060592413, 0.37487033009529, 0.92642092704773)}
        # right after toll
        return {'pos': (-852.024, -517.391 + lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
        #return {'pos':(-717.121, 101, 118.675), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-717.121, 101, 118.675), 'rot': None, 'rot_quat': (0, 0, 0.918812, -0.394696)}
        # racetrack
        return {'pos': (395.125, -247.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
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

def run_scenario_ai_version(vehicle_model='etk800', deflation_pattern=[0,0,0,0]):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error

    random.seed(1703)
    setup_logging()

    # beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model,
                      licence='LOWPRESS', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, 'highway')
    scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(60)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()

    vehicle.ai_set_mode('span')
    vehicle.ai_drive_in_lane(True)
    damage_prev = None
    runtime = 0.0
    traj = []
    for _ in range(650):
        damage = bng.poll_sensors(vehicle)['damage']
        new_damage = diff_damage(damage, damage_prev)
        damage_prev = damage
        vehicle.update_vehicle()
        traj.append(vehicle.state['pos'])
        if new_damage > 0.0:
            break
        bng.step(5)
        runtime += (0.05)
    bng.close()
    return traj

def run_scenario(vehicle_model='etk800', deflation_pattern=[0,0,0,0]):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error
    #vehicle_loadfile = 'vehicles/pickup/pristine.save.json'
    # setup DNN model + weights
    m = Model()
    model = m.define_model_BeamNG("BeamNGmodel-5.h5")

    random.seed(1703)
    setup_logging()

    #beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario(default_scenario, 'research_test')
    vehicle = Vehicle('ego_vehicle', model=vehicle_model,
                      licence='LOWPRESS', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario, 'highway')
    scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=None, rot_quat=spawn['rot_quat'])

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(beamng)

    # Start BeamNG and enter the main loop
    bng = beamng.open(launch=True)

    #bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(100)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

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
    vehicle.update_vehicle()
    print("vehicle position after deflation via beamstate:{}".format(vehicle.get_object_position()))
    print("vehicle position after deflation via vehicle state:{}".format(vehicle.state))
    image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
    plt.imshow(image)
    plt.pause(0.01)

    bng.resume()
    #vehicle.break_all_breakgroups()
    #vehicle.break_hinges()
    #vehicle_loadfile = 'vehicles/etk800/fronttires_0psi.pc'
    #vehicle_loadfile = 'vehicles/etk800/backtires_0psi.pc'
    # vehicle_loadfile = 'vehicles/etk800/chassis_forcefeedback201.pc'
    #vehicle.load_pc(vehicle_loadfile, False)

    wheelspeed = 0.0; throttle = 0.0; prev_error = setpoint; damage_prev = None; runtime = 0.0
    kphs = []; traj = []
    damage = None
    final_img = None
    # Send random inputs to vehice and advance the simulation 20 steps
    for _ in range(1024):
        # collect images
        image = bng.poll_sensors(vehicle)['front_cam']['colour'].convert('RGB')
        img = m.process_image(image)
        prediction = model.predict(img)

        # control params
        kph = ms_to_kph(wheelspeed)
        throttle = throttle_PID(kph, dt)
        brake = 0
        if setpoint < kph:
            brake = 0.0 #throttle / 10000.0
            throttle = 0.0
        steering = float(prediction[0][0]) #random.uniform(-1.0, 1.0)
        vehicle.control(throttle=throttle, steering=steering, brake=brake)

        steering_state = bng.poll_sensors(vehicle)['electrics']['steering']
        steering_input = bng.poll_sensors(vehicle)['electrics']['steering_input']
        avg_wheel_av = bng.poll_sensors(vehicle)['electrics']['avg_wheel_av']
        wheelspeed = bng.poll_sensors(vehicle)['electrics']['wheelspeed']
        damage = bng.poll_sensors(vehicle)['damage']
        new_damage = diff_damage(damage, damage_prev)
        damage_prev = damage
        vehicle.update_vehicle()
        traj.append(vehicle.state['pos'])


        # #print("steering state: {}".format(steering_state))
        # print("AI steering_input: {}".format(steering_input))
        #print("avg_wheel_av: {}".format(avg_wheel_av))
        # print("DAVE2 steering prediction: {}".format(float(prediction[0][0])))
        # print("throttle:{}".format(throttle))
        # print("brake:{}".format(brake))
        # print("kph: {}".format(ms_to_kph(wheelspeed)))
        # print("new_damage:{}".format(new_damage))
        # print("\n")
        kphs.append(ms_to_kph(wheelspeed))
        if new_damage > 0.0:
            final_img = image
            break
        bng.step(5)
        runtime += (0.05)
    #     print("runtime:{}".format(round(runtime, 2)))
    # print("time to crash:{}".format(round(runtime, 2)))
    bng.close()
    avg_kph = float(sum(kphs)) / len(kphs)
    plt.imshow(final_img)
    plt.pause(0.01)
    return runtime, avg_kph, damage['damage'], kphs, traj

def main():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    global centerline
    deflation_patterns = [ [0,0,0,0],
                           [1,1,1,1],
                           [1,0,0,1],
                            [0,1,1,0],
                            [1,0,1,0],
                            [0,1,0,1]]
    vehicles = ['hopper', 'roamer', 'pickup', 'van', 'miramar', 'semi', 'etk800']
    centerline = run_scenario_ai_version()
    print("centerline:{}".format(centerline))
    for v in vehicles:
        for df in deflation_patterns:
            all_kphs = []
            points = []
            r = 0; a = 0; d = 0
            for i in range(10):
                integral = 0.0
                prev_error = 0.0
                rt, av, d, kphs, traj = run_scenario(v, df)
                r += rt / 10.0
                a += av / 10.0
                d += d / 10.0
                all_kphs = all_kphs + kphs
                points.append(copy.deepcopy(traj))
                print("RUN {}, runtime:{} avg_kph:{}, std dev kphs:{}".format(i, rt, av, statistics.stdev(kphs)))
            plot_deviation(points, v, df, centerline)
            dist_from_centerline = calc_deviation_from_center(centerline, points)
            print("ALL RUNS for vehicle {} with deflation pattern {}:"
                  "\n\taverage runtime:{} average kph:{}, std dev kph:{}, average damage:{}, stdev from centerline:{}"
                  .format(v, df, r, a, statistics.stdev(all_kphs), d, dist_from_centerline))


if __name__ == '__main__':
    main()
