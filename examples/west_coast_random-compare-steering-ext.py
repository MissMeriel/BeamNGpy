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

from DAVE2 import Model
import statistics

# globals
default_color = 'White' #'Red'
default_scenario = 'west_coast_usa' #'automation_test_track'
dt = 20
integral = 0.0
prev_error = 0.0
setpoint = 50.0 #53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country

def spawn_point(scenario_locale, spawn_point ='default'):
    if scenario_locale == 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale == 'west_coast_usa':
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
    kp = 1; ki = 0.7; kd = 0.33
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
def plot_deviation():

    return


def run_scenario(vehicle_model='etk800', deflation_pattern=[0,0,0,0]):
    global base_filename, default_color, default_scenario, setpoint
    global prev_error
    #vehicle_loadfile = 'vehicles/pickup/pristine.save.json'
    # setup DNN model + weights
    m = Model()
    model = m.define_model_BeamNG("BeamNGmodel-4.h5")

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

    bng.hide_hud()
    bng.set_deterministic()  # Set simulator to be deterministic
    bng.set_steps_per_second(100)  # With 60hz temporal resolution

    # Load and start the scenario
    bng.load_scenario(scenario)
    bng.start_scenario()

    # perturb vehicle
    vehicle.deflate_tires(deflation_pattern)
    #vehicle.break_all_breakgroups()
    #vehicle.break_hinges()
    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()
    wheelspeed = 0.0; throttle = 0.0; prev_error = setpoint; damage_prev = None; runtime = 0.0
    kphs = []
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
            brake = throttle / 1000.0
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

        print("\n")
        # #print("steering state: {}".format(steering_state))
        # print("AI steering_input: {}".format(steering_input))
        #print("avg_wheel_av: {}".format(avg_wheel_av))
        # print("DAVE2 steering prediction: {}".format(float(prediction[0][0])))
        print("throttle:{}".format(throttle))
        print("brake:{}".format(brake))
        print("kph: {}".format(ms_to_kph(wheelspeed)))
        print("new_damage:{}".format(new_damage))
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
    return runtime, avg_kph, damage['damage'], kphs

def main():
    global base_filename, default_color, default_scenario, setpoint, integral
    global prev_error
    r = 0
    a = 0
    d = 0
    deflation_patterns = [[0,0,0,0], [1,1,1,1]]
    vehicles = ['etk800'] # ['etk800', 'roamer', 'pickup', 'van', 'hopper', 'miramar', 'semi']
    for v in vehicles:
        for df in deflation_patterns:
            all_kphs = []
            for i in range(10):
                integral = 0.0
                prev_error = 0.0
                rt, av, d, kphs = run_scenario(v, df)
                r += rt / 10.0
                a += av / 10.0
                d += d / 10.0
                all_kphs = all_kphs + kphs
                print("RUN {}, runtime:{} avg_kph:{}, std dev kphs:{}".format(i, rt, av, statistics.stdev(kphs)))
                plot_deviation()
            print("ALL RUNS for vehicle {} with deflation pattern {}:\n\taverage runtime:{} average kph:{}, std dev kph:{}, average damage:{}".format(v, df, r, a, statistics.stdev(all_kphs), d))


if __name__ == '__main__':
    main()
