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

from DAVE2 import Model


# globals
default_model = 'etk800' #'pickup' #'etk800'
default_color = 'White' #'Red'
default_scenario = 'automation_test_track'
dt = 20
#base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))
#base_filename = 'G:/{}/{}_{}_'.format(training_dir, default_model, default_scenario.replace("_", ""))
integral = 0.0
prev_error = 0.0
setpoint = 1.5

def spawn_point(scenario_locale):
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
        # starting line
        # return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # 30m down track from starting line
        #return {'pos': (530.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}
        # handling circuit
        #return {'pos': (-294.031, 10.4074, 118.518), 'rot': None, 'rot_quat': (0, 0, 0.708103, 0.706109)}
        # rally track
        #return {'pos': (-374.835, 84.8178, 115.084), 'rot': None, 'rot_quat': (0, 0, 0.718422, 0.695607)}
        # highway (open, farm-like)
        return {'pos': (-294.791, -255.693, 118.703), 'rot': None, 'rot_quat': (0, 0, -0.704635, 0.70957)}
        # default
        #return {'pos': (487.25, 178.73, 131.928), 'rot': None, 'rot_quat': (0, 0, -0.702719, 0.711467)}

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
    return wheelspeed * 6.0 / 100.0

def throttle_PID(kph, dt):
    global integral, prev_error, setpoint
    kp = 1; ki = 0; kd = 0
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

def main():
    global base_filename, default_model, default_color, default_scenario, setpoint
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
    vehicle = Vehicle('ego_vehicle', model=default_model,
                      licence='LOWPRESS', color=default_color)
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario)
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
    #vehicle.ai_set_mode('span')
    #vehicle.ai_drive_in_lane(True)
    #vehicle_loadfile = 'vehicles/etk800/fronttires_0psi.pc'
    # vehicle_loadfile = 'vehicles/etk800/backtires_0psi.pc'
    # vehicle_loadfile = 'vehicles/etk800/chassis_forcefeedback201.pc'
    # vehicle.load_pc(vehicle_loadfile, False)
    vehicle.deflate_tires([1,1,1,1])
    #vehicle.break_all_breakgroups()
    #vehicle.break_hinges()
    # Put simulator in pause awaiting further inputs
    bng.pause()
    assert vehicle.skt
    bng.resume()
    wheelspeed = 0.0; throttle = 0.0; prev_error = setpoint; damage_prev = None; runtime = 0.0
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
        if throttle < 0:
            brake = throttle
            throttle = 0.0
        # throttle = 0.2 # random.uniform(0.0, 1.0)
        # brake = random.choice([0, 0, 0.1 , 0.2])
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
        #print("steering state: {}".format(steering_state))
        print("AI steering_input: {}".format(steering_input))
        #print("avg_wheel_av: {}".format(avg_wheel_av))
        print("DAVE2 steering prediction: {}".format(float(prediction[0][0])))
        print("kph: {}".format(ms_to_kph(wheelspeed)))
        print("new_damage:{}".format(new_damage))
        if new_damage > 50:
            break
        bng.step(5)
        runtime += (0.05)
        print("runtime:{}".format(round(runtime, 2)))
    print("time to crash:{}".format(round(runtime, 2)))
    bng.close()


if __name__ == '__main__':
    main()
