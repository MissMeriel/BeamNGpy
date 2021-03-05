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
default_model = 'roamer' #'pickup' #'etk800'
default_color = 'White' #'Red'
default_scenario = 'automation_test_track'
dt = 20
#base_filename = '{}/{}/{}_{}_'.format(os.getcwd(), training_dir, default_model, default_scenario.replace("_", ""))
#base_filename = 'G:/{}/{}_{}_'.format(training_dir, default_model, default_scenario.replace("_", ""))
integral = 0.0
prev_error = 0.0
setpoint = 53.3 #https://en.wikipedia.org/wiki/Speed_limits_by_country

class BeamNGpycommon:

    def __init__(self):
        return

    def spawn_point(self, scenario_locale, spawn_point ='default'):
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




    def setup_sensors(self, vehicle):
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

    def ms_to_kph(self, wheelspeed):
        return wheelspeed * 3.6

    def throttle_PID(self, kph, dt):
        global integral, prev_error, setpoint
        kp = 1; ki = 0.1; kd = 0.33
        error = setpoint - kph
        deriv = (error - prev_error) / dt
        integral = integral + error * dt
        w = kp * error + ki * integral + kd * deriv
        prev_error = error
        return w

    def diff_damage(self, damage, damage_prev):
        new_damage = 0
        if damage is None or damage_prev is None:
            return 0
        new_damage = damage['damage'] - damage_prev['damage']
        return new_damage


    def plot_deviation(self):

        return


    def run_scenario(self):
        global base_filename, default_model, default_color, default_scenario, setpoint
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
        vehicle = Vehicle('ego_vehicle', model=default_model,
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
            #if throttle < 0:
            if setpoint < kph:
                brake = throttle / 1000.0
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
