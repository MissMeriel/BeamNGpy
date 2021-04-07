"""
.. module:: lidar_tour
    :platform: Windows
    :synopsis: Example starting in west_coast_usa with a vehicle that has a
               Lidar attached and drives around the environment using the
               builtin AI. Lidar data is displayed using the OpenGL-based
               Lidar visualiser.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import sys

from time import sleep


import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer
from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Lidar
from beamngpy.visualiser import LidarVisualiser
from beamngpy.beamngcommon import *
import time
import random, copy
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow

sp = 'highway2'
ai_modes = ['traffic', 'span', 'random']
lanewidth = 3.75 #2.25

#returns lane counted from left to right (1 through 4)
def spawn_point(spawn_point, lane):
    if spawn_point == 'highway': # before tollbooths, this spawnpoint sucks
        return {'pos': (-957.195, -627.665, 106.81), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
    elif spawn_point == 'highway2' and lane == 1:
        #right after toll
        return {'pos': (-852.024 , -517.391+ lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211), 'lane': 1}
    elif spawn_point == 'highway2' and lane == 2:
        #right after toll
        return {'pos': (-852.024, -517.391, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211), 'lane': 2}
    elif spawn_point == 'highway2' and lane == 3:
        #right after toll
        return {'pos': (-852.024 , -517.391- lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211), 'lane': 3}
    elif spawn_point == 'highway2' and lane == 4:
        #right after toll
        return {'pos': (-852.024, -517.391 - 2*lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211), 'lane': 4}
    elif spawn_point == 'highway3':
        #right after bridge -- buggy
        return {'pos': (-721.43,-386.596,106.368), 'rot': None, 'rot_quat': (0.010, 0.002, 0.874, -0.486)}
    elif spawn_point == 'highway4':
        # straightaway
        return {'pos': (-304.921, 40.0301, 111.691), 'rot': None, 'rot_quat': (0.0005, -0.0133, 0.9281, -0.3721)}
    elif spawn_point == 'highway5':
        return {'pos': (-176.193, 176.179, 114.832), 'rot': None, 'rot_quat': (0.00020790436246898, -0.0085823750123382, 0.91742867231369, -0.3978077173233)}
    elif spawn_point == 'racetrack':
        return {'pos': (395.125, -247.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack2':
        return {'pos': (370.125, -252.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack3':
        return {'pos': (350.125, -257.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack4':
        return {'pos': (335.125, -252.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}

def setup_sensors(vehicle):
    # Set up sensors
    #pos = (-0.3, 1, 1.0)
    pos = (-5, 1, 1.0)
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
    #vehicle.attach_sensor('front_cam', front_camera)
    #vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle

def setup_lidar(mountpoint='roof'):
    lidar = Lidar()
    if mountpoint == 'hood':
        #lidar.__init__(offset=(-0.3, 1.0, 1.0), direction=(-0.707, -0.707, 0), vres=2,
        #lidar.__init__(offset=(-5, 0, 1.0), direction=(-0.707, -0.707, 0), vres=2,
        lidar.__init__(offset=(2.4,1,1), direction=(-0.707, -0.707, 0), vres=2,
                   vangle=0.1, rps=10000, hz=60, angle=180, max_dist=200,
                   visualized=True)
    elif mountpoint == 'roof':
        lidar.__init__(offset=(0, 0, 1.7), direction=(-0.707, -0.707, -0.0625), vres=3,
                       vangle=0.1, rps=100000, hz=20, angle=180, max_dist=200,
                       visualized=True)
    elif mountpoint == 'orig':
        lidar.__init__(offset=(0, 0, 1.7), direction=(0, -1, 0), vres=32,
                 vangle=26.9, rps=2200000, hz=20, angle=360, max_dist=200,
                 visualized=True)
    else:
        print("returning lidar")
        return lidar
    return lidar

def traffic_vehicles(length):
    t1 = Vehicle('traffic1', model='etk800', licence='traffic1', color='White')
    t2 = Vehicle('traffic2', model='etk800', licence='traffic2', color='White')
    t3 = Vehicle('traffic3', model='etk800', licence='traffic3', color='White')
    t4 = Vehicle('traffic4', model='etk800', licence='traffic4', color='White')
    t5 = Vehicle('traffic5', model='etk800', licence='traffic5', color='White')
    t6 = Vehicle('traffic6', model='etk800', licence='traffic6', color='White')
    t7 = Vehicle('traffic7', model='etk800', licence='traffic7', color='White')
    t8 = Vehicle('traffic8', model='etk800', licence='traffic7', color='White')
    t9 = Vehicle('traffic9', model='etk800', licence='traffic7', color='White')
    t10 = Vehicle('traffic10', model='etk800', licence='traffic7', color='White')
    tvs = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10]
    return tvs[0:length]

def get_sequence(lane):
    global lanewidth
    if lane == 1:
        return [0, -lanewidth, -2 * lanewidth, -3 * lanewidth]
    if lane == 2:
        return [lanewidth, 0, -lanewidth, -2 * lanewidth]
    elif lane == 3:
        return [2*lanewidth, lanewidth, 0, -lanewidth]
    elif lane == 4:
        return [3*lanewidth, 2*lanewidth, lanewidth, 0]

#return distance between two 3d points
def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def creates_overlap(p, ps, ego_pos):
    pps = copy.deepcopy(ps)
    pps.append(ego_pos)
    for v in pps:
        #if v[0] == p[0] and distance(v, p) <= 5:
        if distance(v, p) <= 5:
            return True
    return False

def get_world_pose(p, ego_pos, rm):
    p = p.transpose()
    p = np.matmul(p, rm)
    p = p + ego_pos
    return p

def generate_vehicle_positions(ego_pos, ego_rot_quat, lane, tvs):
    ps = []
    rm = compute_rotation_matrix(ego_rot_quat)
    print("Ego pos:{}".format(ego_pos))
    sequence = get_sequence(lane)
    for t in range(len(tvs)):
        # choose a lane
        r = random.choice(sequence)
        # choose a distance ahead/behind ego car
        p = np.array([r, -5 + random.uniform(-10, 10), 0])
        p = get_world_pose(p, ego_pos, rm)
        while creates_overlap(p, ps, ego_pos):
            #p = np.array([x, -5 + random.uniform(-3, 1), 0])
            p = np.array([r, -15 + random.uniform(-40, 40), 0])
            p = get_world_pose(p, ego_pos, rm)
        #print("{} pos:{}".format(tvs[t].vid, p))
        ps.append(copy.deepcopy(p))
    print("ps:{}".format(ps))
    return ps

def diff_damage(damage, damage_prev):
    new_damage = 0
    if damage is None or damage_prev is None:
        return 0
    new_damage = damage['damage'] - damage_prev['damage']
    return new_damage


def run_scenario(traffic=2, run_number=0):
    global sp
    setup_logging()
    #beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1')
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering '
                                    'Lidar data')
    # setup vehicle
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR', color='Red')
    vehicle = setup_sensors(vehicle)
    lidar = setup_lidar('')
    vehicle.attach_sensor('lidar', lidar)
    # setup vehicle poses
    lane = random.choice([1,2,3,4])
    print("Lane:{}".format(lane))
    ego_sp = spawn_point(sp, lane)
    ego_pos = ego_sp['pos']
    ego_rot_quat = ego_sp['rot_quat']
    lane = ego_sp['lane']
    # add vehicles to scenario
    print("adding vehicle to scenario...")
    scenario.add_vehicle(vehicle, pos=ego_pos, rot=None, rot_quat=ego_rot_quat)
    tvs = traffic_vehicles(traffic)
    ps = generate_vehicle_positions(ego_pos, ego_rot_quat, lane, tvs)
    for i in range(len(tvs)):
        print("adding vehicle {}...".format(i))
        scenario.add_vehicle(tvs[i], pos=ps[i], rot_quat=ego_rot_quat)
    print("making scenario...")
    scenario.make(beamng)
    print("opening beamNG...")
    bng = beamng.open(launch=True)
    st = time.time()
    # timeout

    crashed = 0
    # try:
    print("loading scenario...")
    bng.load_scenario(scenario)

    bng.set_steps_per_second(60)
    bng.set_deterministic()
    #print("Bounding box: {}".format(vehicle.get_bbox()))
    bng.hide_hud()
    print("starting scenario...")
    bng.start_scenario()
    vehicle.ai_set_mode('traffic')
    #"DecalRoad31765_8"
    vehicle.ai_drive_in_lane(False)
    #vehicle.ai_set_speed(16, mode='limit')
    #vehicle.ai_set_target('traffic')
    #vehicle.ai_set_aggression(1.5)
    #737.385 | I | GELua.core_multiSpawn.multiSpawn | Spawning vehicle group 'traffic' with 3 vehicles 737.391 | I | BeamNGVehicle::spawnObject | Spawning object 'coupe' with config: vehicles / coupe / base_M.pc
    vehicle.ai_set_aggression(2.0)
    bng.start_traffic(tvs)
    bng.switch_vehicle(vehicle)
    bng.pause()
    start = time.time()
    end = time.time()
    damage_prev = None
    filename = 'H:/experiment2/{}_external_vehicles/trace_{}_external_vehicles_run_{}.csv'.format(traffic, traffic, run_number)
    print("filename:{}".format(filename))
    crashed = 0
    #with open('H:/traffic_traces/{}externalvehicles/traffic_lidar_data_{}traffic_run{}.csv'.format(traffic, traffic, run_number), 'w') as f:
    with open(filename,'w') as f:
        f.write("TIMESTAMP,VEHICLE_POSITION,VEHICLE_ORIENTATION,VELOCITY,LIDAR,CRASH,EXTERNAL_VEHICLES\n")
        totalsecs = 0
        while totalsecs <= 45: # and (time.time() - st) < 500:
            sensors = bng.poll_sensors(vehicle)
            points = sensors['lidar']['points']
            damage = sensors['damage']
            #image = sensors['front_cam']['colour'].convert('RGB')
            #imshow(np.asarray(image))
            #plt.pause(0.00001)
            v_state = vehicle.state
            #print("vehicle_state = {}".format(v_state))
            new_damage = diff_damage(damage, damage_prev)
            damage_prev = damage
            if new_damage > 0:
                print("new damage = {}\n".format(new_damage))
                crashed = 1
            f.write("{},{},{},{},{},{},{}\n".format(totalsecs, v_state['pos'], v_state['dir'], v_state['vel'],
                                                 points.tolist(), str(new_damage), traffic))
            #print("Time passed since last step: {}".format(time.time() - end))
            end = time.time()
            #print("Time passed since scenario begun: {}\n".format(end - start))
            #if end - start >= 45:
            #    bng.close()
            bng.step(15, wait=False)
            #bng.step(2)
            totalsecs += 0.25
    # except Exception as e:
    #     print(e)
    # finally:
    bng.close()
    return crashed


def main():
    overallbegin = time.time()
    times = []
    crashes = 0.0
    total_runs = 0.0
    # for t in range(9,10):
    for t in [10]:
        for i in range(194,200):
            begin = time.time()
            crashed = run_scenario(traffic=t, run_number=i)
            tt = time.time() - begin
            times.append(tt)
            crashes += crashed
            total_runs += 1.0
            print("time to run {}_external_vehicles run_{}: {}".format(t, i, tt))
            print("averaged time to run: {}".format(sum(times) / float(len(times))))
            print("total crashes: {}".format(crashes))
            print("crash rate: {}".format(crashes / total_runs))
            print("total runs: {}".format(total_runs))
    print("time to run: {}".format(time.time() - overallbegin))


if __name__ == '__main__':
    main()
