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

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer
from beamngpy.sensors import Lidar
from beamngpy.visualiser import LidarVisualiser
import time
import numpy as np
import cv2
from mss import mss
from PIL import Image
from beamngpy.beamngcommon import *

bounding_box = {'top': 100, 'left': 0, 'width': 400, 'height': 300}
sct = mss()
SIZE = 1024
sp = 'highway4'

def spawn_point():
    global sp
    spawn_point = sp
    if spawn_point == 'highway': # before tollbooths, this spawnpoint sucks
        return {'pos': (-957.195, -627.665, 106.81), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
    elif spawn_point == 'highway2':
        #right after toll
        return {'pos': (-851.024, -517.391, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
    elif spawn_point == 'highway3':
        #right after bridge -- buggy
        p = (-721.43,-386.596,106.368) #(-797.11, -463.62, 106.65)
        rq = (0.010, 0.002, 0.874, -0.486)
        return {'pos': p, 'rot': None, 'rot_quat': rq}
    elif spawn_point == 'highway4':
        # straightaway
        rq = (0.00049602653598413, -0.013275870122015, 0.92808425426483, -0.37213319540024)
        return {'pos': (-304.9208984375, 40.03009796142578, 111.69083404541016), 'rot': None, 'rot_quat': rq}
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

def screenrecord():
    sct_img = sct.grab(bounding_box)
    cv2.imshow('screen', np.array(sct_img))

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        cv2.destroyAllWindows()

def lidar_resize(width, height):
    if height == 0:
        height = 1

    glViewport(0, 0, width, height)

def open_window(width, height):
    glutInit()
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE)
    glutInitWindowSize(width, height)
    window = glutCreateWindow(b'Lidar Tour')
    lidar_resize(width, height)
    return window

def traffic_vehicles():
    t1 = Vehicle('traffic1', model='etk800')
    t2 = Vehicle('traffic2', model='etk800')
    t3 = Vehicle('traffic3', model='etk800')
    t4 = Vehicle('traffic4', model='etk800')
    return [t1, t2, t3, t4]

def setup_sensors(vehicle):
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 0.75, 1.5) #(0, 0.75, -1.5) #(0, 0.75, 0) #(0,1,0)
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

def setup_lidar(mountpoint='roof'):
    lidar = Lidar()
    if mountpoint== 'hood':
        lidar.__init__(offset=(-0.4, 1, 1.0), direction=(-0.707, -0.707, 0), vres=2,
                   vangle=0.1, rps=10000, hz=20, angle=180, max_dist=200,
                   visualized=True)
    else:
        lidar.__init__(offset=(0, 0, 1.7), direction=(-0.707, -0.707, 0), vres=2,
                           vangle=0.1, rps=10000, hz=20, angle=180, max_dist=200,
                           visualized=True)
    return lidar

def diff_damage(damage, damage_prev):
    new_damage = 0
    if damage is None or damage_prev is None:
        return 0
    #print(damage.keys())
    #for k in damage.keys():
    #    print(k)
    #print("damage = {}".format(damage))
    new_damage = damage['damage'] - damage_prev['damage']
    return new_damage

def main():
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering Lidar data')
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR')
    vehicle = setup_sensors(vehicle)
    lidar = setup_lidar('hood')
    vehicle.attach_sensor('lidar', lidar)
    spawn_pt = spawn_point()
    ego_pos = spawn_pt['pos']
    ego_rot_quat = spawn_pt['rot_quat']
    scenario.add_vehicle(vehicle, pos=ego_pos, rot=None, rot_quat=ego_rot_quat)
    tvs = traffic_vehicles()
    scenario.make(beamng)
    bng = beamng.open(launch=True)

    #try:
    bng.load_scenario(scenario)
    bng.set_steps_per_second(60)
    bng.set_deterministic()
    bng.hide_hud()
    bng.start_scenario()
    vehicle.ai_set_mode('traffic')
    vehicle.ai_set_aggression(0.5)
    vehicle.ai_drive_in_lane(True)
    #vehicle.ai_set_speed(16, mode='limit')
    #vehicle.ai_set_target('traffic')
    bng.switch_vehicle(vehicle)
    damage_prev = None
    start = time.time()
    end = time.time()
    print("Bounding box: {}".format(vehicle.get_bbox()))
    with open('lidar_data.csv', 'w') as f:
        f.write("TIMESTAMP,VEHICLE_POSITION,VEHICLE_ORIENTATION,VELOCITY,LIDAR,CRASH\n")
        for _ in range(1024):

            sensors = bng.poll_sensors(vehicle)
            points = sensors['lidar']['points']
            damage = sensors['damage']
            v_state = vehicle.state
            print("vehicle_state = {}".format(v_state))
            print("vehicle_state[pos] = {}".format(v_state['pos']))
            print("vehicle_state[dir] = {}".format(v_state['dir']))
            print("Vehicle b  ounding box:{}".format(vehicle.get_bbox()))
            #ai_state = vehicle.ai_get_state()
            #print("ai_state = {}".format(ai_state))
            new_damage = diff_damage(damage, damage_prev)
            damage_prev = damage
            #print("new damage = {}".format(new_damage))
            print()

            f.write("{},{},{},{},{},{}\n".format(_ * 0.5, v_state['pos'], v_state['dir'], v_state['vel'], points.tolist(), str(new_damage)))
            #bng.step(30, wait=False)
            bng.step(30)
            #print("Time passed since last step: {}".format(time.time() - end))
            end = time.time()
            #print("Time passed since scenario begun: {}\n".format(end - start))

            #screenrecord()

            if end - start >= 30:
                #bng.close()
                continue
    #except Exception as e:
    #    print(e)
    #finally:
    #    bng.close()


if __name__ == '__main__':
    main()