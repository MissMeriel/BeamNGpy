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
import time

sp = 'racetrack5'
lanewidth = 3.75

def spawn_point(spawn_point):
    if spawn_point == 'highway':
        return {'pos': (-957.195, -627.665, 106.81), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
    elif spawn_point == 'racetrack':
        return {'pos': (395.125, -247.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack2':
        return {'pos': (370.125, -252.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack3':
        return {'pos': (350.125, -257.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    elif spawn_point == 'racetrack4':
        return {'pos': (335.125, -252.713, 145.67), 'rot': None, 'rot_quat': (0, 0, 0.700608, 0.713546)}
    else:
        return {'pos': (-852.024 , -517.391- lanewidth, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211), 'lane': 3}

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
    t1 = Vehicle('traffic1', model='etk800', licence='traffic1', color='White')
    t2 = Vehicle('traffic2', model='etk800', licence='traffic2', color='White')
    t3 = Vehicle('traffic3', model='etk800', licence='traffic3', color='White')
    t4 = Vehicle('traffic4', model='etk800', licence='traffic4', color='White')
    t5 = Vehicle('traffic5', model='etk800', licence='traffic5', color='White')
    t6 = Vehicle('traffic6', model='etk800', licence='traffic6', color='White')
    t7 = Vehicle('traffic7', model='etk800', licence='traffic7', color='White')
    return [t1, t2, t3] #, t4] #], t5, t6, t7]


def main():
    global sp
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='H:/BeamNG.research.v1.7.0.1clean')
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering '
                                    'Lidar data')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR', color='Red')
    lidar = Lidar()
    lidar.__init__(offset=(0, 0, 1.7), direction=(-0.707, -0.707, 0), vres=32,
                   vangle=0.01, rps=2200000, hz=20, angle=360, max_dist=200,
                   visualized=True)
    vehicle.attach_sensor('lidar', lidar)
    ego_sp = spawn_point(sp)
    ego_pos = ego_sp['pos']
    ego_rot_quat = ego_sp['rot_quat']
    scenario.add_vehicle(vehicle, pos=ego_pos, rot=None, rot_quat=ego_rot_quat)
    tvs = traffic_vehicles()
    #ps = [(-722.121, 103, 118.675), (-714.121, 101, 118.675), (-715.121, 105, 118.675), (-721.121, 100, 118.675)]
    ps = [(ego_pos[0]-10, ego_pos[1]+2, ego_pos[2]),
          (ego_pos[0]-10, ego_pos[1]-2, ego_pos[2]),
          (ego_pos[0]-14, ego_pos[1]+5, ego_pos[2]),
          (ego_pos[0]+5, ego_pos[1]-1, ego_pos[2]),
          (ego_pos[0]+8, ego_pos[1]-4, ego_pos[2]),
          (ego_pos[0]+10, ego_pos[1]+9, ego_pos[2]),
          (ego_pos[0]+1, ego_pos[1]+11, ego_pos[2])]
    ps1 = [(ego_pos[0] - 10, ego_pos[1] + 3, ego_pos[2]),
          (ego_pos[0] - 10, ego_pos[1] - 1, ego_pos[2]),
          (ego_pos[0] - 14, ego_pos[1] + 5, ego_pos[2]),
          (ego_pos[0] + 5, ego_pos[1] - 1, ego_pos[2]),
          (ego_pos[0] + 8, ego_pos[1] - 4, ego_pos[2]),
          (ego_pos[0] + 10, ego_pos[1] + 9, ego_pos[2]),
          (ego_pos[0] + 1, ego_pos[1] + 11, ego_pos[2])]
    ps_oldorig = [(ego_pos[0]-7, ego_pos[1]+3, ego_pos[2]),
          (ego_pos[0]+5, ego_pos[1]-1, ego_pos[2]),
          (ego_pos[0]+4, ego_pos[1]+5, ego_pos[2]),
          (ego_pos[0]+5, ego_pos[1]-1, ego_pos[2]),
          (ego_pos[0]+8, ego_pos[1]-4, ego_pos[2]),
          (ego_pos[0]+10, ego_pos[1]+9, ego_pos[2]),
          (ego_pos[0]+1, ego_pos[1]+11, ego_pos[2])]
    for i in range(len(tvs)):
        scenario.add_vehicle(tvs[i], pos=ps[i], rot_quat=ego_rot_quat)
    scenario.make(beamng)
    bng = beamng.open(launch=True)

    try:
        bng.load_scenario(scenario)
        bng.set_steps_per_second(60)
        bng.set_deterministic()
        bng.hide_hud()
        bng.start_scenario()
        vehicle.ai_set_mode('traffic')
        vehicle.ai_drive_in_lane(True)
        #vehicle.ai_set_speed(16, mode='limit')
        #vehicle.ai_set_target('traffic')
        vehicle.ai_set_aggression(0.5) #0.7)
        bng.start_traffic(tvs)
        bng.switch_vehicle(vehicle)

        start = time.time()
        end = time.time()
        damage_prev = bng.poll_sensors(vehicle)
        with open('lidar_data.csv', 'w') as f:
            for _ in range(1024):
                sensors = bng.poll_sensors(vehicle)
                points = sensors['lidar']['points']
                #print(points.tolist())
                #print()
                #f.write("{}\n".format(points.tolist()))

                print("Time passed since last step: {}".format(time.time() - end))
                end = time.time()
                print("Time passed since scenario begun: {}\n".format(end - start))
                if end - start >= 30:
                    bng.close()

                bng.step(30, wait=False)

    except Exception as e:
        print(e)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
