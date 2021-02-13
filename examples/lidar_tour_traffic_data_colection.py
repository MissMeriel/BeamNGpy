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
from beamngpy.sensors import Lidar
from beamngpy.visualiser import LidarVisualiser

SIZE = 1024


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


def main():
    setup_logging()

    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    scenario = Scenario('west_coast_usa', 'lidar_tour',
                        description='Tour through the west coast gathering '
                                    'Lidar data')

    vehicle = Vehicle('ego_vehicle', model='etk800', licence='LIDAR')
    lidar = Lidar()
    vehicle.attach_sensor('lidar', lidar)
    ego_pos = (-957.195, -627.665, 106.81)
    ego_rot_quat = (0, 0, 0.926127, -0.377211)
    #scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.add_vehicle(vehicle, pos=ego_pos, rot=None, rot_quat=ego_rot_quat)
    tvs = traffic_vehicles()
    #ps = [(-722.121, 103, 118.675), (-714.121, 101, 118.675), (-715.121, 105, 118.675), (-721.121, 100, 118.675)]
    ps = [(ego_pos[0]+6, ego_pos[1]+3, ego_pos[2]),
          (ego_pos[0]-4, ego_pos[1], ego_pos[2]),
          (ego_pos[0]-3, ego_pos[1]+4, ego_pos[2]),
          (ego_pos[0]+5, ego_pos[1]-1, ego_pos[2])]
    for i in range(len(tvs)):
        scenario.add_vehicle(tvs[i], pos=ps[i], rot_quat=ego_rot_quat)
    scenario.make(beamng)
    bng = beamng.open(launch=True)

    try:
        bng.load_scenario(scenario)
        #bng.set_steps_per_second(60)
        bng.set_deterministic()
        bng.hide_hud()
        bng.start_scenario()
        vehicle.ai_set_mode('random')
        #vehicle.ai_drive_in_lane(True)
        vehicle.ai_set_speed(16, mode='limit')
        #vehicle.ai_set_target('traffic')
        bng.start_traffic(tvs)
        bng.switch_vehicle(vehicle)

        with open('lidar_data.csv', 'w') as f:
            for _ in range(1024):
                sensors = bng.poll_sensors(vehicle)
                points = sensors['lidar']['points']
                print(points.tolist())
                print()

                f.write("{}\n".format(points.tolist()))
                bng.step(3, wait=False)

    except Exception as e:
        print(e)
    finally:
        bng.close()


if __name__ == '__main__':
    main()