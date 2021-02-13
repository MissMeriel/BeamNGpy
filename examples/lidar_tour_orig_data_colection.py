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
    lidar.__init__(offset=(0, 0, 1.7), direction=(0, -1, 0), vres=32,
                 vangle=2, rps=2200000, hz=20, angle=360, max_dist=200,
                 visualized=True)
    vehicle.attach_sensor('lidar', lidar)
    ego_pos = (-957.195, -627.665, 106.81)
    ego_rot_quat = (0, 0, 0.926127, -0.377211)
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    #scenario.add_vehicle(vehicle, pos=ego_pos, rot=None, rot_quat=ego_rot_quat)
    tvs = traffic_vehicles()
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
        bng.switch_vehicle(vehicle)

        with open('lidar_data.csv', 'w') as f:
            f.write("TIMESTAMP,VEHICLE_POS,LIDAR\n")
            for _ in range(1024):
                sensors = bng.poll_sensors(vehicle)
                points = sensors['lidar']['points']
                v_state = vehicle.state
                print("vehicle_state = {}".format(v_state))
                print()

                f.write("{},{},{}\n".format(_ * 0.5, v_state['pos'], points.tolist()))
                bng.step(3, wait=False)

    except Exception as e:
        print(e)
    finally:
        bng.close()


if __name__ == '__main__':
    main()