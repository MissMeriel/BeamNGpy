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
import glob

sp = 'highway2'
ai_modes = ['traffic', 'span', 'random']
lanewidth = 3.75 #2.25

def handle_files():
    raw_file_location = "H:/traffic_traces/"
    file_names = glob.glob(raw_file_location + "3externalvehicles/*.csv")
    empty_file_count = 0
    full_file_count = 0
    for file_name in file_names:
        number_of_lines = len(open(file_name).readlines())
        if number_of_lines <= 1:
            empty_file_count += 1
        else:
            full_file_count += 1
            print("Lidar scans in file: " + str(number_of_lines) + " - estimated duration: " + str(number_of_lines / 2.0) + "s")
    print("-----------")
    print("total files: " + str(len(file_names)))
    print("Empty: " + str(empty_file_count))
    print("Full: " + str(full_file_count))

def main():
    #with open('H:/traffic_traces/{}externalvehicles/traffic_lidar_data_{}traffic_run{}.csv'.format(traffic,traffic,run_number),'w') as f:
    #    f.write("TIMESTAMP,VEHICLE_POSITION,VEHICLE_ORIENTATION,VELOCITY,LIDAR,CRASH,EXTERNAL_VEHICLES\n")
    handle_files()

if __name__ == '__main__':
    main()
