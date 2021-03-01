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


def diff_damage(damage, damage_prev):
    new_damage = 0
    if damage is None or damage_prev is None:
        return 0
    new_damage = damage['damage'] - damage_prev['damage']
    return new_damage


def run_scenario():
    directory = "H:/experiment2/"
    # set True if counting crashes
    # Counting crashes is sloowwww
    counting_crashes = True
    dirs = os.listdir(directory)
    print("dirs:{}".format(dirs))
    crashes = 0
    total_runs = 0
    for d in dirs:
        files = os.listdir("{}{}".format(directory, d))
        dir_crashes = 0
        for filename in files:
            filename = "{}{}/{}".format(directory, d, filename)
            total_runs += 1
            if counting_crashes:
                with open(filename,'r') as f:
                    #print("reading {}".format(filename))
                    line = f.readline()
                    line = f.readline()
                    #header = ["TIMESTAMP","VEHICLE_POSITION","VEHICLE_ORIENTATION","VELOCITY","LIDAR","CRASH","EXTERNAL_VEHICLES"]
                    while line:
                        crash = line.split(",")
                        # print("crash[-1]:{}".format(crash[-1]))
                        # print("crash[-2]:{}".format(crash[-2]))
                        # print("crash[-3]:{}".format(crash[-3]))
                        #break
                        if crash[-2] != "0":
                            print("File {} contains crash with severity {}".format(filename, crash[-2]))
                            crashes += 1
                            dir_crashes += 1
                            break
                        line = f.readline()
        print("Crashes in {}: {}\n".format(d, dir_crashes))
    print("Total runs: {}".format(total_runs))
    print("Total crashes: {}".format(crashes))
    print("Crash rate: {}".format(crashes / total_runs))


def main():
    overallbegin = time.time()
    run_scenario()



if __name__ == '__main__':
    main()
