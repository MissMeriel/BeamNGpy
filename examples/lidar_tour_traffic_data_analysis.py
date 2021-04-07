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

from scipy.stats import norm
from astropy import modeling

def diff_damage(damage, damage_prev):
    new_damage = 0
    if damage is None or damage_prev is None:
        return 0
    new_damage = damage['damage'] - damage_prev['damage']
    return new_damage


def make_gaussian(crash_vals):
    mean, std = norm.fit(crash_vals)

    plt.hist(crash_vals, bins=30) #, normed=True)
    xmin, xmax = plt.xlim()
    print("mean:{} std:{} xmin:{} xmax:{}".format(mean, std, xmin, xmax))

    x = np.linspace(xmin, xmax, 100)
    y = norm.pdf(x, mean, std)
    plt.plot(x, y)

    # fit line
    # fitter = modeling.fitting.LevMarLSQFitter()
    # model = modeling.models.Gaussian1D()  # depending on the data you need to give some initial values
    # fitted_model = fitter(model, x, crash_vals)
    #
    # plt.plot(x, crash_vals)
    # plt.plot(x, fitted_model(x))
    plt.show()
    plt.pause(1)

def make_time_to_crash_histograms(time_to_crash_vals):
    temp = [c for c in time_to_crash_vals if c < 45]
    for i, binwidth in enumerate([1, 5, 10, 15]):
        # set up plot
        ax = plt.subplot(2, 2, i+1)
        # draw plot
        maximum = max(time_to_crash_vals)
        ax.hist(temp, bins=int(maximum/binwidth), color='blue', edgecolor='black')
        ax.set_title('Time to Crash \n(Binwidth={})'.format(binwidth), size=18)
        ax.set_xlabel('Time (s)', size=10)
        ax.set_ylabel('Runs', size=10)
    plt.tight_layout()
    plt.show()
    plt.pause(1)

def make_crash_histograms(crash_vals):
    temp = [c for c in crash_vals if c != 0]
    for i, binwidth in enumerate([10, 100, 1000, 2000]):
        # set up plot
        ax = plt.subplot(2, 2, i+1)
        # draw plot
        maximum = max(crash_vals)
        ax.hist(temp, bins=int(maximum/binwidth), color='blue', edgecolor='black')
        ax.set_title('Crash Vehicle Damage \n(Binwidth={})'.format(binwidth), size=18)
        ax.set_xlabel('Vehicle damage', size=10)
        ax.set_ylabel('Runs', size=10)
    plt.tight_layout()
    plt.show()
    plt.pause(1)


def make_histograms(crash_vals):
    #for i, binwidth in enumerate([1000, 2000, 5000, 10000]):
    for i, binwidth in enumerate([10, 100, 1000, 2000]):
        # set up plot
        ax = plt.subplot(2, 2, i+1)
        # draw plot
        maximum = max(crash_vals)
        ax.hist(crash_vals, bins=int(maximum/binwidth), color='blue', edgecolor='black')
        ax.set_title('All Vehicle Damage \n(Binwidth={})'.format(binwidth), size=18)
        ax.set_xlabel('Vehicle damage', size=10)
        ax.set_ylabel('Runs', size=10)

    plt.tight_layout()
    plt.show()
    plt.pause(1)



def process_time_to_crash(ts):
    print("ts:{}".format(ts))
    crash_ts = [t for t in ts if t < 45]
    crash_avg = sum(crash_ts) / len(crash_ts)
    avg = sum(ts) / len(ts)
    print("Number of crash traces:{}".format(len(crash_ts)))
    print("Avg time to crash for crash traces:{}".format(crash_avg))
    print("Avg time for all traces:{}".format(avg))


def run_scenario(directory="H:/experiment2/"):
    #directory = "H:/experiment2/"
    # set True if counting crashes
    # Counting crashes is sloowwww
    counting_crashes = True
    dirs = os.listdir(directory)
    print("dirs:{}".format(dirs))
    crashes = 0
    total_runs = 0
    crash_vals = []
    ts = []
    for d in dirs:
        files = os.listdir("{}{}".format(directory, d))
        dir_crashes = 0
        dir_crash_vals = []
        for filename in files:
            filename = "{}{}/{}".format(directory, d, filename)
            total_runs += 1
            if counting_crashes:
                with open(filename,'r') as f:
                    #print("reading {}".format(filename))
                    t = 0
                    line = f.readline()
                    line = f.readline()
                    #header = ["TIMESTAMP","VEHICLE_POSITION","VEHICLE_ORIENTATION","VELOCITY","LIDAR","CRASH","EXTERNAL_VEHICLES"]
                    while line:
                        crash = line.split(",")
                        # print("crash[-1]:{}".format(crash[-1]))
                        # print("crash[-2]:{}".format(crash[-2]))
                        # print("crash[-3]:{}".format(crash[-3]))
                        #break
                        crash_val = float(crash[-2])
                        if crash_val != 0:
                            #print("File {} contains crash with severity {}".format(filename, crash[-2]))
                            crashes += 1
                            dir_crashes += 1
                            crash_vals.append(float(crash[-2]))
                            dir_crash_vals.append(float(crash[-2]))
                            #ts.append(t)
                            break
                        line = f.readline()
                        t += 0.25
                    crash_vals.append(0)
                    ts.append(t)
        print("Crashes in {}: {} ({} of {} runs)".format(d, dir_crashes, dir_crashes / float(len(files)), len(files)))
        print("Avg crash severity:{}\n".format(sum(dir_crash_vals) / len(dir_crash_vals)))
        #break
    print("Total runs: {}".format(total_runs))
    print("Total crashes: {}".format(crashes))
    print("Crash rate: {}".format(crashes / total_runs))
    print("Max crash severity:{}".format(max(crash_vals)))
    print("Avg crash severity:{}".format(sum(crash_vals)/len(crash_vals)))
    #print("Min crash val:{}".format(min(crash_vals)))
    return crash_vals, ts

def main():
    overallbegin = time.time()
    crash_vals, ts = run_scenario()
    make_histograms(crash_vals)
    make_crash_histograms(crash_vals)
    make_time_to_crash_histograms(ts)
    make_gaussian(crash_vals)
    process_time_to_crash(ts)



if __name__ == '__main__':
    main()
