"""
.. module:: west_coast_random
    :platform: Windows
    :synopsis: Example code making a scenario in west_coast_usa and having a
               car drive around randomly.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""
import mmap
import random, math
import sys, time
from time import sleep
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging, StaticObject
from beamngpy.sensors import Camera, GForces, Electrics, Damage, Timer

from scipy.spatial.transform import Rotation as R
from PIL import Image
import PIL
import cv2
import scipy.misc
import csv
def process_csv(filename):
    global path_to_trainingdir
    hashmap = []
    timestamps = []; steerings = []; throttles = []
    with open(filename) as csvfile:
        metadata = csv.reader(csvfile, delimiter=',')
        next(metadata)
        for row in metadata:
            steerings.append(float(row[2]))
            throttles.append(float(row[3]))
            timestamps.append(float(row[1]))
            # imgfile = row[0].replace("\\", "/")
            # hashmap[i] = row[1:]
    return timestamps, steerings, throttles

def plot_one_lap_of_steering():
    filename = 'H:/BeamNG_DAVE2_racetracks_all/training_images_industrial-racetrackstartinggate0/data.csv'
    x,y_steer, y_throttle = process_csv(filename)
    plt.plot(x[:1492], y_steer[:1492])
    plt.title("Steering over one lap")
    plt.show()
    plt.pause(0.01)
    print(len([i for i in y_steer if i > 0.1]))

def main():
    plot_one_lap_of_steering()


if __name__ == '__main__':
    main()
