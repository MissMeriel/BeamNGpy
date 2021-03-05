from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import setup_logging, Config
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer
import beamngpy
import time, random

def main():
    filename = 'C:/Users/merie/Documents/BeamNG.research/breakgroups.txt'
    breakgroups = set()
    with open(filename, 'r') as f:
        lines = f.readlines()
        for l in lines[2:]:
            spl = l.split(" ")
            bg = spl[-1].strip("\n")
            print(spl)
            breakgroups.add(bg)
    print(breakgroups)
    print(len(breakgroups))


if __name__ == "__main__":
    main()