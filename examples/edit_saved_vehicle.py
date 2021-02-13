import sys

from time import sleep


import numpy as np

from scipy import interpolate

from beamngpy import BeamNGpy, Scenario, Road, Vehicle, setup_logging
import json

SIZE = 1024


def main():
    pristine_filename = 'C:/Users/merie/Documents/BeamNG.research/vehicles/pickup/pristine.save.json'
    filename = 'C:/Users/merie/Documents/BeamNG.research/vehicles/pickup/poppedtires.save.json'
    with open(filename, 'r') as f:
        file_json = f.read().replace('\n', '')
        j = json.loads(file_json)
        print(j)
        print()
        print(j.keys())
        for k in j.keys():
            try:
                print("j[{}].keys() = {}".format(k, j[k].keys()))
                for k2 in j[k].keys():
                    #if 'tire' in k2:
                    print("\tj[{}][{}] = {}".format(k, k2, j[k][k2]))
                        #print(j[k][k2])
                print()
            except:
                if 'luaState' in k:
                    print("j[{}] =".format(k))
                    parts = j[k].split(',')
                    for p in parts:
                        print("\t{}".format(p))
                    print()
                else:
                    print("j[{}] = {}\n".format(k, j[k]))


if __name__ == '__main__':
    main()
