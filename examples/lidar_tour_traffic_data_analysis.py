import sys
from time import sleep
import numpy as np
from beamngpy.beamngcommon import *
import time
import random, copy
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.pyplot import imshow
import pandas as pd
import seaborn as sn
import matplotlib.pyplot as plt
from scipy.stats import norm
from astropy import modeling
import shutil
import sklearn
from sklearn import cluster

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

def collate_crash_files(directory="H:/experiment2/"):
    dirs = os.listdir(directory)
    print("dirs:{}".format(dirs))
    for d in dirs:
        files = os.listdir("{}{}".format(directory, d))
        for filename in files:
            full_filename = "{}{}/{}".format(directory, d, filename)
            with open(full_filename,'r') as f:
                print("reading {}".format(full_filename))
                header = f.readline().replace("\n","").split(",") # get header
                line = f.readline()
                crash_val = 0
                # header = ["TIMESTAMP","VEHICLE_POSITION","VEHICLE_ORIENTATION","VELOCITY","LIDAR","CRASH","EXTERNAL_VEHICLES"]
                while line and crash_val == 0:
                    crash_val = float(line.split(",")[-2])
                    if crash_val != 0:
                        print("File {} contains crash with severity {}".format(filename, crash_val))
                        # copy file to collated crash directory
                        dst = 'H:/experiment_2_crashes/{{}'.format(filename)
                        print("copying", full_filename, " to ", dst)
                        shutil.copyfile(full_filename, dst)
                        break
                    line = f.readline()

def parse_files(directory="H:/experiment2/"):
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
                            print("File {} contains crash with severity {}".format(filename, crash[-2]))
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

def separate_line(line, header):
    sdline = dict.fromkeys(header)
    linecomma = line.split(",")
    # print(len(linecomma))
    # print(linecomma[-4:-1])
    # get single value entries
    sdline['TIMESTAMP'] = linecomma[0]
    sdline['CRASH'] = float(linecomma[-2])
    sdline['EXTERNAL_VEHICLES'] = linecomma[-1].replace("\n","")
    # get array value entries
    linearr = line.split("[")
    for str,h in zip(linearr[1:5], header[1:5]):
        str = str.split("]")[0]
        str = str.split(", ")
        sdline[h] = [float(s) for s in str]
    return sdline

# format data for dataframe
def format_data(dicts, header):
    print("dicts none?", dicts is None)
    collated_dict = dict.fromkeys(header)
    for g in ["VEHICLE_POSITION", "VEHICLE_ORIENTATION", "VELOCITY"]:
        for gg in ["_X","_Y","_Z"]:
            collated_dict[g+gg] = None
        del gg
    del g
    for h in header:
        if h == "VEHICLE_POSITION" or h == "VEHICLE_ORIENTATION" or h == "VELOCITY":
            for i,gg in enumerate(["_X","_Y","_Z"]):
                temp = []
                for d in dicts:
                        temp.append(d[h][i])
                collated_dict[h+gg] = copy.deepcopy(temp)
        if h == "LIDAR":
            for i,gg in enumerate(["_MEAN","_VAR"]):
                temp = []
                for d in dicts:
                    if "MEAN" in gg:
                        temp.append(np.mean(d[h]))
                    elif "VAR" in gg:
                        temp.append(np.var(d[h]))
                collated_dict[h+gg] = copy.deepcopy(temp)
        else:
            temp = []
            for d in dicts:
                temp.append(d[h])
            collated_dict[h] = copy.deepcopy(temp)
    for h in header:
        if h != "LIDAR":
            print("collated_dict", h, collated_dict[h])
    # keys = collated_dict.keys()
    # for h in keys:
    #     if collated_dict[h] == None:
    #         collated_dict.pop(h)
    return collated_dict

def parse_files_endstate(directory="H:/experiment2/"):
    #directory = "H:/experiment2/"
    # set True if counting crashes
    # Counting crashes is sloowwww
    counting_crashes = True
    dirs = os.listdir(directory)
    print("dirs:{}".format(dirs))
    crashes = 0
    total_runs = 0
    crash_vals = []
    all_dicts = []
    for d in dirs:
        files = os.listdir("{}{}".format(directory, d))
        dir_dicts = []
        for filename in files:
            filename = "{}{}/{}".format(directory, d, filename)
            total_runs += 1
            with open(filename,'r') as f:
                print("reading {}".format(filename))
                t = 0
                header = f.readline().replace("\n","").split(",") # get header
                # print("header", header)
                # lastline = f.readlines()[-1]
                # print(lastline)
                line = f.readline()
                crash_val = 0
                # header = ["TIMESTAMP","VEHICLE_POSITION","VEHICLE_ORIENTATION","VELOCITY","LIDAR","CRASH","EXTERNAL_VEHICLES"]
                while line and crash_val == 0:
                    crash_val = float(line.split(",")[-2])
                    if crash_val != 0:
                        print("File {} contains crash with severity {}".format(filename, crash_val))
                        separated_line_dict = separate_line(line, header)
                        dir_dicts.append(copy.deepcopy(separated_line_dict))
                    line = f.readline()
        all_dicts.extend(dir_dicts)
        data = format_data(dir_dicts, header)
        df = pd.DataFrame(data, columns=data.keys())
        print(df)
        corrMatrix = df.corr()
        print(corrMatrix)
        figure(figsize=(10,10), dpi=80)
        sn.heatmap(corrMatrix, annot=True)

        plt.title("{} Pearson Correlation".format(d))
        plt.gcf().subplots_adjust(left=0.21, bottom=0.25)
        plt.show()
        # plt.savefig(directory + "{}-Pearson-Correlation.jpg".format(d))
        plt.pause(0.01)

        # print("Crashes in {}: {} ({} of {} runs)".format(d, dir_crashes, dir_crashes / float(len(files)), len(files)))
        # print("Avg crash severity:{}\n".format(sum(dir_crash_vals) / len(dir_crash_vals)))
        # #break
    data = format_data(dir_dicts, header)
    df = pd.DataFrame(data, columns=data.keys())
    print("dataframe:\n", df)
    corrMatrix = df.corr()
    print(corrMatrix)
    figure(figsize=(10, 10), dpi=80)
    sn.heatmap(corrMatrix, annot=True)
    plt.title("All Traces Pearson Correlation")
    plt.gcf().subplots_adjust(left=0.25, bottom=0.25)
    plt.show()
    # plt.savefig(directory+"All-Traces-Pearson-Correlation.jpg")
    plt.pause(0.01)

    print("Total runs: {}".format(total_runs))
    print("Total crashes: {}".format(crashes))
    print("Crash rate: {}".format(crashes / total_runs))
    print("Max crash severity:{}".format(max(crash_vals)))
    print("Avg crash severity:{}".format(sum(crash_vals)/len(crash_vals)))
    #print("Min crash val:{}".format(min(crash_vals)))
    return crash_vals, ts

def create_feature_vector(line_dict):
    arr = []
    for k in line_dict.keys():
        if k != "LIDAR":
             if isinstance(line_dict[k], list):
                arr.extend(line_dict[k])
             else:
                 arr.append(float(line_dict[k]))
    return arr

def parse_files_equiv(directory="H:/experiment2_crashes/"):
    files = os.listdir(directory)
    crash_sigs = {}
    for filename in files:
        if ".csv" in filename:
            full_filename = "{}/{}".format(directory, filename)
            with open(full_filename,'r') as f:
                # print("reading {}".format(full_filename))
                header = f.readline().replace("\n","").split(",") # get header
                line = f.readline()
                crash_val = 0
                # header = ["TIMESTAMP","VEHICLE_POSITION","VEHICLE_ORIENTATION","VELOCITY","LIDAR","CRASH","EXTERNAL_VEHICLES"]
                while line and crash_val == 0:
                    crash_val = float(line.split(",")[-2])
                    if crash_val != 0:
                        print("File {} contains crash with severity {}".format(filename, crash_val))
                        separated_line_dict = separate_line(line, header)
                        arr = create_feature_vector(separated_line_dict)
                        crash_sigs[filename] = copy.deepcopy(arr)
                        break
                    line = f.readline()

    df = pd.DataFrame(crash_sigs, columns=crash_sigs.keys())
    print("dataframe:\n", df)
    corrMatrix = df.corr()
    # print(corrMatrix)
    # GROUP BY HIGHEST CORRELATION ONLY
    c = df.corr()#.abs()
    s = c.unstack()
    so = s.sort_values(kind="quicksort")
    # print("len(so)", len(so))
    # print(so[-296:-148])
    new = so[-296:-148]
    # array of sets
    groups = np.array([])
    for i in new.items():
        # file names are i[0][0] and i[0][1]
        added = False
        for group in groups:
            if i[0][0] in group or i[0][1] in group:
                group.add(i[0][0])
                group.add(i[0][1])
                added = True
        if not added:
            groups = np.append(groups, {i[0][0], i[0][1]})
    # REGROUP BY SECOND HIGHEST CORRELATION -- use second most correlated grouping to consolidate groups
    print(so[-444:-296])
    new = so[-444:-296]
    for i in new.items():
        # file names are i[0][0] and i[0][1]
        added = False
        for group in groups:
            if i[0][0] in group or i[0][1] in group:
                group.add(i[0][0])
                group.add(i[0][1])
                added = True
        if not added:
            groups = np.append(groups, {i[0][0], i[0][1]})
    # print finished groups
    print("\nCORRELATION GROUPS ({}):".format(groups.shape))
    for g in groups:
        print(g)
        # correlation is i[1]

    # # Convert DataFrame to matrix
    # mat = df.values
    # # Using sklearn
    # km = cluster.KMeans(n_clusters=5)
    # km.fit(mat)
    # # Get cluster assignment labels
    # labels = km.labels_
    # # Format results as a DataFrame
    # results = pd.DataFrame(data=labels, columns=['cluster'])
    # print("SKLEARN RESULTS ({}, {}): ".format(type(results), len(results)))
    # print(results)

    # sz = 150
    # dpi = 100
    # figure(figsize=(sz, sz), dpi=100)
    # sn.heatmap(corrMatrix, annot=True)
    # plt.title("All Traces Pearson Correlation")
    # plt.gcf().subplots_adjust(left=0.25, bottom=0.25)
    # plt.show()
    # plt.pause(0.01)
    # plt.savefig(directory+"All-Traces-Pearson-Correlation-{}x{}dpi={}.png".format(sz,sz,dpi))
    # find highest correlations

def main():
    overallbegin = time.time()
    # collate_crash_files()
    # crash_vals, ts = parse_files()
    # crash_vals, ts = parse_files_endstate()
    parse_files_equiv()
    # make_histograms(crash_vals)
    # make_crash_histograms(crash_vals)
    # make_time_to_crash_histograms(ts)
    # make_gaussian(crash_vals)
    # process_time_to_crash(ts)



if __name__ == '__main__':
    main()
