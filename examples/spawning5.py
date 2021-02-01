from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import setup_logging, Config
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer
import beamngpy
import time, random

# globals
default_model = 'pickup'
default_scenario = 'west_coast_usa' #'cliff' # smallgrid
dt = 20

def spawn_point(scenario_locale):
    if scenario_locale is 'cliff':
        #return {'pos':(-124.806, 142.554, 465.489), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-124.806, 190.554, 465.489), 'rot': None, 'rot_quat': (0, 0, 0.3826834, 0.9238795)}
    elif scenario_locale is 'west_coast_usa':
        #return {'pos':(-717.121, 101, 118.675), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}
        return {'pos': (-717.121, 101, 118.675), 'rot': None, 'rot_quat': (0, 0, 0.918812, -0.394696)}
    #906, 118.78 rot:
    elif scenario_locale is 'smallgrid':
        return {'pos':(0.0, 0.0, 0.0), 'rot':None, 'rot_quat':(0, 0, 0.3826834, 0.9238795)}

def setup_sensors(vehicle):
    # Set up sensors
    pos = (-0.3, 1, 1.0)
    direction = (0, 1, 0)
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
    damage.encode_vehicle_request()
    lidar = Lidar(visualized=False)
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('front_cam', front_camera)
    vehicle.attach_sensor('back_cam', back_camera)
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)
    return vehicle

def compare_damage(d1, d2):
    for key in d1['damage']:
        if d1['damage'][key] != d2['damage'][key]:
            print("d1['damage'][{}] == {}; d2['damage'][{}] == {}".format(key, d1['damage'][key], key, d2['damage'][key]))
            try:
                # handle specific keys
                if key == 'deform_group_damage' or key == 'part_damage':
                    for k in d1['damage'][key].keys():
                        print("\td1['damage'][{}][{}] == {}; d2['damage'][{}][{}] == {}".format(key, k, d1['damage'][key][k], key, k,
                                                                                      d2['damage'][key][k]))
                else:
                    if d1['damage'][key] < d2['damage'][key]:
                        print("\td2[damage][{}] is greater".format(key))
                    else:
                        print("\td1[damage][{}] is greater".format(key))
            except:
                continue
    print()
    return

def backup(cum_list, sec):
    #return "1_24"
    dt = sec * 5.0
    index = len(cum_list) - int(dt)
    if index < 0:
        index = 0
    elif index >= len(cum_list):
        index = len(cum_list) -1
    print("cum_list={}".format(cum_list))
    print("index={}".format(index))
    #try:
    return cum_list[index]
    #except:

    #return "0_0"

def main():
    global default_model, default_scenario
    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')

    #scenario = Scenario('smallgrid', 'spawn_objects_example')
    scenario = Scenario(default_scenario, 'research_test', description='Random driving for research')

    vehicle = Vehicle('ego_vehicle', model=default_model, licence='PYTHON')
    vehicle = setup_sensors(vehicle)
    spawn = spawn_point(default_scenario)
    scenario.add_vehicle(vehicle, pos=spawn['pos'], rot=spawn['rot'], rot_quat=spawn['rot_quat'])

    scenario.make(beamng)

    bng = beamng.open()
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.update_vehicle()

    d1 = bng.poll_sensors(vehicle)
    cum_list = []
    bound = 0.0
    for i in range(3):
        for _ in range(45):
            bound = bound + 0.0 # 0.1
            # vehicle.save()
            vehicle.update_vehicle()
            d2 = bng.poll_sensors(vehicle)
            throttle = 1.0
            #throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1 * bound, bound)
            brake = 0.0 #random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)
            pointName = "{}_{}".format(i, _)
            cum_list.append(pointName)
            vehicle.saveRecoveryPoint(pointName)
            bng.step(20)
        print("SEGMENT #{}: COMPARE DAMAGE".format(i))
        damage_diff = compare_damage(d1, d2)
        d1 = d2
        # "Back up" 1 second -- load vehicle at that time in that position.
        backup_pointName = backup(cum_list, 0.001)
        print('recovering to {}'.format(pointName))
        loadfile = vehicle.loadRecoveryPoint(backup_pointName)
        print('loadfile is {}'.format(loadfile))
        bng.pause()
        vehicle.update_vehicle()
        vehicle.load(loadfile)
        #vehicle.load("vehicles/pickup/vehicle.save.json")
        bng.resume()
        #vehicle.startRecovering()
        #time.sleep(1.5)
        #vehicle.stopRecovering()

    vehicle.update_vehicle()
    bng.pause()
    time.sleep(2)
    # vehicle.load("vehicles/pickup/vehicle.save.json")
    bng.resume()
    bng.close()

if __name__ == "__main__":
    main()