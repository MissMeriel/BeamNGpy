from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import setup_logging, Config
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer
import beamngpy
import time, random

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
            print("d1[damage][{}] == {}; d2[damage][{}] == {}".format(key, d1['damage'][key], key, d2['damage'][key]))
            try:
                # handle specific keys
                if key is 'deform_group_damage' or key is 'part_damage':
                    if isinstance(d1['damage'][key], list):
                        keys = d2['damage'][key].keys()
                        for k in keys:
                            print("\td1['damage'][{}][{}] == {}; d2['damage'][{}][{}] == {}".format(key, k, None, key, k,
                                                                                          d2['damage'][key][k]))

                    elif isinstance(d2['damage'][key], list):
                        keys = d1['damage'][key].keys()
                        for k in keys:
                            print(
                                "\td1['damage'][{}][{}] == {}; d2['damage'][{}][{}] == {}".format(key, k, d1['damage'][key][k]), key, k,
                                                                                                  None)
                    else:
                        for k in d1['damage'][key].keys():
                            print("\td1['damage'][{}][{}] == {}; d2['damage'][{}][{}] == {}".format(key, k, d1['damage'][key][k],
                                                                                                    key, k, d2['damage'][key][k]))
                            # if d1['damage'][key]['damage'] < d2['damage'][key]['damage']:
                            #     print("\td2[damage][{}][damage] is greater".format(key))
                            # else:
                            #     print("\td1[damage][{}][damage] is greater".format(key))
                else:
                    if d1['damage'][key] < d2['damage'][key]:
                        print("\td2[damage][{}] is greater".format(key))
                    else:
                        print("\td1[damage][{}] is greater".format(key))
            except Exception as e:
                print(e)
                continue
    print()
    return


def main():
    beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
    #scenario = Scenario('smallgrid', 'spawn_objects_example')
    # Create a scenario in west_coast_usa
    scenario = Scenario('west_coast_usa', 'research_test', description='Random driving for research')
    scenario.make(beamng)
    print("scenario.make(beamng)")
    bng = beamng.open()
    print("beamng.open()")
    scenario.connect(bng)
    print("scenario.connect(bng)")
    bng.load_scenario(scenario)
    print("loaded scenario")
    vehicle = Vehicle('ego_vehicle', model='pickup', licence='PYTHON')
    vehicle = setup_sensors(vehicle)
    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    #vehicle.load()
    #print("vehicle loaded")

    scenario_obj = ScenarioObject(oid='roadblock',
                                  name='sawhorse',
                                  otype='BeamNGVehicle',
                                  pos=(0,-5,0),
                                  rot=None,
                                  rot_quat=(0,0,0,1),
                                  scale=(1, 1, 1),
                                  JBeam = 'sawhorse',
                                  datablock="default_vehicle"
                                  )
    scenario.add_object(scenario_obj)
    print("scenario.add_object(scenario_obj)")

    bng.start_scenario()
    print("started scenario")
    #bng.spawn_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))
    #scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot=None, rot_quat=(0, 0, 0.3826834, 0.9238795))

    print("vel: {}".format(vehicle.state['vel']))
    vehicle.update_vehicle()
    # print("vel: {}".format(vehicle.state['vel']))
    # print("pos: {}".format(vehicle.state['pos']))
    # print("dir: {}".format(vehicle.state['dir']))
    # print("front: {}".format(vehicle.state['front']))
    # print("up: {}".format(vehicle.state['up']))
    #vehicle.load()
    d1 = bng.poll_sensors(vehicle)
    for i in range(3):
        for _ in range(35):
            # vehicle.save()
            vehicle.update_vehicle()
            bng.update_scenario()
            scenario.update()
            d2 = bng.poll_sensors(vehicle)
            throttle = random.uniform(0.0, 0.5) # 1.0
            steering = random.uniform(-.2, 0.2) #-1.0
            brake = 0.0 #random.choice([0, 0, 0, 1]) #0.0
            vehicle.control(throttle=throttle, steering=steering, brake=brake)
            bng.step(20)
            print(bng.get_gamestate())
            print()

        print("SEGMENT #{}: COMPARE DAMAGE".format(i))
        damage_diff = compare_damage(d1, d2)
        d1 = d2
        print("\nSTART RECOVERING")
        vehicle.startRecovering()
        print("SLEEP")
        time.sleep(1.5)
        print("STOP RECOVERING")
        vehicle.stopRecovering()
        print("updating scenario...")
        bng.update_scenario()
        print("damage after recovering: {}\n".format(bng.poll_sensors(vehicle)['damage']['damage']))
        time.sleep(2)



    time.sleep(3)
    # dir indices equate to rpy
    # positive roll is upwards
    #temp = [vehicle.state['dir'][0]+0, vehicle.state['dir'][1]+50, vehicle.state['dir'][2]+0]
    #bng.teleport_vehicle(vehicle, pos=(0,0,0), rot=temp, rot_quat=None)
    #print("teleported vehicle to  pos=(0,0,0), rot={}".format(temp))

    print("updating vehicle...")
    vehicle.update_vehicle()
    # print("vel: {}".format(vehicle.state['vel']))
    # print("pos: {}".format(vehicle.state['pos']))
    # print("dir: {}".format(vehicle.state['dir']))
    # print("front: {}".format(vehicle.state['front']))
    # print("up: {}".format(vehicle.state['up']))
    print("\nSTART RECOVERING")
    vehicle.startRecovering()
    print("SLEEP")
    time.sleep(1.5)
    print("STOP RECOVERING")
    vehicle.stopRecovering()
    #vehicle.state['pos'] = [0,0,0]
    #vehicle.update_vehicle()    # syncs object state to sim state; doesn't update sim.
    #scenario.remove_vehicle(vehicle) # closes socket for vehicle
    #print("vehicle removed")

    #vehicle.load()
    #print("vehicle loaded")
    #scenario.add_vehicle(vehicle, pos=(0,0,0), rot=None, rot_quat=(0,0,0,1))
    #print("added vehicle to  pos=(0,0,0), rot_quat=(0,0,0,1)")
    #bng.teleport_vehicle(vehicle, pos=(0,0,0), rot=None, rot_quat=(0,0,0,0))
    #print("teleported vehicle to  pos=(0,0,0), rot_quat=(0,0,0,0)")
    print("end of script")

if __name__ == "__main__":
    main()