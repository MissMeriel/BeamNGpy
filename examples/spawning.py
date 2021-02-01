from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import Config

beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')

scenario = Scenario('smallgrid', 'spawn_objects_example')

vehicle = Vehicle('ego_vehicle', model='pickup', licence='PYTHON')
scenario.add_vehicle(vehicle, pos=(0,0,0), rot=None, rot_quat=(0,0,0,1))

scenario_obj = ScenarioObject(oid='roadblock',
                              name='sawhorse',
                              otype='BeamNGVehicle',
                              pos=(0,-10,0),
                              rot=None,
                              rot_quat=(0,0,0,1),
                              scale=(1, 1, 1),
                              JBeam = 'sawhorse',
                              datablock="default_vehicle"
                              )
scenario.add_object(scenario_obj)

scenario.make(beamng)

bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()


vehicle = Vehicle('ego_vehicle3', model='etk800', color='White')
bng.spawn_vehicle(vehicle, pos=(0,-20, 0), rot=None, rot_quat=(0, 1, 0, 0))