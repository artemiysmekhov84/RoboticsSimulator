import numpy
from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# TODO: Try to use another hosting.
# TODO: Try to ask a questions on Drake/Meshcat/Railway forums.
# TODO: Add 'reset' endpoint.

builder = DiagramBuilder()
plant, sceneGraph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)  # type: ignore
Parser(plant).AddModelFromFile("./pendulum.sdf")  # Load model.
plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -9.8])  # Add gravity.
plant.Finalize()

# Set initial position of joints.
plant.GetJointByName("joint_0").set_default_angle(numpy.pi / 2.0)
plant.GetJointByName("joint_1").set_default_angle(numpy.pi / 2.0)

# Add visualizer to visualize the geometries.
params = MeshcatParams(host="0.0.0.0", port=8000, web_url_pattern="https://{host}:{port}")
meshcat = Meshcat(params)
MeshcatVisualizer.AddToBuilder(builder, sceneGraph, meshcat)  # type: ignore

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
plantContext = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
plant.get_actuation_input_port().FixValue(plantContext, numpy.zeros(plant.num_actuators()))
simulator.AdvanceTo(numpy.inf)
