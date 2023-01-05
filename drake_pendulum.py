import numpy
from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, Role, MeshcatParams, Meshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# TODO: Add tkinter to requirements.txt. Remove it from deployment step in Railway.

p = MeshcatParams(port=8080)
meshcat = Meshcat(p)  # type: ignore

# def model_inspector(filename):
#     meshcat.Delete()
#     meshcat.DeleteAddedControls()
#     builder = DiagramBuilder()
#     # Note: Setting time_step ~= 0.0 tells the constructor
#     # that this system is a discrete system
#     plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)  # type: ignore
#     # Load the file into the plant/scene_graph.
#     parser = Parser(plant)
#     parser.AddModelFromFile(filename)
#     plant.Finalize()
#     # Add two visualizers, one to publish the "visual" geometry, and one to
#     # publish the "collision" geometry.
#     visual = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kPerception, prefix="visual"))  # type: ignore
#     collision = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kProximity, prefix="collision"))  # type: ignore
#     # Disable the collision geometry at the start; it can be enabled by the
#     # checkbox in the meshcat controls.
#     meshcat.SetProperty("collision", "visible", False)
#     # Set the timeout to a small number in test mode. Otherwise, JointSliders
#     # will run until "Stop JointSliders" button is clicked.
#     default_interactive_timeout = 1.0 if "TEST_SRCDIR" in os.environ else None
#     sliders = builder.AddSystem(JointSliders(meshcat, plant))
#     diagram = builder.Build()
#     sliders.Run(diagram, default_interactive_timeout)


# # Press the 'Stop JointSliders' button in MeshCat to continue.
# model_location = "./double_pendulum.sdf"
# # Test to see if model works:
# model_inspector(model_location)


def create_scene(sim_time_step=0.0001):
    # Clean up MeshCat.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)  # type: ignore
    parser = Parser(plant)

    # Load model:
    model_location = "./pendulum.sdf"
    parser.AddModelFromFile(model_location)

    # Add Gravity:
    plant.mutable_gravity_field().set_gravity_vector([0, 0, -9.8])
    plant.Finalize()

    # Set Initial Position of Joints:
    joint_0 = plant.GetJointByName("joint_0")
    joint_0.set_default_angle(numpy.pi / 2.0)
    joint_1 = plant.GetJointByName("joint_1")
    joint_1.set_default_angle(numpy.pi / 2.0)

    context = plant.CreateDefaultContext()

    # Add visualizer to visualize the geometries.
    params = MeshcatVisualizerParams(role=Role.kPerception, prefix="visual")
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)  # type: ignore

    diagram = builder.Build()
    return diagram, visualizer, plant, context


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    return simulator


def run_simulation(sim_time_step):
    diagram, visualizer, plant, context = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    visualizer.StartRecording()
    # Set input ports to be zero:
    plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    plant.get_actuation_input_port().FixValue(plant_context, numpy.zeros(plant.num_actuators()))  # type: ignore
    simulator.AdvanceTo(numpy.inf)
    visualizer.PublishRecording()


# Run the simulation at localhost:7000.
run_simulation(sim_time_step=0.0001)
