import numpy
from pydrake.geometry import MeshcatVisualizer, MeshcatParams, Meshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# import flask
# import waitress

# TODO: Add tkinter to requirements.txt. Remove it from deployment step in Railway.
# Upd: tkinter can't be installed on local machine. Find another way.

# TODO: Try to run simulation in a separate thread and use Flask/Waitress to stream meshcat.StaticHtml() content.

# TODO: Add 'reset' endpoint and call run_simulation() on it.

p = MeshcatParams(host="0.0.0.0", port=8080, show_stats_plot=False)
meshcat = Meshcat(p)  # type: ignore


# @app.route("/stream", methods=["GET"])
# def stream():
#     return flask.Response(meshcat.StaticHtml(), mimetype="multipart/x-mixed-replace; boundary=frame")


def create_scene(sim_time_step=0.0001):
    # Clean up MeshCat.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)  # type: ignore
    parser = Parser(plant)

    # Load model.
    parser.AddModelFromFile("./pendulum.sdf")

    # Add gravity.
    plant.mutable_gravity_field().set_gravity_vector([0, 0, -9.8])
    plant.Finalize()

    # Set initial position of joints.
    joint_0 = plant.GetJointByName("joint_0")
    joint_0.set_default_angle(numpy.pi / 2.0)
    joint_1 = plant.GetJointByName("joint_1")
    joint_1.set_default_angle(numpy.pi / 2.0)

    # Add visualizer to visualize the geometries.
    # params = MeshcatVisualizerParams(role=Role.kPerception, prefix="visual")
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)  # type: ignore # , params)

    diagram = builder.Build()

    return diagram, plant


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    return simulator


def run_simulation(sim_time_step):
    diagram, plant = create_scene(sim_time_step)  # type: ignore
    simulator = initialize_simulation(diagram)
    # visualizer.StartRecording()
    # Set input ports to be zero.
    plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    plant.get_actuation_input_port().FixValue(plant_context, numpy.zeros(plant.num_actuators()))  # type: ignore
    simulator.AdvanceTo(numpy.inf)
    # visualizer.PublishRecording()


# Run the simulation at localhost:7000.
run_simulation(sim_time_step=0.0001)
