# TODO: Seems like all OS-level dependensies installed locally are not including into the final Docker image.
# This is why deploy crashes on run with error "ImportError: libX11.so.6: cannot open shared object file".
# Need to make Docker container including all required dependencies and push it to Git repository.
# Required dependency list: libx11-6 libsm6 libglib2.0-0 [python-tk].
# Google for "putting X11 into a docker container", "install python drake into docker".
# apt-get update -y && apt-get upgrade -y && apt-get install -y --no-install-recommends xvfb xpra xserver-xephyr xinit xauth xclip x11-xserver-utils x11-utils libx11-6 libsm6 libglib2.0-0 python-tk

# TODO: Add 'reset' endpoint.

import subprocess

print("========== CHECK IF libx11 IS INSTALLED ==========")
subprocess.call("apt list --installed | grep libx11", shell=True)

import numpy

print("========== BEFORE IMPORT DRAKE ==========")

from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

print("========== AFTER IMPORT DRAKE ==========")

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
