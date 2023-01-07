# TODO: Remove all default UI controls.
# TODO: Need to figure out how does all these classes and methods work.

import time
import numpy
import pydrake.geometry
import pydrake.multibody.plant
import pydrake.multibody.parsing
import pydrake.systems.analysis
import pydrake.systems.framework


class DoublePendulum:
    meshcat: pydrake.geometry.Meshcat
    simulator: pydrake.systems.analysis.Simulator
    timeStep: float

    def __init__(self):
        self.meshcat = pydrake.geometry.Meshcat()
        self.meshcat.AddButton("RESET", "KeyR")
        self.timeStep = 0.001
        self.reset()

    def reset(self):
        builder = pydrake.systems.framework.DiagramBuilder()
        plant, sceneGraph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, time_step=self.timeStep)  # type: ignore
        pydrake.geometry.MeshcatVisualizer.AddToBuilder(builder, sceneGraph, self.meshcat)  # type: ignore
        diagram = builder.Build()
        pydrake.multibody.parsing.Parser(plant).AddModelFromFile("./double_pendulum.sdf")  # Load model.
        plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -9.8])  # Add gravity.
        plant.GetJointByName("joint_0").set_default_angle(numpy.pi / 2.0)
        plant.GetJointByName("joint_1").set_default_angle(numpy.pi / 2.0)
        plant.Finalize()
        self.simulator = pydrake.systems.analysis.Simulator(diagram)
        self.simulator.Initialize()
        self.simulator.set_target_realtime_rate(1.0)
        plantContext = diagram.GetMutableSubsystemContext(plant, self.simulator.get_mutable_context())
        plant.get_actuation_input_port().FixValue(plantContext, numpy.zeros(plant.num_actuators()))

    def run(self):
        resetButtonClicksNumber = 0
        while True:
            if self.meshcat.GetButtonClicks("RESET") != resetButtonClicksNumber:
                resetButtonClicksNumber = self.meshcat.GetButtonClicks("RESET")
                self.reset()

            self.simulator.AdvanceTo(self.simulator.get_context().get_time() + self.timeStep)
            time.sleep(self.timeStep)


if __name__ == "__main__":
    doublePendulum = DoublePendulum()
    doublePendulum.run()
