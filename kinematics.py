import time
import numpy as np
import pinocchio as pin
import placo
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

robot = placo.RobotWrapper("robot/")
viz = robot_viz(robot)

solver = robot.make_solver()

joints_task = solver.add_joints_task()
joints_task.configure("joints", "soft", 1.0)

constraint = solver.add_relative_position_task("a1", "a2", np.array([0., 0., 0.]))
constraint.configure("constraint", "hard", 1.0)
constraint.mask.set_axises("xy")

t:float = 0.0
dt:float = 0.01

while True:
    viz.display(robot.state.q)

    joints_task.set_joints({"cylinder": np.sin(t)*0.05})

    robot.update_kinematics()
    solver.solve(True)
    solver.dump_status()

    t += dt
    time.sleep(dt)
