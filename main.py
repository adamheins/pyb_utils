import time
import numpy as np
import pybullet as pyb
import pybullet_data

from collision import NamedCollisionObject, CollisionDetector


def load_environment(client_id):
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )

    # ground plane
    ground_id = pyb.loadURDF(
        "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
    )

    # KUKA iiwa robot arm
    kuka_id = pyb.loadURDF(
        "kuka_iiwa/model.urdf",
        [0, 0, 0],
        useFixedBase=True,
        physicsClientId=client_id,
    )

    # some cubes for obstacles
    cube1_id = pyb.loadURDF(
        "cube.urdf", [1, 1, 0.5], useFixedBase=True, physicsClientId=client_id
    )
    cube2_id = pyb.loadURDF(
        "cube.urdf", [-1, -1, 0.5], useFixedBase=True, physicsClientId=client_id
    )
    cube3_id = pyb.loadURDF(
        "cube.urdf", [1, -1, 0.5], useFixedBase=True, physicsClientId=client_id
    )

    # store body indices in a dict with more convenient key names
    bodies = {
        "robot": kuka_id,
        "ground": ground_id,
        "cube1": cube1_id,
        "cube2": cube2_id,
        "cube3": cube3_id,
    }
    return bodies


def main():

    # main simulation server, with a GUI
    sim_id = pyb.connect(pyb.GUI)

    # simulation server only used for collision detection
    col_id = pyb.connect(pyb.DIRECT)

    # add bodies to both of the environments
    bodies = load_environment(sim_id)
    collision_bodies = load_environment(col_id)

    # define bodies (and links) to use for shortest distance computations and
    # collision checking
    ground = NamedCollisionObject("ground")
    cube1 = NamedCollisionObject("cube1")
    cube2 = NamedCollisionObject("cube2")
    cube3 = NamedCollisionObject("cube3")
    link7 = NamedCollisionObject("robot", "lbr_iiwa_link_7")  # last link

    col_detector = CollisionDetector(
        col_id,
        collision_bodies,
        [(link7, ground), (link7, cube1), (link7, cube2), (link7, cube3)],
    )

    while True:
        # compute shortest distances for a random configuration
        q = np.pi * (np.random.random(7) - 0.5)
        d = col_detector.compute_distances(q)
        in_col = col_detector.in_collision(q)

        print(f"Configuration = {q}")
        print(f"Distance to obstacles = {d}")
        print(f"In collision = {in_col}")

        # wait for user to press enter to continue
        input()

        # the main GUI-based simulation is not affected
        pyb.stepSimulation(physicsClientId=sim_id)


if __name__ == "__main__":
    main()
