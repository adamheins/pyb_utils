"""This module provides utilities for collision checking between objects."""
from dataclasses import dataclass

import numpy as np
import pybullet as pyb

from .named_tuples import getJointInfo


@dataclass
class NamedCollisionObject:
    """Name of a body and one of its links.

    The body name must correspond to the key in the ``bodies`` dict, but is
    otherwise arbitrary. The link name should match the URDF. The link name may
    also be ``None``, in which case the base link ``(index -1)`` is used.

    Attributes
    ----------
    body_name : str
        Name of the body.
    link_name : str
        Name of the link on the body.
    """

    body_name: str
    link_name: str = None


@dataclass
class IndexedCollisionObject:
    """Index of a body and one of its links.

    Attributes
    ----------
    body_uid : int
        UID of the body.
    link_uid : int
        Index of the link on the body.
    """
    body_uid: int
    link_uid: int


def index_collision_pairs(physics_uid, bodies, named_collision_pairs):
    """Convert a list of named collision pairs to indexed collision pairs.

    In other words, convert named bodies and links to the indexes used by
    PyBullet to facilate computing collisions between the objects.

    Parameters
    ----------
    physics_uid : int
        Index of the PyBullet physics server to use.
    bodies : dict
        A dictionary with body names as keys and corresponding UIDs as values.
    named_collision_pairs : list
        A list of pairs of ``NamedCollisionObject``.

    Returns
    -------
        A list of pairs of ``IndexedCollisionObject``.
    """

    # build a nested dictionary mapping body names to link names to link
    # indices
    body_link_map = {}
    for name, uid in bodies.items():
        body_link_map[name] = {}
        n = pyb.getNumJoints(uid, physics_uid)
        for i in range(n):
            link_name = getJointInfo(
                uid, i, physics_uid, decode="utf8"
            ).linkName
            body_link_map[name][link_name] = i

    def _index_named_collision_object(obj):
        """Map body and link names to corresponding indices."""
        body_uid = bodies[obj.body_name]
        if obj.link_name is not None:
            link_uid = body_link_map[obj.body_name][obj.link_name]
        else:
            link_uid = -1
        return IndexedCollisionObject(body_uid, link_uid)

    # convert all pairs of named collision objects to indices
    indexed_collision_pairs = []
    for a, b in named_collision_pairs:
        a_indexed = _index_named_collision_object(a)
        b_indexed = _index_named_collision_object(b)
        indexed_collision_pairs.append((a_indexed, b_indexed))

    return indexed_collision_pairs


class CollisionDetector:
    """Detector of collisions between bodies.

    Parameters
    ----------
    col_id : int
        Index of the physics server used for collision queries.
    bodies : dict
        A dictionary with body names as keys and corresponding UIDs as values.
    named_collision_pairs : list
        A list of pairs of ``NamedCollisionObject``.
    """
    def __init__(self, col_id, bodies, named_collision_pairs):
        self.col_id = col_id
        self.bodies = bodies
        self.indexed_collision_pairs = index_collision_pairs(
            self.col_id, bodies, named_collision_pairs
        )

    def compute_distances(self, q=None, max_distance=1.0):
        """Compute closest distances for a given configuration.

        Parameters
        ----------
        q : iterable
            The desired configuration. This is applied directly to PyBullet
            body with index ``bodies["robot"]``.
        max_distance : float
            Bodies farther apart than this distance are not queried by
            PyBullet, the return value for the distance between such bodies
            will be ``max_distance``.

        Returns
        -------
        :
            An array of distances, one per pair of collision objects.
        """

        # put the robot in the given configuration
        if q is not None:
            robot_id = self.bodies["robot"]
            for i in range(
                pyb.getNumJoints(robot_id, physicsClientId=self.col_id)
            ):
                pyb.resetJointState(
                    robot_id, i, q[i], physicsClientId=self.col_id
                )

        # compute shortest distances between all object pairs
        distances = []
        for a, b in self.indexed_collision_pairs:
            closest_points = pyb.getClosestPoints(
                a.body_uid,
                b.body_uid,
                distance=max_distance,
                linkIndexA=a.link_uid,
                linkIndexB=b.link_uid,
                physicsClientId=self.col_id,
            )

            # if bodies are above max_distance apart, nothing is returned, so
            # we just saturate at max_distance. Otherwise, take the minimum
            if len(closest_points) == 0:
                distances.append(max_distance)
            else:
                distances.append(np.min([pt[8] for pt in closest_points]))

        return np.array(distances)

    def in_collision(self, q=None, margin=0):
        """Check if a configuration is in collision.

        Parameters
        ----------
        q : iterable
            The desired configuration of the robot.
        margin : float
            Distance at which objects are considered in collision. Default is
            ``0``.

        Returns
        -------
        :
            True if configuration q is in collision, False otherwise.
        """
        ds = self.compute_distances(q, max_distance=margin)
        return (ds < margin).any()
