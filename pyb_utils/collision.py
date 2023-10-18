"""This module provides utilities for collision checking between objects."""
from collections import namedtuple

import numpy as np
import pybullet as pyb

from .named_tuples import getJointInfo, getClosestPoints


_IndexedCollisionObject = namedtuple(
    "_IndexedCollisionObject", ["body_uid", "link_uid"]
)


def _index_collision_pairs(physics_uid, collision_pairs):
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
        A list of pairs of ``_IndexedCollisionObject``.
    """

    def _get_link_index(body_uid, link_name):
        n = pyb.getNumJoints(body_uid, physics_uid)
        for i in range(n):
            if (
                getJointInfo(body_uid, i, physics_uid, decode="utf8").linkName
                == link_name
            ):
                return i
        raise ValueError(
            f"Body with UID {body_uid} has no link named {link_name}"
        )

    def _index_named_collision_object(obj):
        """Map body and link names to corresponding indices."""
        if type(obj) == int:
            body_uid = int(obj)
            link_uid = -1
        else:
            body_uid = obj[0]
            if type(obj[1]) == int:
                link_uid = obj[1]
            elif type(obj[1]) == str:
                link_uid = _get_link_index(body_uid, obj[1])
            else:
                raise TypeError(f"Invalid type for link identifier {obj[1]}")
        return _IndexedCollisionObject(body_uid, link_uid)

    # convert all pairs of named collision objects to indices
    indexed_collision_pairs = []
    for a, b in collision_pairs:
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
    collision_pairs : list
        A list of collision pairs, where each element is either a single
        ``int`` representing a body UID, a tuple ``(int, int)`` representing
        the body UID and link index, or a tuple ``(int, str)`` representing the
        body UID and the link name.
    """

    def __init__(self, col_id, collision_pairs):
        self.col_id = col_id
        self.indexed_collision_pairs = _index_collision_pairs(
            self.col_id, collision_pairs
        )

    def compute_distances(self, max_distance=10.0):
        """Compute closest distances for a given configuration.

        Parameters
        ----------
        max_distance : float
            Bodies farther apart than this distance are not queried by
            PyBullet, the return value for the distance between such bodies
            will be ``max_distance``.

        Returns
        -------
        :
            An array of distances, one per pair of collision objects.
        """

        # compute shortest distances between all object pairs
        distances = []
        for a, b in self.indexed_collision_pairs:
            closest_points = getClosestPoints(
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
                distances.append(
                    np.min([pt.contactDistance for pt in closest_points])
                )

        return np.array(distances)

    def in_collision(self, margin=0):
        """Check if a configuration is in collision.

        Parameters
        ----------
        margin : float
            Distance at which objects are considered in collision. Default is
            ``0``.

        Returns
        -------
        :
            True if configuration q is in collision, False otherwise.
        """
        ds = self.compute_distances(max_distance=margin + 0.1)
        return (ds < margin).any()
