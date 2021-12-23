import numpy as np
import pybullet as pyb
import pybullet_data


class NamedCollisionObject:
    def __init__(self, body_name, link_name=None):
        self.body_name = body_name
        self.link_name = link_name


class IndexedCollisionObject:
    def __init__(self, body_uid, link_uid):
        self.body_uid = body_uid
        self.link_uid = link_uid


def index_collision_pairs(physics_uid, bodies, named_collision_pairs):
    # bodies is a dict with names as keys and UIDs as values
    # named_collision_pairs is a list of tuples of CollisionObjects

    body_link_map = {}
    for name, uid in bodies.items():
        body_link_map[name] = {}
        n = pyb.getNumJoints(uid, physics_uid)
        for i in range(n):
            info = pyb.getJointInfo(uid, i, physics_uid)
            link_name = info[12].decode("utf-8")
            body_link_map[name][link_name] = i

    def _index_named_collision_object(obj):
        body_uid = bodies[obj.body_name]
        if obj.link_name is not None:
            link_uid = body_link_map[obj.body_name][obj.link_name]
        else:
            link_uid = -1
        return IndexedCollisionObject(body_uid, link_uid)

    indexed_collision_pairs = []
    for a, b in named_collision_pairs:
        a_indexed = _index_named_collision_object(a)
        b_indexed = _index_named_collision_object(b)
        indexed_collision_pairs.append((a_indexed, b_indexed))

    return indexed_collision_pairs


class CollisionDetector:
    def __init__(self, col_id, bodies, named_collision_pairs):
        self.col_id = col_id
        self.robot_id = bodies["robot"]
        self.indexed_collision_pairs = index_collision_pairs(
            self.col_id, bodies, named_collision_pairs
        )

    def compute_distances(self, q, max_distance=1.0, flatten=False):
        """Compute closest distances between specified pairs."""

        # put the robot in the given configuration
        for i in range(pyb.getNumJoints(self.robot_id, physicsClientId=self.col_id)):
            pyb.resetJointState(self.robot_id, i, q[i], physicsClientId=self.col_id)

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
            distances.append([pt[8] for pt in closest_points])

        if flatten:
            return np.array([d for dists in distances for d in dists])
        return distances

    def in_collision(self, q, margin=0):
        """Return True if system is in collision, False otherwise."""
        ds = self.compute_distances(q, max_distance=margin, flatten=True)
        return (ds <= margin).any()
