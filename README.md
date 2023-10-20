# pyb_utils: utilities for PyBullet

This is a collection of utilities I've found useful for working with PyBullet,
including:
* Collision detection: conveniently set up shortest distance computations and
  collision checking between arbitrary objects in arbitrary configurations with
  PyBullet. See the accompanying [blog post](https://adamheins.com/blog/collision-detection-pybullet).
* Ghost objects: add purely visual objects to the simulation, optionally
  attached to another body.
* Camera: virtual camera from which to get RGBA, depth, segmentation, and point
  cloud data. Also provides video recording using OpenCV.
* Convenience class for easily creating rigid bodies.
* Versions of some PyBullet functions that return *named* tuples, for easy
  field access.
* Basic quaternion functions.

## Install and run
This package supports **Python 3.8–3.11**. It has been tested on Ubuntu 16.04,
18.04, and 20.04.

### From pip
```
pip install pyb_utils
```

### From source
Clone the repo:
```bash
git clone https://github.com/adamheins/pyb_utils
cd pyb_utils
```

Install using [poetry](https://python-poetry.org/):
```bash
poetry install
poetry run python examples/collision_detection_example.py  # for example
```

Or using pip:
```bash
python -m pip install .
```

## Documentation

The project's documention is available [here](https://pyb-utils.readthedocs.io).

## Usage and examples
This package provides a few basic quality of life utilities. First, PyBullet
represents rotations using quaternions (in `[x, y, z, w]` order). We provide a
few helper routines to convert to rotation matrices and rotate points (using
[spatialmath](https://github.com/bdaiinstitute/spatialmath-python) under the
hood):
```python
>>> import pyb_utils
>>> q = (0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2)  # 90 deg rotation about z-axis

>>> pyb_utils.quaternion_to_matrix(q)  # convert to rotation matrix
array([[-0.,  -1.,  0.],
       [ 1.,  -0.,  0.],
       [ 0.,   0.,  1.]])

>>> pyb_utils.quaternion_multiply(q, q)  # rotate two quaternions together
array([0, 0, -1, 0])                     # 180 deg rotate about z

>>> pyb_utils.quaternion_rotate(q, [1, 0, 0])  # rotate a point
array([0, 1, 0])
```

Second, we provide a simple class to quickly create rigid bodies
programmatically, which is useful for adding basic objects to manipulate or act
as obstacles:
```python
>>> import pybullet as pyb
>>> import pyb_utils

>>> pyb.connect(pyb.GUI)

# create a 1x1x1 cube at the origin
>>> box = pyb_utils.BulletBody.box(position=[0, 0, 0], half_extents=[0.5, 0.5, 0.5])

# put a ball on top
>>> ball = pyb_utils.BulletBody.sphere(position=[0, 0, 1.5], radius=0.5)

# now put it somewhere else
>>> ball.set_pose(position=[2, 0, 0.5])
```

Third, we wrap some PyBullet functions to return *named* tuples, rather than
normal tuples. When the tuples have 10+ fields in them, it is rather helpful to
have names! The names and arguments of these functions are the same as the
underlying PyBullet ones, to make swapping effortless. Continuing our previous
example:
```python
# built-in PyBullet method
# the output is not easy to read!
>>> pyb.getDynamicsInfo(box.uid, -1)
(1.0,
 0.5,
 (0.16666666666666666, 0.16666666666666666, 0.16666666666666666),
 (0.0, 0.0, 0.0),
 (0.0, 0.0, 0.0, 1.0),
 0.0,
 0.0,
 0.0,
 -1.0,
 -1.0,
 2,
 0.001)

# switch to the pyb_utils version
# now we can access fields by name
>>> info = pyb_utils.getDynamicsInfo(box.uid, -1)
>>> info.mass
1.0
>>> info.localInertiaPos
(0.0, 0.0, 0.0)
```
The functions we've wrapped in this way are `getClosestPoints`,
`getConstraintInfo`, `getContactPoints`, `getDynamicsInfo`, `getJointInfo`,
`getJointState(s)`, and `getLinkState(s)`. There are two differences from the
vanilla PyBullet API. The first is that in pyb_utils `getJointInfo` also
accepts an optional argument `decode`, which will convert the byte strings
returned by PyBullet to the specifed encoding. For example, `decode="utf8"`.
The second difference is that in pyb_utils `getLinkState(s)` will always return
`LinkState` tuples with 8 fields, even if `computeLinkVelocity=False`. When
`computeLinkVelocity=False`, then `worldLinkLinearVelocity` and
`worldLinkAngularVelocity` are both set to `None`.

And there's more! You can find example scripts of all of this package's
utilities in the `examples/` directory:

* [rigid bodies](examples/bodies_example.py)
* [camera](examples/camera_example.py)
* [collision detection](examples/collision_detection_example.py)
* [ghost objects](examples/ghost_object_example.py)
* [named tuples](examples/named_tuples_example.py)
* [video](examples/video_example.py)

## Known issues
Feel free to open issues (or better yet, a pull request!) if you find a
problem. Currently known issues:

* Video recording does not output MP4 videos correctly. The AVI format works,
  however.
* Ghost objects sometimes flicker (spooky, but undesirable).

## Development
* Run `tox` to run the tests.
* Sphinx is used to build the documentation. With Sphinx installed, run `make
  html` in the `docs` directory.

## License
[MIT](https://github.com/adamheins/pyb_utils/blob/main/LICENSE)
