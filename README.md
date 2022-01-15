# pyb_utils: utilities for PyBullet

This is a collection of utilities I've found useful for working with PyBullet,
including:
* Collision detection: conveniently set up shortest distance computations and
  collision checking between arbitrary objects in arbitrary configurations with
  PyBullet. See the accompanying [blog post](https://adamheins.com/blog/collision-detection-pybullet).
* Ghost objects: add purely visual objects to the simulation, optionally
  attached to another body.

## Install and run
This package requires **Python 3.7+**.

### From source
Clone the repo:
```bash
git clone https://github.com/adamheins/collision-detection-pybullet
cd collision-detection-pybullet
```

Install using [poetry](https://python-poetry.org/):
```bash
poetry install
poetry run python scripts/run_collision_detection.py  # for example
```

### Using pip
TODO

## License
MIT
