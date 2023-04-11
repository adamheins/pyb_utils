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

## Install and run
This package requires **Python 3.7+**. It has been tested on Ubuntu 16.04,
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
poetry run python scripts/collision_detection_example.py  # for example
```

Or using pip:
```bash
python -m pip install .
```

## Usage and examples
You can find example scripts demonstrating all of this package's utilities in
the `scripts/` directory:

* [collision detection](https://github.com/adamheins/pyb_utils/blob/main/scripts/collision_detection_example.py)
* [ghost objects](https://github.com/adamheins/pyb_utils/blob/main/scripts/ghost_object_example.py)
* [camera](https://github.com/adamheins/pyb_utils/blob/main/scripts/camera_example.py)
* [video](https://github.com/adamheins/pyb_utils/blob/main/scripts/video_example.py)

## Known issues
Feel free to open issues (or better yet, a pull request!) if you find a
problem. Currently known issues:

* Video recording does not output MP4 videos correctly. The AVI format works,
  however.
* Ghost objects sometimes flicker (spooky, but undesirable).

## License
[MIT](https://github.com/adamheins/pyb_utils/blob/main/LICENSE)
