# Collision Detection in PyBullet

Conveniently set up shortest distance computations and collision checking
between arbitrary objects in arbitrary configurations with PyBullet. See the
accompanying [blog post](https://adamheins.com/blog/collision-detection-pybullet).

## Install and run
This code should be run with Python 3.7+.

Clone the repo:
```bash
git clone https://github.com/adamheins/collision-detection-pybullet
cd collision-detection-pybullet
```

If you have [pipenv](https://pypi.org/project/pipenv/), you can do
```bash
pipenv install
pipenv run python main.py
```

Otherwise, do
```bash
pip install -r requirements.txt
python main.py
```
where the `pip` and `python` executables are Python 3.7+.

## License
MIT
