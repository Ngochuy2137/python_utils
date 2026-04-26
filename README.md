# python_utils

Small Python helpers (printing, plotting, file utilities, optional ROS helpers).

## Installation

From the repository root of this package:

```bash
cd /path/to/python_utils
pip install .
```

Or editable install while developing:

```bash
pip install -e .
```

Installing with `pip install .` pulls in **runtime dependencies** declared in `setup.py` (see below). If you only copy the source tree onto `PYTHONPATH` without installing, you must install those packages yourself.

### Dependencies (declared in `setup.py`)

| Package      | Used by |
|-------------|---------|
| **numpy**   | `math`, `plotter`, `prediction_plotter` |
| **matplotlib** | `plotter` (`matplotlib.pyplot`, `mpl_toolkits.mplot3d`) |
| **plotly**  | `plotter`, `prediction_plotter` |

One-line install (if you are not using `pip install .`):

```bash
pip install "numpy>=1.20" "matplotlib>=3.5" "plotly>=5.0"
```

### Common error: `No module named 'mpl_toolkits'`

`mpl_toolkits` is part of **matplotlib** (3D axes). Install matplotlib in the same environment you use to run your scripts:

```bash
pip install matplotlib
```

On Debian/Ubuntu you can alternatively use:

```bash
sudo apt install python3-matplotlib
```

### Optional: ROS

`python_utils/ros_node_handle.py` imports `rospy` and `visualization_msgs`. These come from a ROS distribution, not PyPI. Only install/use that module in a ROS workspace; there is no standard `pip` dependency for it.

### Import / PyPI name clash

PyPI also has distributions named like `python-utils` / `python_utils` that are **unrelated** to this repository. If `import python_utils` resolves to a site-packages copy, you may get errors such as `No module named 'python_utils.subutils'`.

**Fix:** install this project in your environment (`pip install -e .` from this folder), **or** put the **setuptools project root** on `PYTHONPATH` (the directory that contains `setup.py` and the inner package folder `python_utils/`), not only a parent monorepo path.

## Usage

Prefer a normal install:

```bash
pip install -e .
```

Then:

```python
from python_utils.plotter import Plotter
from python_utils.printer import Printer
```

If you rely on `PYTHONPATH` instead, add the directory that contains `setup.py` (this README’s parent), for example:

```bash
export PYTHONPATH="/path/to/python_utils:$PYTHONPATH"
```
