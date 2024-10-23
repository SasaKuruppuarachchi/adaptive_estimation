# Pixi

install: https://pixi.sh/latest/basic_usage/

### Init (if not already done in the repo. I.e. if there's no `tpc` directory)

```bash
pixi init tpc
cd tpc
```

Activate env with: `pixi shell`

### Package installation
```bash
pixi add <package>
```
or edit the `pixi.toml` file directly
```
...
[dependencies]
numpy = ">=3.1.1,<3"
hydra-core = ">=1.3.2,<2"
python = ">=3.12.6,<4"
...

```

### Usage
Go wild

# Gotchas with ROS2

## Building custom services

When building custom a service I ran into cmake not using the pixi environment python. 
Instead using the system python. To fix this I did:
```bash
export PYTHON_EXECUTABLE=$(which python)   # Path to the Python executable in your environment
export Python3_EXECUTABLE=$(which python)  # Ensure CMake uses the same Python
export Python3_LIBRARY=$(python3-config --libs | cut -d' ' -f1 | sed 's/-L//')  # Path to Python 3.10 library
export Python3_INCLUDE_DIR=$(python3-config --includes | cut -d' ' -f1 | sed 's/-I//')  # Path to Python 3.10 includes
export Python3_NumPy_INCLUDE_DIRS=$(python -c "import numpy; print(numpy.get_include())")  # Path to NumPy includes
```

and then running the build command. 
```bash
colcon build --packages-select tpc_ros --cmake-args \
  -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE \
  -DPython3_EXECUTABLE=$Python3_EXECUTABLE \
  -DPython3_LIBRARY=$Python3_LIBRARY \
  -DPython3_INCLUDE_DIR=$Python3_INCLUDE_DIR \
  -DPython3_NumPy_INCLUDE_DIRS=$Python3_NumPy_INCLUDE_DIRS
```

This left me with an error:
```
CMake Error at /home/nicklas/Projects/adaptive_estimation/tpc/.pixi/envs/default/share/cmake-3.26/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
  Could NOT find Python3 (missing: Development NumPy Development.Module
  Development.Embed) (found version "3.10.13")
```

which was fixed with updating the `CMakeLists.txt` file to include the following:
```cmake
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
```
