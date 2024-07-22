# Efficient-Path-Planner
## Installation
### Install Dependencies
To use this code, relevant other packages must be installed before. Below you will find a list of instructions

**Install Submodules**
One external dependency (Pybind11) must be added to this codebase in the form of a Git Submodule. (The other external dependencies for path planning are already provided within this repository). To install the submodule run
```
git submodule init
git submodule update --remote
```

If you have not cloned this repo yet, you can also download all submodules during the clone via
```
git clone --recurse-submodules https://github.com/Autonomous-Drone-Racing-Lab/Efficient-Path-Planner
```

**Install Eigen**
```
sudo apt install libeigen3-dev
```

**Install glog**

From outside this directory e.g. `code` directory, run
```
# Fetch glog in version 6
git clone https://github.com/google/glog.git --branch v0.6.0
cd glog

cmake -S . -B build -G "Unix Makefiles"
cmake --build build
cmake --build build --target install
```

**Install YAML CPP**
```
sudo apt-get install libyaml-cpp-dev
```

**Install OMPL**

Follow the tutorial on the [ompl website](https://ompl.kavrakilab.org/installation.html)

### Build Code
To generate the python binding simply run
```
pip install .
```
from the root of this package. This makes the path planning package available to python via the name `polynomial_trajectory`, i.e. `import polynomial_trajectory`


**Important: In case the first run fails, you must first setup the build system.** For this follow the steps:
```
mkdir build
cd build
cmake ..
```

As build errors are obfuscated in the `pip install .` command for debugging purposes it is recommended to do a normal C-Make build for debugging
```
cd build
make
```

## Third Party Software
Within our work, we utilized different software of other people. Important to mention are
- [Mav Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) for experimenting with minimum snap trajectories
- [Tobias Kunz](https://github.com/tobiaskunz/trajectories) for providing implementations of his time-parametrization algorithm




