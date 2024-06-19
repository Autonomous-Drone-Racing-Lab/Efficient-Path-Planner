# Efficient-Path-Planner
## Installation
### Install Dependencies
To use this code, relevant other packages must be installed before. Below you will find a list of instructions

**Install Submodules**
Two external dependencies must be added to this codebase in the form of Git Submodules. Pybind11 for creating the pyhon bindings as well as a custom poly_traj package for generting minimum snap trajectories. Download both packages using
```
git submodule init
git submodule update
```

If you have not cloned this repo yet, you can also download all submodules during the clone via
```
git clone --recurse-submodules <repository_url>
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

### Build Code
To generate the python binding simply run
```
pip install .
```
from the root of the directory. This makes the package available via the name `polynomial_trajectory` in python, i.e. `import polynomial_trajectory`

As build errors are obfuscated in this command for debugging purposes it is recommended to do a normal C-Make build
```
mkdir build
cd build
cmake ..
make
```




