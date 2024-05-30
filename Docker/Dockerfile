FROM quay.io/pypa/manylinux1_x86_64

# Install necessary system packages and dependencies
RUN yum install -y \
    git \
    cmake \
    gcc \
    gcc-c++ \
    make \
    wget \
    tar \
    bzip2

# Install miniforge


ENV PATH /opt/conda/bin:$PATH

# Create and activate a Conda environment with Python 3.8
RUN conda create -n build_env python=3.8 \
    && echo "source activate build_env" > ~/.bashrc
ENV PATH /opt/conda/envs/build_env/bin:$PATH

# Install Eigen 3.3.3
RUN wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.tar.gz \
    && tar -xf eigen-3.3.3.tar.gz \
    && cd eigen-3.3.3 \
    && mkdir build && cd build \
    && cmake .. \
    && make install \
    && cd ../.. \
    && rm -rf eigen-3.3.3 eigen-3.3.3.tar.gz

# Install Boost (latest version)
RUN yum install -y boost-devel

# Install OMPL (latest version)
RUN yum install -y epel-release \
    && yum install -y ompl ompl-devel

# Install yaml-cpp (latest version)
RUN wget -q https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.tar.gz \
    && tar -xf yaml-cpp-0.7.0.tar.gz \
    && cd yaml-cpp-yaml-cpp-0.7.0 \
    && mkdir build && cd build \
    && cmake .. \
    && make install \
    && cd ../.. \
    && rm -rf yaml-cpp-yaml-cpp-0.7.0 yaml-cpp-0.7.0.tar.gz

# Install glog 0.6.0
RUN wget -q https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz \
    && tar -xf v0.6.0.tar.gz \
    && cd glog-0.6.0 \
    && mkdir build && cd build \
    && cmake .. \
    && make install \
    && cd ../.. \
    && rm -rf glog-0.6.0 v0.6.0.tar.gz

# Install Python build tools in the Conda environment
RUN /opt/conda/envs/build_env/bin/pip install setuptools wheel auditwheel