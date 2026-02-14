# Dockerfile for DexCap with CUDA support
# Based on Ubuntu 20.04 for deoxys_controller compatibility
# CUDA 12.4 (compatible with NVIDIA driver 555+)

FROM nvidia/cuda:12.4.0-devel-ubuntu20.04

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies including Python 3.10
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-dev \
    make \
    git \
    build-essential \
    pkg-config \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libegl1 \
    libgles2 \
    libosmesa6 \
    wget \
    curl \
    software-properties-common \
    libpoco-dev \
    libeigen3-dev \
    libfmt-dev \
    lsb-release \
    iputils-ping \
    vim \
    python3-pip

# Install CMake 3.24.3 (Ubuntu 20.04 default is older)
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.3/cmake-3.24.3-linux-x86_64.sh && \
    chmod +x cmake-3.24.3-linux-x86_64.sh && \
    ./cmake-3.24.3-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.24.3-linux-x86_64.sh


#RUN mkdir -p /etc/apt/keyrings && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list # buildkit

# This package seems to have installation errors now, maybe there were recent updates
#RUN apt-get update && apt-get install -y \
#    robotpkg-pinocchio \
#    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy requirements first for better caching
COPY DexCap/requirements.txt /workspace/
COPY DexCap/pyproject.toml /workspace/
COPY DexCap/.python-version /workspace/
COPY deoxys_control/deoxys/requirements.txt /workspace/requirements_deoxys.txt 

WORKDIR /workspace/DexCap
# Create virtual environment using Python 3.10
RUN uv init && uv venv

# Add venv to PATH so it's automatically used
ENV PATH="/workspace/DexCap/.venv/bin:${PATH}"
#ENV VIRTUAL_ENV="/workspace/DexCap/.venv"

WORKDIR /workspace/
# Install Python dependencies using uv (will use venv automatically)
RUN uv pip install -r requirements.txt
RUN uv pip install -r requirements_deoxys.txt
RUN uv pip install "protobuf==3.13.0"
# Install PyTorch with CUDA support (after other packages for better compatibility)
# RUN uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124

# Clone and install deoxys_control (if needed, can be done at runtime)
# The workspace directory structure should be set up before installing deoxys
# Users will need to clone deoxys_control to the appropriate location

# Set up environment for deoxys_controller
ENV PYTHONPATH=/workspace:${PYTHONPATH}

# COPY deoxys_control/deoxys/InstallPackage /workspace/
# RUN echo "0.15.0" | sh /workspace/InstallPackage

# Default command
CMD ["/bin/bash"]
