# Docker Setup Guide for DexCap

This guide explains how to set up and use the DexCap project in a Docker container with CUDA support.

## Prerequisites

1. **Docker**: Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/) or Docker Engine
2. **NVIDIA Container Toolkit**: Required for CUDA support
   - Follow instructions at: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
   - For Windows with WSL2, install in your WSL2 Ubuntu distribution
3. **NVIDIA Driver**: Version 555+ (already installed)

## Quick Start

### Build the Docker Image

```bash
docker-compose build
```

Or using Docker directly:

```bash
docker build -t dexcap:latest .
```

### Run the Container

```bash
docker-compose up -d
docker-compose exec dexcap bash
```

Or using Docker directly:

```bash
docker run --gpus all -it --rm \
  -v /path/to/your/workspace:/workspace \
  dexcap:latest bash
```

## Installing deoxys_controller

Once inside the container, you need to install `deoxys_controller`:

```bash
# Navigate to the workspace
cd /workspace/src

# Clone deoxys_control (if not already cloned)
git clone git@github.com:montrealrobotics/deoxys_control.git

# Install deoxys
cd deoxys_control/deoxys
./InstallPackage
make -j build_deoxys=1
pip install -e .

# Install protobuf
cd protobuf/python
pip install -e .
```

## Using the Container

### Running Inference/Training

```bash
# Inside the container
cd /workspace/src/DexCap

# Example: Run inference
cd STEP3_inference
python test_policy.py
```

### Accessing GPU

Verify CUDA is available:

```bash
python3 -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

### Volume Mounts

The `docker-compose.yml` mounts:
- `../../` -> `/workspace` (entire workspace)
- Creates a persistent `dexcap_data` volume

Adjust volumes in `docker-compose.yml` as needed.

## Troubleshooting

### CUDA not detected

1. Verify NVIDIA Container Toolkit is installed:
   ```bash
   docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu20.04 nvidia-smi
   ```

2. Check driver compatibility:
   ```bash
   nvidia-smi
   ```

### Build issues with CMake

If CMake installation fails, the Dockerfile installs CMake 3.24.3 manually. If issues persist, check the logs:

```bash
docker-compose build --no-cache
```

### Permission issues

If you encounter permission issues with mounted volumes, you may need to adjust file permissions or use a user namespace.

## Notes

- The container uses Ubuntu 20.04 for `deoxys_controller` compatibility
- CUDA 12.4 is pre-installed (compatible with driver 555+)
- PyTorch with CUDA support is installed separately
- All Python dependencies from `requirements.txt` are installed via `uv`

