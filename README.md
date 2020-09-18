# Lensdriver

This application demonstrates how to use lens driver to control the lens on a
Boulder AI camera using i2c commands.

## Setup

Two motor driver interfaces are currently supported.  For dnncam version 1, no setup
is required.  For NX and bullet cameras, the /data/production/lens/motor-driver directory
must be configured appropriately.  Refer to release documentation for more details.

The lensdriver application supports persistent storage for JSON configuration in files
stored within the rootfs.  Please see the #define definitions in motordriverbase.cpp 
for relevant paths and their use.  You will need to create these paths to support persistent
storage of parameters

## Building

Install dependencies:
```
sudo apt-get -y update && sudo apt-get install -y \
    cmake \
    build-essential \
    libboost-all-dev \
    libi2c-dev i2c-tools \
    libreadline-dev
```

Then 
```
mkdir build && cd build && cmake .. && make
```
