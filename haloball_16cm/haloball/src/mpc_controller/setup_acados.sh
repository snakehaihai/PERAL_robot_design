#!/bin/bash

set -e

cd

# Check if virtualenv command exists
if ! command -v virtualenv &> /dev/null; then
    echo "Installing python3-virtualenv..."
    sudo apt-get update
    sudo apt-get install -y python3-virtualenv
else
    echo "python3-virtualenv is already installed."
fi

# Install Acados
if [ ! -d ~/acados/build ]; then
    echo "Installing Acados ..."
    git clone https://github.com/acados/acados.git
    cd acados
    # Specific commit (remove to change to newest version)
    git checkout 00ca3a45b86f7f4ef5677d76fc912ed890962c4a
    git submodule update --recursive --init
    mkdir -p build
    cd build
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=GENERIC
    make install -j$(nproc) # use all cores
    cd ../bin
    wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux  # download file
    mv t_renderer-v0.0.34-linux t_renderer
    chmod +x t_renderer
    cd

    virtualenv env --python=/usr/bin/python3
    source ~/env/bin/activate
    pip install -e ~/acados/interfaces/acados_template

    echo "Acados is installed."
else
    echo "Acados already exists, skipping installation."
fi