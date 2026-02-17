#!/usr/bin/env bash

# Install dependencies for ROS2 Humble on Ubuntu 22.04

# Install development libraries (pybind11-dev and python3-pybind11 for ROS2, not pybind11-catkin)
sudo apt install -y libcgal-qt5-dev libcgal-dev libcgal-demo libeigen3-dev pybind11-dev python3-pybind11

# Install FAISS
INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/facebookresearch/faiss.git
cd faiss
git checkout v1.6.3
./configure --prefix=$INSTALL_DIR --without-cuda
make install

# Install Gurobi
wget https://packages.gurobi.com/9.1/gurobi9.1.1_linux64.tar.gz
tar xvfz gurobi9.1.1_linux64.tar.gz
sudo cp -r gurobi911/ /opt/

rm -rf gurobi911/
rm -rf gurobi9.1.1_linux64.tar.gz

echo ' ' >> ~/.bashrc
echo '# Gurobi setup - added by cdcpd/install_scripts/install_for_humble_22.sh' >> ~/.bashrc
echo 'export GUROBI_HOME=/opt/gurobi911/linux64' >> ~/.bashrc
echo 'export PATH=${PATH}:${GUROBI_HOME}/bin' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib' >> ~/.bashrc

echo ' ' >> ~/.bashrc
echo '# Local dependency paths' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=~/.local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export PATH=~/.local/bin:${PATH}' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=~/.local:$CMAKE_PREFIX_PATH' >> ~/.bashrc

source ~/.bashrc

echo "Dependencies installation completed!"
