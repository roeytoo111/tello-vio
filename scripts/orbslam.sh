#!/bin/bash

set -euo pipefail

SUDO=""
if (( EUID != 0 )); then
  SUDO="sudo"
fi

echo " - Install dependencies"
# ORB_SLAM2 (alsora fork) is built outside of ROS and only needs OpenCV/Eigen/etc.
# On Ubuntu 22.04 (ROS 2 Humble), Foxy binary packages do not apply.
$SUDO apt update || echo " ! Warning: 'apt update' failed (likely due to a broken third-party repo). Continuing with existing package indexes."
$SUDO apt install -y \
  build-essential cmake git pkg-config \
  libeigen3-dev libopencv-dev \
  libgl1-mesa-dev libglew-dev libegl1-mesa-dev \
  libwayland-dev libxkbcommon-dev wayland-protocols \
  ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
  libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev \
  libsuitesparse-dev

echo "- Create libs folder"
mkdir -p ../libs
cd ../libs

echo " - Install Pangolin"
if [[ ! -d Pangolin ]]; then
  git clone https://github.com/stevenlovegrove/Pangolin.git
fi
cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release
cmake --build Pangolin/build -j"$(nproc)"
$SUDO cmake --install Pangolin/build

echo " - Install G2O"
if [[ ! -d g2o ]]; then
  git clone https://github.com/RainerKuemmerle/g2o.git
fi
cmake -S g2o -B g2o/build -DCMAKE_BUILD_TYPE=Release
cmake --build g2o/build -j"$(nproc)"
$SUDO cmake --install g2o/build

cd ../..

echo " - Install ORB SLAM 2 (modified version w/o pangolin)"
if [[ ! -d ORB_SLAM2 ]]; then
  git clone https://github.com/alsora/ORB_SLAM2
fi

# Help older CMake logic locate OpenCV 4 on Ubuntu 22.04.
if [[ -f /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake ]]; then
  export OpenCV_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4"
fi

cd ORB_SLAM2
chmod +x build.sh
./build.sh

echo "export ORB_SLAM2_ROOT_DIR=$(pwd)" >> ~/.bashrc
