#!/bin/bash

set -euo pipefail

SUDO=""
if (( EUID != 0 )); then
  SUDO="sudo"
fi

echo " - Install dependencies"
# ORB_SLAM2 (alsora fork) is built outside of ROS and only needs OpenCV/Eigen/etc.
# On Ubuntu 22.04 (ROS 2 Humble), Foxy binary packages do not apply.
if [[ "${SKIP_APT:-0}" != "1" ]]; then
  $SUDO apt update || echo " ! Warning: 'apt update' failed (likely due to a broken third-party repo). Continuing with existing package indexes."
  $SUDO apt install -y \
    build-essential cmake git pkg-config \
    python3 python3-pip python3-setuptools \
    libeigen3-dev libopencv-dev \
    libgl1-mesa-dev libglew-dev libegl1-mesa-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev \
    libsuitesparse-dev
else
  echo " - SKIP_APT=1 set; skipping apt install"
fi

echo "- Create libs folder"
mkdir -p ../libs
cd ../libs

if [[ "${BUILD_PANGOLIN:-0}" == "1" ]]; then
  echo " - Install Pangolin (optional)"
  if [[ ! -d Pangolin ]]; then
    git clone https://github.com/stevenlovegrove/Pangolin.git
  fi
  cmake -S Pangolin -B Pangolin/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_PANGO_PYTHON=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DCMAKE_INSTALL_PREFIX="$(pwd)/Pangolin/install"
  cmake --build Pangolin/build -j"$(nproc)"
  cmake --install Pangolin/build
else
  echo " - Skipping Pangolin (not required for alsora/ORB_SLAM2 fork)"
fi

if [[ "${BUILD_G2O:-0}" == "1" ]]; then
  echo " - Install g2o (optional)"
  if [[ ! -d g2o ]]; then
    git clone https://github.com/RainerKuemmerle/g2o.git
  fi
  cmake -S g2o -B g2o/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(pwd)/g2o/install"
  cmake --build g2o/build -j"$(nproc)"
  cmake --install g2o/build
else
  echo " - Skipping g2o (ORB_SLAM2 builds its own Thirdparty/g2o)"
fi

echo " - Install ORB SLAM 2 (modified version w/o pangolin)"
# Always keep ORB_SLAM2 inside this repo's libs folder:
#   <repo>/libs/ORB_SLAM2
if [[ ! -d ORB_SLAM2 ]]; then
  git clone https://github.com/alsora/ORB_SLAM2 ORB_SLAM2
fi
cd ORB_SLAM2

# Help CMake find OpenCV on Ubuntu 22.04 (opencv4 config package).
if [[ -z "${OpenCV_DIR:-}" ]] && [[ -f /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake ]]; then
  export OpenCV_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4"
fi

# Patch DBoW2 OpenCV detection to support OpenCV4.
DBOW2_CMAKE="Thirdparty/DBoW2/CMakeLists.txt"
if [[ -f "${DBOW2_CMAKE}" ]]; then
  if grep -q "find_package(OpenCV 3.0" "${DBOW2_CMAKE}"; then
    echo " - Patching DBoW2 OpenCV find_package()"
    cp "${DBOW2_CMAKE}" "${DBOW2_CMAKE}.bak"
    python3 - <<'PY'
from pathlib import Path
p = Path("Thirdparty/DBoW2/CMakeLists.txt")
s = p.read_text()
s2 = s.replace(
    "find_package(OpenCV 3.0 QUIET)\n"
    "if(NOT OpenCV_FOUND)\n"
    "   find_package(OpenCV 2.4.3 QUIET)\n"
    "   if(NOT OpenCV_FOUND)\n"
    "      message(FATAL_ERROR \"OpenCV > 2.4.3 not found.\")\n"
    "   endif()\n"
    "endif()\n",
    "find_package(OpenCV REQUIRED)\n",
)
if s2 != s:
    p.write_text(s2)
PY
  fi
fi

# Patch ORB_SLAM2 top-level OpenCV requirement (OpenCV4 is fine).
TOP_CMAKE="CMakeLists.txt"
if [[ -f "${TOP_CMAKE}" ]]; then
  if grep -q "find_package(OpenCV 3.0 REQUIRED)" "${TOP_CMAKE}"; then
    echo " - Patching ORB_SLAM2 OpenCV find_package()"
    cp "${TOP_CMAKE}" "${TOP_CMAKE}.bak"
    python3 - <<'PY'
from pathlib import Path
p = Path("CMakeLists.txt")
s = p.read_text()
s2 = s.replace("find_package(OpenCV 3.0 REQUIRED)", "find_package(OpenCV REQUIRED)")
if s2 != s:
    p.write_text(s2)
PY
  fi
fi

# Configure + build (avoid relying on repo's build.sh).
rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="${OpenCV_DIR:-}" \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"
cmake --build build -j"$(nproc)"

# Unpack vocabulary if needed
if [[ -f Vocabulary/ORBvoc.txt.tar.gz ]] && [[ ! -f Vocabulary/ORBvoc.txt ]]; then
  tar -C Vocabulary -xf Vocabulary/ORBvoc.txt.tar.gz
fi

echo
echo "Done. In new terminals, set:"
echo "  export ORB_SLAM2_ROOT_DIR=$(pwd)"
