## Client

Install submodules

```bash
# install libfreenect
sudo apt update
sudo apt install -y git cmake build-essential libusb-1.0-0-dev pkg-config
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
make -j"$(nproc)"
sudo make install
sudo ldconfig

# clone repo
cd ..
git clone https://github.com/pj0620/gpt-pet-v2.git
cd gpt-pet-v2

# submodules
git submodule update --init --recursive

# Install ROS2 Humble (see site)
# ..steps...

# Install colcon
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# ROS Control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# ROS Dep
cd ros_ws/src/bot_ws
rosdep install --from-paths src --ignore-src -r -y

# Ros Dep


```