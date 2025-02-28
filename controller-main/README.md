## Description:
This repository contains mpc implementation of motion planning for omni-wheel drive robot, with focus on RoboCup MSL Competition.

## Dependencies:
1. Eigen3:
```bash
sudo apt-get update
sudo apt-get install libeigen3-dev
```

2. Boost:
```bash
sudo apt-get install libboost-all-dev
```

3. Acado:
```bash
git clone https://github.com/acado/acado.git
cd acado
mkdir build
cd build
cmake ..
make
sudo make install
```
You may follow acado/examples/getting_started for examples of Acado library

## Build and run the code:
```bash
cd model_predictive_control
colcon build
source install/local_setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run controller_package controller
```

## Issues :
The controller stops working/crashes if target x or y coordinates is made > 0.5  and  if theta >0.3 or 0.4