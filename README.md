# Environment
* ubuntu 22.04 (recommend)
* c++ development environment
  ```
  sudo apt install build-essential
  ```
* install eigen3
  ```
  sudo apt update -y
  sudo apt install libeigen3-dev
  ```
  check install
  ```
  dpkg -S libeigen3-dev
  ```

# build & execution
```
cd 6DOF_manipulator
cd build
```
```
cmake ..
```
```
make
```
Then ```actuate_basic.exe```, ```actuate_robotics.exe```, ```actuate_velocity_traj.exe``` will be made in ```/build``` directory.
