# Environment
* ubuntu 22.04 (recommend)
* C++ development environment
  ```
  sudo apt install build-essential
  ```
* Install eigen3
  ```
  sudo apt update -y
  sudo apt install libeigen3-dev
  ```
  Check install
  ```
  dpkg -S libeigen3-dev
  ```
  Please install other required dependencies which I missed too.

# build & execution
remove all file in ```/build``` directory then do
```
cd 6DOF_manipulator
cd build
cmake ..
make
```
Then ```actuate_basic.exe```, ```actuate_robotics.exe```, ```actuate_velocity_traj.exe``` will be made in ```/build``` directory.
If you change code, then just do ```make``` at ```/build``` directory. ```cmake ..``` only do when ```CMakeLists.txt``` file changed.

* execution
  * before execution, you have to do USB port open & change USB latency
    ```
    sudo chmod a+rw /dev/ttyUSB0
    ```
    ```
    sudo gedit /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    ```
    Then change number 16 -> 1 in file and save.
    These two works need just once time after connected usb.
  * go to ```/build``` directory and
    ```
    ./actuate_basic
    ```
    or
    ```
    ./actuate_robotics
    ```
    or
    ```
    ./actuate_velocity_traj
    ```
    Then robot will actuated.
    
# about code
### actuate_basic.cpp
* This code introduce how to actuate motors. You can control motor with position mode, velocity mode, torque mode.
* You can check how to add motors, get position data from encoders, change motor control mode. 
* Exe file is ```actuate_basic.exe```
* If your environment is under ubuntu 22.04, then you can execution only ```actuate_basic.exe```. ```actuate_robotics.exe``` and ```actuate_velocity_traj.exe``` only available in ubuntu 22.04 . It because one function ```seq``` in eigen3 is only work on 22.04.
* if your environment is under ubuntu 22.04, you also change ```6DOF_manipulator/CMakeLists.txt``` file little.

### actuate_robotics.cpp
* You can use function about robotics such as Fkinbody (forward kinematics), jacobian ...etc
* This code move robot endeffector point to point. You can change endeffector's trajectory.
* Exe file is ```actuate_robotics.exe```

### position_velocity.cpp
* Example code about velocity trajectory tracking. ```velocity_test.csv``` is velocity trajectory about endeffector, and manipulator's endeffector tracking that trajectory.
* Exe file is ```actuate_velocity_traj.exe```
