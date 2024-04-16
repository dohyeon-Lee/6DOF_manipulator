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
cmake ..
make
```
Then ```actuate_basic.exe```, ```actuate_robotics.exe```, ```actuate_velocity_traj.exe``` will be made in ```/build``` directory.

* execution
  * before execution, you have to do USB port open & change USB latency
    ```
    sudo chmod a+rw /dev/ttyUSB0
    ```
    ```
    sudo gedit /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    ```
    Then gedit file open, change number 16 -> 1 in file and save.
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
     
    
