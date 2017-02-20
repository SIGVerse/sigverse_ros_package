## Setup

### Install CMake 3.2 or later

```bash:
$ cd ~/Downloads
$ wget https://cmake.org/files/v3.7/cmake-3.7.2.tar.gz
$ tar zxvf cmake-3.7.2.tar.gz
$ cd cmake-3.7.2
$ ./configure
$ make
$ sudo make install
```

### Install Mongo C Driver

```bash:
$ cd ~/Downloads
$ wget https://github.com/mongodb/mongo-c-driver/releases/download/1.4.2/mongo-c-driver-1.4.2.tar.gz
$ tar zxvf mongo-c-driver-1.4.2.tar.gz
$ cd mongo-c-driver-1.4.2
$ ./configure
$ make
$ sudo make install
```

### Install Mongo C++ Driver

```bash:
$ cd ~/Downloads
$ wget https://github.com/mongodb/mongo-cxx-driver/archive/r3.0.3.tar.gz
$ tar zxvf r3.0.3.tar.gz
$ cd mongo-cxx-driver-r3.0.3/build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DLIBMONGOC_DIR=/usr/local -DLIBBSON_DIR=/usr/local ..
$ sudo make EP_mnmlstc_core
$ make
$ sudo make install
```

### Install SIGVerse ROS Bridge

```bash:
$ cd ~/catkin_ws/src
$ git clone http://fs.iir.nii.ac.jp/sigverse/sigverse_ros_bridge.git
$ cd ..
$ catkin_make
```


## How to use

```bash:
$ rosrun sigverse_ros_bridge sigverse_ros_bridge
```

Default port number is 50001.  
If you need change the port number, please pass the argument.

```bash
$ rosrun sigverse_ros_bridge sigverse_ros_bridge 12345
```
