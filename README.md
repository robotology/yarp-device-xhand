# yarp-device-xhand
Device to use the Robotera XHand1 with yarp

# xhand sdk installation
Download the ``xhand_control_sdk_x86_64_v146.tar.gz`` file from [ergocub2-design-hand/robotera/xhand1/SDK/C++](https://github.com/mesh-iit/ergocub2-design-hand/tree/master/robotera/xhand1/SDK/C++) and unzip it.



Set the following env variables environment variable to point to the unzipped folder,
```bash
export XHAND_SDK_PATH=/path/to/xhand_control_sdk_x86_64_v146
export xhand_control_DIR=$XHAND_SDK_PATH/xhand_control_sdk/share/xhand_control/cmake
export LD_LIBRARY_PATH=$XHAND_SDK_PATH/xhand_control_sdk/lib:$LD_LIBRARY_PATH
```

# yarp-device: build/install and environments configurations

```
cd /<path to yarp-device-xhand repository>
mkdir build & cd build
cmake .. -DALLOW_DEVICE_PARAM_PARSER_GENERATION=ON -DCMAKE_INSTALL_PREFIX=/<path to yarp-device-xhand install dir>
make install -j4
export PATH=$PATH:/<path to yarp-device-xhand install dir>/bin
export YARP_DATA_DIRS=$YARP_DATA_DIRS:/<path to yarp-device-xhand install dir>/share/yarp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH>:/<path to yarp-device-xhand install dir>/lib
```

# ethercat configurations

```
echo "$XHAND_SDK_PATH/xhand_control_sdk/lib" | sudo tee -a /etc/ld.so.conf.d/yarp.conf
echo "/<path to yarp-device-xhand install dir>/lib" | sudo tee -a /etc/ld.so.conf.d/yarp.conf
echo "/<path to robotology-superbuild install dir>/lib" | sudo tee -a /etc/ld.so.conf.d/yarp.conf
sudo ldconfig
sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep' "$(which yarprobotinterface)"
```

# yarpmotorgui

Open the `yarprobotinterface` using the config file [yarprobotinterface.ini](src/devices/xHandControlBoard/robots-configuration/xhand1/yarprobotinterface.ini)

```
yarprobotinterface --from src/devices/xHandControlBoard/robots-configuration/xhand1/yarprobotinterface.ini
```
If needed, the yarpmotorgui:

```
yarpmotorgui --robot robotera --parts "(xhand)"
```

