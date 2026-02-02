# yarp-device-xhand
Device to use the Robotera XHand1 with yarp

# xhand sdk installation
Download the ``xhand_control_sdk_x86_64_v141.tar.gz`` file from https://di6kz6gamrw.feishu.cn/drive/folder/WGyhflqb1lRu9ddtc0scjDhwngg and unzip it.
Set the following env variables environment variable to point to the unzipped folder,
```bash
export XHAND_SDK_PATH=/path/to/xhand_control_sdk_x86_64_v141
export xhand_control_DIR=XHAND_SDK_PATH/xhand_control_sdk/share/xhand_control/cmake
export LD_LIBRARY_PATH=$XHAND_SDK_PATH/xhand_control_sdk/lib:$LD_LIBRARY_PATH
```

# yarp-device environments configurations

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH>:"/<path to yarp-device-xhand install dir>/lib"
```

# ethercat configurations

```
sudo echo "$XHAND_SDK_PATH/xhand_control_sdk/lib" >> /etc/ld.so.conf.d/yarp.conf
sudo echo "/<path to yarp-device-xhand install dir>/lib" >> /etc/ld.so.conf.d/yarp.conf
sudo echo "/<path to path to robotology-superbuild install dir>/lib">>/etc/ld.so.conf.d/yarp.conf
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

