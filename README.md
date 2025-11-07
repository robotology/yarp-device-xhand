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


