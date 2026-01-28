cd /etc/ld.so.conf.d/yarp.conf
metti il contenuto di LD_LIBRARY_PATH (in realta' dovrebbe essere sufficiente solo:

/home/pasquale/software/xhand1_robotera/xhand_control_sdk_x86_64_v141/xhand_control_sdk/lib
/home/pasquale/software/robotology-superbuild/build/install/lib
/home/pasquale/software/yarp-device-xhand/build/install/lib

o anche meno
)

sudo ldconfig (da fare solo la prima volta) o riavviare il pc

sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep' path/to/yarprobotinterface