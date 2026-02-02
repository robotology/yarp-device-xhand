# Build, install and environment setup 

```
cd yarp-device-xhand/build
cmake .. -DCMAKE_INSTALL_PREFIX=</path/to/install>
make install -j
export PATH=$PATH:</path/to/install>/bin
export YARP_DATA_DIRS=$YARP_DATA_DIRS:</path/to/install>/share/yarp
```

## Reduce USB latency timer and create `ttyXHAND` symlink 

Copy the [99-xhand.rules](/src/tests/99-xhand.rules) in `/etc/udev/rules.d` to:
- reduce the USB latency timer to `1 ms`;
- create a SYMLINK `ttyXHAND` fot the hand.

## Give the binary the network / realtime capabilities it needs

Grant capabilities so you can run as normal user:

```
sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep' </path/to/install>/bin/xhand-multithread-test
```

Verify:
```
getcap </path/to/install>/bin/xhand-joint-test
```

Remark: every time you install the exe you should repeat the above steps.

## Revoke the binary the network / realtime capabilities

```
sudo setcap -r </path/to/install>/bin/xhand-joint-test
```

# Plot results

The test generates a `.csv` file, plot it with [plotTest](/src/tests/xhand-joint-test/plotTest.py):

```
python3 plotTest.py path/to/file.csv
```
