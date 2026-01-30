import yarp
import asyncio
import os
import signal
import sys
import numpy as np



class EncoderState:
    def __init__(self, encAxes):
        self.axesNames = encAxes
        self.pos = yarp.Vector(len(encAxes), 0.0)

    def __str__(self):
        state_str = "[EncoderState]\n"
        for i, axis in enumerate(self.axesNames):
            state_str += f"  {axis}: {self.pos[i]}\n"
        return state_str


class Module(yarp.RFModule):

    def __init__(self):
        super().__init__()
        self.name = "Module"
        self.running = True
        self.initTime = yarp.now()
        self.HomePosition = None
        # commonConf
        self.period = None
        # remotecontrolboardremapperConf
        self.robot_name = None
        self.axesNames = []
        self.drivers = None
        self.ienc = None
        self.iposDir = None
        # encoderConf
        self.encState = None

    def interruptModule(self):
        print("Module interrupted")
        self.running = False
        return True

    def close(self):
        print("Closing module")
        self.drivers.close()
        self.rpcPort.close()
        return True

    def getPeriod(self):
        return self.period

    def configure(self, rf: yarp.ResourceFinder):
        print("[Module] Configuring module...")

        if not self.commonConf(rf):
            print("Failed to check parameters. See error(s) above.")
            return False

        if not self.remotecontrolboardremapperConf(rf):
            print("Failed to configure drivers. See error(s) above.")
            return False

        if not self.encoderConf():
            print("Failed to configure encoder state. See error(s) above.")
            return False

        print("[Module] Waiting for the interfaces to be ready...")
        yarp.delay(1.0)
        print("[Module] Waiting for the interfaces to be ready...ok")

        self.readEncoders()
        self.HomePosition = yarp.Vector(self.encState.pos)

        print("[Module] Configuring module...OK")

        return True

    def commonConf(self, rf: yarp.ResourceFinder):

        print("[Module] commonConf...")

        config = rf.findFileByName("common.ini")
        if not config:
            print("Configuration file not found.")
            return False
        options = yarp.Property()
        if not options.fromConfigFile(config):
            print("Failed to parse configuration file.")
            return False

        print("Configuration parameters:\n", options.toString())

        if not (options.check("period") and options.find("period").isFloat64()):
            print("Period not specified in the configuration file.")
            return False
        self.period = options.find('period').asFloat64()

        # Print the common parameters
        print(f"Common parameters:")
        print(f"Period: {self.period}")

        print("[Module] commonConf...OK")


        return True

    def remotecontrolboardremapperConf(self, rf: yarp.ResourceFinder):

        print("[Module] remotecontrolboardremapperConf...")

        config = rf.findFileByName("remotecontrolboardremapper.ini")
        if not config:
            print("Configuration file not found.")
            return False
        options = yarp.Property()
        if not options.fromConfigFile(config):
            print("Failed to parse configuration file.")
            return False

        print("Configuration parameters:\n", options.toString())

        if not options.check("robot"):
            print("Robot not specified in the configuration file.")
            return False
        if not options.check("device"):
            print("Device not specified in the configuration file.")
            return False
        if not options.check("localPortPrefix"):
            print("Local port prefix not specified in the configuration file.")
            return False
        if not options.check("remoteControlBoards"):
            print("Remote control boards not specified in the configuration file.")
            return False
        if not options.check("axesNames"):
            print("Axes names not specified in the configuration file.")
            return False

        self.robot_name = options.find('robot').asString()

        for axis in options.find("axesNames").toString().split(" "):
            self.axesNames.append(axis)

        # opening the drivers
        print('Opening the motor driver...')
        self.drivers = yarp.PolyDriver()
        self.drivers.open(options)
        if not self.drivers.isValid():
            print('Cannot open the driver!')
            sys.exit()

        # view the interfaces
        self.ienc = self.drivers.viewIEncoders()
        if self.ienc is None:
            print('Cannot view encoders!')
            sys.exit()

        self.iposDir = self.drivers.viewIPositionDirect()
        if self.iposDir is None:
            print('Cannot view position direct!')
            sys.exit()

        print("[Module] remotecontrolboardremapperConf...OK")

        return True

    def encoderConf(self):

        print("[Module] encoderConf...")

        self.encState = EncoderState(self.axesNames)

        print("[Module] encoderConf...OK")

        return True

    def updateModule(self):

        self.readEncoders()
        self.moveHand()

        print(self.encState)

        return self.running

    def readEncoders(self):
        attempt = 0
        while True:
            if self.ienc.getEncoders(self.encState.pos.data()):
                break
            else:
                attempt += 1
                if attempt == 5:
                    print("Failed to read encoders after multiple attempts.")
                    return False
                yarp.delay(self.period * 0.20)
        print("[readEncoders] Reading attempts :", attempt, "]")

    def moveHand(self):
        joint_to_move = 0
        period = 5.0
        angle = self.HomePosition[joint_to_move] + 5.0 * np.abs(np.sin(2 * np.pi / (period) * (yarp.now() - self.initTime)))
        # self.iposDir.setPosition(joint_to_move, angle)
        positions = yarp.Vector(self.HomePosition)
        positions[joint_to_move] = angle
        self.iposDir.setPositions(positions.data())

        print(f"[moveHand] Moving joint {joint_to_move} to angle {angle} from home position {self.HomePosition[joint_to_move]}")

async def run_module(module: Module, stop_event: asyncio.Event):
    while not stop_event.is_set() and module.updateModule():
        await asyncio.sleep(module.getPeriod())
    module.close()

async def main():

    this_dir = os.path.dirname(os.path.abspath(__file__))
    conf_dir = os.path.join(this_dir, "conf")
    os.environ["YARP_DATA_DIRS"] = conf_dir + ":" + os.environ.get("YARP_DATA_DIRS", "")

    yarp.Network.init()
    rf = yarp.ResourceFinder()
    rf.setDefaultContext(conf_dir)
    rf.configure([])

    module = Module()
    if not module.configure(rf):
        print("Failed to configure the Module module.")
        yarp.Network.fini()
        return

    stop_event = asyncio.Event()

    def shutdown():
        print("\n[Shutdown] SIGINT received, stopping...")
        module.interruptModule()
        stop_event.set()

    loop = asyncio.get_running_loop()
    try:
        loop.add_signal_handler(signal.SIGINT, shutdown)
    except NotImplementedError:
        print("Signal handling not supported in this environment (e.g., Windows or some IDEs).")

    await run_module(module, stop_event)
    yarp.Network.fini()

if __name__ == "__main__":
    asyncio.run(main())