import yarp
import asyncio
import os
import signal
import sys

class xHandControlBoardTest(yarp.RFModule):
    def __init__(self):
        super().__init__()
        self.name = "xHandControlBoardTest"
        self.running = True
        self.period = 1.0  # seconds
        self.Axes = 12

    def getPeriod(self):
        return self.period

    def interruptModule(self):
        print( "[" + self.name + "::interruptModule]")
        self.running = False
        return True

    def close(self):
        print("[" + self.name + "::close]")
        return True

    def configure(self, rf: yarp.ResourceFinder):
        print("[" + self.name + "::configure]")
        options = yarp.Property()
        file = rf.findFile("conf.ini")
        print("Using configuration file:", file)
        if not options.fromConfigFile(file):
            print("Failed to parse configuration file.")
            return False

        # opening the drivers
        print('Opening the motor driver...')
        self.drivers = yarp.PolyDriver()
        self.drivers.open(options)
        if not self.drivers.isValid():
            print('Cannot open the driver!')
            sys.exit()

        self.ienc = self.drivers.viewIEncoders()
        if self.ienc is None:
            print('Cannot view encoders!')
            sys.exit()

        delay = 1.0
        print(f'Waiting for {delay} seconds to let the device be ready...')
        yarp.delay(delay)

        return True

    def updateModule(self):

        encMeas = yarp.Vector(self.Axes, 0.0)
        if not self.ienc.getEncoders(encMeas.data()):
            print('Cannot read encoders!')

        print('Encoder measurements:', encMeas.toString())

        return self.running



async def run_module(module: xHandControlBoardTest, stop_event: asyncio.Event):
    while not stop_event.is_set() and module.updateModule():
        await asyncio.sleep(module.getPeriod())
    module.close()



async def main():

    this_dir = os.path.dirname(os.path.abspath(__file__))
    os.environ["YARP_DATA_DIRS"] = this_dir + ":" + os.environ.get("YARP_DATA_DIRS", "")

    yarp.Network.init()
    rf = yarp.ResourceFinder()
    rf.setDefaultContext(this_dir)
    rf.configure([])

    module = xHandControlBoardTest()
    if not module.configure(rf):
        print("Failed to configure the logger module.")
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