import yarp
import asyncio
import os
import signal
import sys
import time
import rerun as rr
from datetime import datetime

class xHandControlBoardTest(yarp.RFModule):
    def __init__(self):
        super().__init__()
        self.name = "xHandControlBoardTest"
        self.running = True
        self.period = 1.0  # seconds
        self.Axes = 12
        self.encMeas = yarp.Vector(self.Axes, 0.0)
        self.rerunRootPath = "/" + self.name

    def getPeriod(self):
        return self.period

    def interruptModule(self):
        print("[" + self.name + "::interruptModule]")
        self.running = False
        return True

    def close(self):
        print("[" + self.name + "::close]")
        return True

    def configure(self, rf: yarp.ResourceFinder):
        print("[" + self.name + "::configure]")
        file = rf.findFile("conf.ini")
        print("Using configuration file:", file)
        options = yarp.Property()
        if not options.fromConfigFile(file):
            print("Failed to parse configuration file.")
            return False

        self._configureDrivers(options)
        self._configureRerun(options)

        return True

    def _configureRerun(self, options: yarp.Property):
        print("[" + self.name + "::_configureRerun]")
        if not options.check("RERUN"):
            print("[" + self.name + "::_configureRerun] RERUN group not found; skipping Rerun initialization.")
            return
        
        rerun_options = options.findGroup("RERUN")

        if not (rerun_options.check("app_id") and rerun_options.find("app_id").isString()):
            print("Missing required parameter: app_id")
            sys.exit()
        if not (rerun_options.check("recording_id") and rerun_options.find("recording_id").isString()):
            print("Missing required parameter: recording_id")
            sys.exit()
        if not (rerun_options.check("time_line") and rerun_options.find("time_line").isString()):
            print("Missing required parameter: time_line")
            sys.exit()

        self.log_time = rerun_options.find("time_line").asString()

        # Get current date and time
        now = datetime.now()

        if not rerun_options.check("log_path"):
            # Create log directory
            log_dir = os.path.dirname(os.path.abspath(__file__)) + "/logs_" + now.strftime("y%Y_m%m_d%d") + "/"
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
        else: 
            if not rerun_options.find("log_path").isString():
                print("Invalid log_path parameter.")
                sys.exit()
            log_dir = rerun_options.find("log_path").asString()
            if not os.path.exists(log_dir):
                print("Log directory does not exist.")
                sys.exit()
            if not log_dir.endswith("/"):
                log_dir += "/"

        print(f"Log directory: {log_dir}")

        #rerun >= v0.24.0
        self.rec  = rr.init(application_id=rerun_options.find("app_id").asString(), recording_id=rerun_options.find("recording_id").asString())
        file_name = log_dir + self.name + "_" + now.strftime("y%Y_m%m_d%d_h%H_min%M_s%S") + ".rrd"
        
        if (rerun_options.check("viewer_ip") and rerun_options.find("viewer_ip").isString()):
            viewer_ip = rerun_options.find("viewer_ip").asString()
            print(f"Viewer IP: {viewer_ip}")
            if (viewer_ip == 'localhost'):
                rr.set_sinks(rr.GrpcSink(), rr.FileSink(file_name))
            else:
                rr.set_sinks(rr.GrpcSink("rerun+http://"+viewer_ip+":9876/proxy"), rr.FileSink(file_name))
        else:
            print("No viewer_ip specified; using FileSink only.")
            rr.save(file_name, self.rec)

        rr.log(self.rerunRootPath, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        return

    def _configureDrivers(self, options: yarp.Property):
        print("[" + self.name + "::_configureDrivers] Opening the motor driver...")
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
        print("[" + self.name + f"::_configureDrivers] Waiting for {delay} seconds to let the device be ready...")
        yarp.delay(delay)

    def updateModule(self):

        # Measure time with high precision clock
        tic = time.perf_counter()
        if not self.ienc.getEncoders(self.encMeas.data()):
            print('Cannot read encoders!')
        toc = time.perf_counter()
        print(f" Time taken to read encoders: {toc - tic} seconds")
        print("Encoder measurements: " + self.encMeas.toString())

        self._logRerun()

        return self.running

    def _logRerun(self):
        now = yarp.now()
        rr.set_time(self.log_time, duration=now)

        for i in range(self.encMeas.size()):
            entity_path = self.rerunRootPath + "/joint_" + str(i)
            rr.log(entity_path, rr.Scalars(self.encMeas[i]))



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