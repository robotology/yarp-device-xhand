#include <xhand-joint-test/module.h>
#include <yarp/os/LogStream.h>

Module::Module(const std::string &module_name): yarp::os::RFModule(),
                                                class_name_(module_name)
{}

bool Module::configure(yarp::os::ResourceFinder &rf){

    yDebug() << "[" + class_name_ + "::" + __func__ + "]";

    if(!rf.check("period") || !rf.find("period").isFloat64()){
        yError() << "[" + class_name_ + "::" + __func__ + "] 'period' parameter not found/valid.";
        return false;
    }
    period_ = rf.find("period").asFloat64();

    xhand_control::XHandControl xhand_control;
    auto ifnames = xhand_control.enumerate_devices("EtherCAT");

    if (ifnames.empty()) {
        std::cout << "No EtherCAT devices found" << std::endl;
        return 0;
    }

    for (const auto& ifname : ifnames) {std::cout << ifname << std::endl;}

    yDebug() << "[" + class_name_ + "::" + __func__ + "] Done.";


    return true;
}

bool Module::close(){return true;}

double Module::getPeriod(){return period_;}

bool Module::interruptModule(){return true;}

bool Module::updateModule(){

    yDebug() << "[" + class_name_ + "::" + __func__ + "]" << yarp::os::Time::now();

    

    return true;
}