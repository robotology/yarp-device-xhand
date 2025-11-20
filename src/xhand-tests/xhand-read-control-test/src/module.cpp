#include <xhand-read-control-test/module.h>
#include <yarp/os/LogStream.h>


Module::Module(const std::string &module_name): yarp::os::RFModule(),
                                                class_name_(module_name)
{}

bool Module::updateModule(){

    static auto tic = std::chrono::high_resolution_clock::now();
    bool readFirst = true;
    bool force_update = true;
    if(!xhand_->test(readFirst, force_update, std::vector<float>(12, 0.0f))){
        yError() << "[" + class_name_ + "::" + __func__ + "] Test Stopped.";
        return false;
    }
    auto toc = std::chrono::high_resolution_clock::now();

    yInfo() << "[" + class_name_ + "::" + __func__ + "] UpdateModule time: " << std::chrono::duration<double, std::milli>(toc - tic).count() << " ms";
    tic = toc;

    return true;
}


bool Module::close(){

    xhand_->homing();
    double delay = 1.0;
    yInfo() << "[" + class_name_ + "::" + __func__ + "] Waiting " << delay << " s for the hand to go home...";
    yarp::os::Time::delay(delay);
    yInfo() << "[" + class_name_ + "::" + __func__ + "] Closing hand device...";
    xhand_->close_device();

    return true;
}


double Module::getPeriod(){return period_;}


bool Module::interruptModule(){return true;}


void Module::printErrorStruct(const xhand_control::ErrorStruct& err){
    yError() << "[" + class_name_ + "::" + __func__ + "]"
    << " Read state failed" << err.error_code << "  " << err.error_message
    << "\n error_type: " << err.error_details.error_type
    << "\n error_name: " << err.error_details.error_name;
}
