#ifndef XHAND_JOINT_TEST_MODULE_H
#define XHAND_JOINT_TEST_MODULE_H

#include <string>
#include <yarp/os/RFModule.h>
#include "xhand_control.hpp"
#include "xhand-joint-test/xhand.h"


class Module : public yarp::os::RFModule
{
public:
    explicit Module(const std::string &module_name);

    bool configure(yarp::os::ResourceFinder &rf) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

    // bool attach(yarp::os::Port &source) override;

private:

    double period_{1.0};
    const std::string class_name_{};

    std::unique_ptr<xHAND> xhand_;

    bool configureCommon(const yarp::os::Bottle& bot);
    bool configureHand(const yarp::os::Bottle& bot);
    void printErrorStruct(const xhand_control::ErrorStruct& err);
};

#endif /* XHAND_JOINT_TEST_MODULE_H */
