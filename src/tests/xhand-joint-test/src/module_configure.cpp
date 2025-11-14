#include <xhand-joint-test/module.h>
#include <yarp/os/LogStream.h>


bool Module::configure(yarp::os::ResourceFinder &rf){

    /* Check for and retrieve mandatory groups. */
    auto groupCheckAndRetrieve = [&](const yarp::os::ResourceFinder &rf_in, const std::string &group_name_in, yarp::os::Bottle &bottle_group_out)
    {
        if (!rf_in.check(group_name_in))
        {
            yError() << "[" + class_name_ + "::" + __func__ + "]   Error: mandatory group " + group_name_in + " missing. ";
            return false;
        }

        bottle_group_out = rf_in.findGroup(group_name_in);

        return true;
    };

    yarp::os::Bottle COMMON_bot, HAND_bot;
    if (    !groupCheckAndRetrieve(rf, "COMMON", COMMON_bot)
        ||  !groupCheckAndRetrieve(rf, "HAND", HAND_bot)
    ){
        yError() << "[" + class_name_ + "::" + __func__ + "]  Configuration failed. See error(s) above. ";
        return false;
    }

    if (    !configureCommon(COMMON_bot)
        ||  !configureHand(HAND_bot)
    ){
        yError() << "[" + class_name_ + "::" + __func__ + "]  Configuration failed. See error(s) above. ";
        return false;
    }


    yDebug() << "[" + class_name_ + "::" + __func__ + "] Done.";
    return true;
}


bool Module::configureCommon(const yarp::os::Bottle& bot){

    if(!bot.check("period") || !bot.find("period").isFloat64()){
        yError() << "[" + class_name_ + "::" + __func__ + "] 'period' parameter not found/valid.";
        return false;
    }
    period_ = bot.find("period").asFloat64();

    yDebug() << "[" + class_name_ + "::" + __func__ + "] period: " << period_;

    yDebug() << "[" + class_name_ + "::" + __func__ + "] Done.";

    return true;
}


bool Module::configureHand(const yarp::os::Bottle& bot){

    if( !bot.check("connection") || !bot.find("connection").isString()
    ||  !bot.check("kp") || !bot.find("kp").isInt32()
    ||  !bot.check("ki") || !bot.find("ki").isInt32()
    ||  !bot.check("kd") || !bot.find("kd").isInt32()
    ||  !bot.check("tor_max") || !bot.find("tor_max").isInt32() || bot.find("tor_max").asInt32() < 0
    ){
        yError() << "[" + class_name_ + "::" + __func__ + "] Mandatory parameter(s) missing or invalid.";
        return false;
    }

    xhand_ = std::make_unique<xHAND>();
    xhand_->init(bot.find("connection").asString(),
                 bot.find("kp").asInt16(), bot.find("ki").asInt16(), bot.find("kd").asInt16(),
                 bot.find("tor_max").asInt16());

    yDebug() << "[" + class_name_ + "::" + __func__ + "] Done.";

    return true;
}
