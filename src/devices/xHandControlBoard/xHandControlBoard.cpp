// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "xHandControlBoard.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(CB)
YARP_LOG_COMPONENT(CB, "yarp.device.xHandControlBoard")

yarp::dev::xHandControlBoard::~xHandControlBoard()
{
}

bool yarp::dev::xHandControlBoard::open(yarp::os::Searchable& config)
{
    return false;
}

bool yarp::dev::xHandControlBoard::close()
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& p)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* ps)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* p)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits)
{
    return false;
}

bool yarp::dev::xHandControlBoard::resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getAxes(int* ax)
{
    return false;
}

bool yarp::dev::xHandControlBoard::positionMove(int j, double ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::positionMove(const double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::positionMove(const int n_joints, const int* joints, const double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTargetPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTargetPositions(double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::relativeMove(int j, double delta)
{
    return false;
}

bool yarp::dev::xHandControlBoard::relativeMove(const double* deltas)
{
    return false;
}

bool yarp::dev::xHandControlBoard::relativeMove(const int n_joints, const int* joints, const double* deltas)
{
    return false;
}

bool yarp::dev::xHandControlBoard::checkMotionDone(int j, bool* flag)
{
    return false;
}

bool yarp::dev::xHandControlBoard::checkMotionDone(bool* flag)
{
    return false;
}

bool yarp::dev::xHandControlBoard::checkMotionDone(const int n_joints, const int* joints, bool* flags)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefSpeed(int j, double sp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefSpeeds(const double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefSpeeds(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefAcceleration(int j, double acc)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefAccelerations(const double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefAccelerations(const int n_joints, const int* joints, const double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefSpeed(int j, double* ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefSpeeds(const int n_joints, const int* joints, double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefAcceleration(int j, double* acc)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefAccelerations(const int n_joints, const int* joints, double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::stop(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::stop()
{
    return false;
}

bool yarp::dev::xHandControlBoard::stop(const int n_joints, const int* joints)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getLastJointFault(int j, int& fault, std::string& message)
{
    return false;
}

bool yarp::dev::xHandControlBoard::velocityMove(int j, double v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::velocityMove(const double* v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::resetEncoder(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::resetEncoders()
{
    return false;
}

bool yarp::dev::xHandControlBoard::setEncoder(int j, double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setEncoders(const double* vals)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoder(int j, double* v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoders(double* encs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncodersTimed(double* encs, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoderTimed(int j, double* v, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoderSpeed(int j, double* sp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoderSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoderAcceleration(int j, double* acc)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getEncoderAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getNumberOfMotorEncoders(int* num)
{
    return false;
}

bool yarp::dev::xHandControlBoard::resetMotorEncoder(int m)
{
    return false;
}

bool yarp::dev::xHandControlBoard::resetMotorEncoders()
{
    return false;
}

bool yarp::dev::xHandControlBoard::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setMotorEncoder(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setMotorEncoders(const double* vals)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoder(int m, double* v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoders(double* encs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncodersTimed(double* encs, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderTimed(int m, double* v, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderSpeed(int m, double* sp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderAcceleration(int m, double* acc)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorEncoderAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::enableAmp(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::disableAmp(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getAmpStatus(int* st)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getAmpStatus(int j, int* v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setMaxCurrent(int j, double v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMaxCurrent(int j, double* v)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getNominalCurrent(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setNominalCurrent(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPeakCurrent(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPeakCurrent(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPWM(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPWMLimit(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPWMLimit(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getPowerSupplyVoltage(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setLimits(int j, double min, double max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setVelLimits(int j, double min, double max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getVelLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    return false;
}

bool yarp::dev::xHandControlBoard::isCalibratorDevicePresent(bool* isCalib)
{
    return false;
}

yarp::dev::IRemoteCalibrator* yarp::dev::xHandControlBoard::getCalibratorDevice()
{
    return nullptr;
}

bool yarp::dev::xHandControlBoard::calibrateSingleJoint(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::calibrateWholePart()
{
    return false;
}

bool yarp::dev::xHandControlBoard::homingSingleJoint(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::homingWholePart()
{
    return false;
}

bool yarp::dev::xHandControlBoard::parkSingleJoint(int j, bool _wait)
{
    return false;
}

bool yarp::dev::xHandControlBoard::parkWholePart()
{
    return false;
}

bool yarp::dev::xHandControlBoard::quitCalibrate()
{
    return false;
}

bool yarp::dev::xHandControlBoard::quitPark()
{
    return false;
}

bool yarp::dev::xHandControlBoard::calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setCalibrationParameters(int j, const yarp::dev::CalibrationParameters& params)
{
    return false;
}

bool yarp::dev::xHandControlBoard::calibrationDone(int j)
{
    return false;
}

bool yarp::dev::xHandControlBoard::abortPark()
{
    return false;
}

bool yarp::dev::xHandControlBoard::abortCalibration()
{
    return false;
}

bool yarp::dev::xHandControlBoard::getNumberOfMotors(int* num)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTemperature(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTemperatures(double* vals)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTemperatureLimit(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setTemperatureLimit(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getGearboxRatio(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setGearboxRatio(int m, const double val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getAxisName(int j, std::string& name)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefTorques(double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefTorque(int j, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefTorques(const double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefTorque(int j, double t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setImpedance(int j, double stiff, double damp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setImpedanceOffset(int j, double offset)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTorque(int j, double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTorques(double* t)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTorqueRange(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getTorqueRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getImpedance(int j, double* stiff, double* damp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getImpedanceOffset(int j, double* offset)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getCurrentImpedanceLimit(int j, double* min_stiff, double* max_stiff, double* min_damp, double* max_damp)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getControlMode(int j, int* mode)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getControlModes(int* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getControlModes(const int n_joint, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setControlMode(const int j, const int mode)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setControlModes(const int n_joints, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setControlModes(int* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPosition(int j, double ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPositions(const int n_joints, const int* joints, const double* dpos)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setPositions(const double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefPositions(double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

yarp::os::Stamp yarp::dev::xHandControlBoard::getLastInputStamp()
{
    return yarp::os::Stamp();
}

bool yarp::dev::xHandControlBoard::velocityMove(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefVelocity(const int joint, double* vel)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefVelocities(double* vels)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefDutyCycle(int m, double ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefDutyCycles(const double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefDutyCycle(int m, double* ref)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefDutyCycles(double* refs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getDutyCycle(int m, double* val)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getDutyCycles(double* vals)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getCurrent(int m, double* curr)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getCurrents(double* currs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getCurrentRange(int m, double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getCurrentRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefCurrents(const double* currs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefCurrent(int m, double curr)
{
    return false;
}

bool yarp::dev::xHandControlBoard::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefCurrents(double* currs)
{
    return false;
}

bool yarp::dev::xHandControlBoard::getRefCurrent(int m, double* curr)
{
    return false;
}
