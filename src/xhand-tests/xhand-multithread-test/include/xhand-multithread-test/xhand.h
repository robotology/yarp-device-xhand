#ifndef XHAND_H
#define XHAND_H

#include "xhand_control.hpp"


class xHAND
{
public:
    xHAND() = default;
    ~xHAND() = default;
    bool init(std::string connection, int16_t kp, int16_t ki, int16_t kd, uint16_t tor_max);
    bool updateHandState(bool force_update);
    bool sendCommand(const std::vector<float>& finger_positions);
    xhand_control::ErrorStruct sendCommand(const HandCommand_t& command);
    std::pair<xhand_control::ErrorStruct, HandState_t> readState(bool force_update);
    void homing();
    void close_device();

    void printErrorStruct(const xhand_control::ErrorStruct& err);

    bool test(bool readFirst, bool force_update, const std::vector<float>& finger_positions);

private:
    HandState_t m_state{}, m_stateHome{};
    xhand_control::XHandControl m_ctrl{};
    std::vector<int8_t> m_lowerBound = {0, -60, -10, -10, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<int8_t> m_upperBound = {105, 90, 105, 10, 110, 110, 110, 110, 110, 110, 110, 110};
    uint8_t m_id{0};
    uint16_t m_mode{3}; //Powerless = 0; Powerfull = 3;
    int16_t m_kp{225};
    int16_t m_ki{0};
    int16_t m_kd{12000};
    uint16_t m_tor_max{350};
    std::string class_name_{"xHAND"};
};

#endif /* XHAND_H */
