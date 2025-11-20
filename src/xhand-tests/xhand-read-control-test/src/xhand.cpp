#include "xhand-read-control-test/xhand.h"
#include <yarp/os/LogStream.h>

#include <chrono>


bool xHAND::init(std::string connection, int16_t kp, int16_t ki, int16_t kd, uint16_t tor_max)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_tor_max = tor_max;

    yDebug() << "[" + class_name_ + "::" + __func__ + "] Settings:"
    <<"\nConnection: " << connection
    <<"\nID: " << static_cast<int>(m_id)
    <<"\nMode: " << m_mode
    <<"\nKP: " << m_kp
    <<"\tKI: " << m_ki
    <<"\tKD: " << m_kd
    << "\tTorMax: " << m_tor_max;

    // Looking for available devices
    std::vector<std::string> ifnames = m_ctrl.enumerate_devices(connection);
    if (ifnames.empty()) {
        yError() << "[" + class_name_ + "::" + __func__ + "] No " << connection << " devices found";
        return false;
    }
    std::string debugMsg{};
    for (const auto& ifname : ifnames) {debugMsg+=ifname + " ";}
    yDebug() << "[" + class_name_ + "::" + __func__ + "] Devices found:" << debugMsg;

    // Connecting to the first available device
    const int first_device = 0;

    xhand_control::ErrorStruct ret;
    if (connection == "RS485")
        ret = m_ctrl.open_serial("/dev/ttyXHAND", 3000000);
    else if (connection == "EtherCAT")
        ret = m_ctrl.open_ethercat(ifnames[first_device]);
    else{
        yError() << "[" + class_name_ + "::" + __func__ + "] Unknown connection type: " << connection;
        return false;
    }

    if (!ret){
        yError() << "[" + class_name_ + "::" + __func__ + "] Failed to open" + connection;
        printErrorStruct(ret);
        return false;
    }
    yDebug() << "[" + class_name_ + "::" + __func__ + "] Successfully connected to " << ifnames[first_device];

    std::vector<uint8_t> hand_list = m_ctrl.list_hands_id();
    if (hand_list.empty()){
        yError() << "[" + class_name_ + "::" + __func__ + "] No hands found on the network.";
        return false;
    }
    m_id = hand_list[first_device];

    // Give some time to the hand to be ready
    double delay = 1.0;
    yInfo() << "[" + class_name_ + "::" + __func__ + "] Waiting " << delay << " s for the hand to be ready...";
    yarp::os::Time::delay(delay);

    updateHandState(true);
    m_stateHome = m_state;


    return true;
}


bool xHAND::updateHandState(bool force_update){

    auto t1 = std::chrono::high_resolution_clock::now();
    auto ret = m_ctrl.read_state(m_id, force_update);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (!ret.first) {
        printErrorStruct(ret.first);
        yError() << "[" + class_name_ + "::" + __func__ + "] See error(s) above.";
        return false;
    }

    m_state = ret.second;

    // for(const auto& finger: m_state.finger_state){
    //     yInfo() << "Finger ID: " << finger.id
    //             << " \tPosition [deg]: " << finger.position * (180.0/M_PI)
    //             << " \tControl mode: " << (finger.raw_position & 0xf)
    //             << " \tTorque: " << finger.torque
    //             << " \tTemperature: " << finger.temperature;
    // }

    return true;
}


bool xHAND::sendCommand(const std::vector<float>& finger_positions){
    HandCommand_t command;
    memset(&command, 0, sizeof(command));
    for(uint16_t finger_id = 0; finger_id < 12; finger_id++){
        command.finger_command[finger_id].id = finger_id;
        command.finger_command[finger_id].position = finger_positions[finger_id];
        command.finger_command[finger_id].kp = m_kp;
        command.finger_command[finger_id].ki = m_ki;
        command.finger_command[finger_id].kd = m_kd;
        command.finger_command[finger_id].mode = m_mode;
        command.finger_command[finger_id].tor_max = m_tor_max;
    }

    auto err = m_ctrl.send_command(m_id, command);

    if (!err){
        printErrorStruct(err);
        yError() << "[" + class_name_ + "::" + __func__ + "] See error(s) above.";
        return false;
    }

    return true;
}


bool xHAND::test(bool readFirst, bool force_update, const std::vector<float>& finger_positions){

    static const int test_size{1000};
    static std::vector<double> read_time, control_time, test_time;
    read_time.reserve(test_size);
    control_time.reserve(test_size);
    test_time.reserve(test_size);


    HandCommand_t command;
    memset(&command, 0, sizeof(command));
    for(uint16_t finger_id = 0; finger_id < 12; finger_id++){
        command.finger_command[finger_id].id = finger_id;
        command.finger_command[finger_id].position = finger_positions[finger_id];
        command.finger_command[finger_id].kp = m_kp;
        command.finger_command[finger_id].ki = m_ki;
        command.finger_command[finger_id].kd = m_kd;
        command.finger_command[finger_id].mode = m_mode;
        command.finger_command[finger_id].tor_max = m_tor_max;
    }

    xhand_control::ErrorStruct send_command_ret;
    std::pair<xhand_control::ErrorStruct, HandState_t> read_state_ret;
    std::chrono::_V2::system_clock::time_point t1, t2, t2_bis, t3;

    const std::chrono::milliseconds delay_ms{50};

    if(readFirst){
        t1 = std::chrono::high_resolution_clock::now();
        read_state_ret = m_ctrl.read_state(m_id, force_update);
        t2 = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(delay_ms);
        t2_bis = std::chrono::high_resolution_clock::now();
        send_command_ret = m_ctrl.send_command(m_id, command);
        t3 = std::chrono::high_resolution_clock::now();
    }else{
        t1 = std::chrono::high_resolution_clock::now();
        send_command_ret = m_ctrl.send_command(m_id, command);
        t2 = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(delay_ms);
        t2_bis = std::chrono::high_resolution_clock::now();
        read_state_ret = m_ctrl.read_state(m_id, force_update);
        t3 = std::chrono::high_resolution_clock::now();
    }

    if (!read_state_ret.first) {
        printErrorStruct(read_state_ret.first);
        yError() << "[" + class_name_ + "::" + __func__ + "] See error(s) above.";
        return false;
    }
    if (!send_command_ret){
        printErrorStruct(send_command_ret);
        yError() << "[" + class_name_ + "::" + __func__ + "] See error(s) above.";
        return false;
    }

    if(readFirst){
        read_time.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
        control_time.push_back(std::chrono::duration<double, std::milli>(t3 - t2_bis).count());
    }else{
        control_time.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
        read_time.push_back(std::chrono::duration<double, std::milli>(t3 - t2_bis).count());
    }
    test_time.push_back(std::chrono::duration<double, std::milli>(t3 - t2_bis + t2 - t1).count());


    yInfo() << "[" + class_name_ + "::" + __func__ + "]"
    << "force_update" << force_update
    << "\n test/test_size: " << read_time.size() << " / " << test_size;


    if(read_time.size() >= test_size){
        double read_time_1_avg = std::accumulate(read_time.begin(), read_time.end(), 0.0) / read_time.size();
        double control_time_avg = std::accumulate(control_time.begin(), control_time.end(), 0.0) / control_time.size();
        double test_time_avg = std::accumulate(test_time.begin(), test_time.end(), 0.0) / test_time.size();

        double read_time_std = 0.0, control_time_std = 0.0, test_time_std = 0.0;

        for(size_t i = 0; i < test_time.size(); i++){
            read_time_std += std::pow(read_time[i] - read_time_1_avg, 2);
            control_time_std += std::pow(control_time[i] - control_time_avg, 2);
            test_time_std += std::pow(test_time[i] - test_time_avg, 2);
        }

        read_time_std = std::sqrt(read_time_std / read_time.size());
        control_time_std = std::sqrt(control_time_std / control_time.size());
        test_time_std = std::sqrt(test_time_std / test_time.size());

        yInfo() << "[" + class_name_ + "::" + __func__ + "] Average over " << test_size << " tests:"
        << "\nTest type:" << (readFirst ? "Read first" : "Control first")
        << "\nRead state time [ms] - first: " << read_time.front() << " , second last: " << read_time[read_time.size() - 2] << " , last: " << read_time.back()
        << "\nSend command time [ms] - first: " << control_time.front() << " , second last: " << control_time[control_time.size() - 2] << " , last: " << control_time.back()
        << "\nTotal test time [ms] - first: " << test_time.front() << " , second last: " << test_time[test_time.size() - 2] << " , last: " << test_time.back()
        << "\nRead state time [ms] - avg: " << read_time_1_avg << " , std: " << read_time_std
        << "\nSend command time [ms] - avg: " << control_time_avg << " , std: " << control_time_std
        << "\nTotal test time [ms] - avg: " << test_time_avg << " , std: " << test_time_std;

        // Create a file with the results
        std::ofstream results_file;
        std::string filename = "xhand-read-control-test-" + std::string(readFirst ? "read" : "control") + "-first.csv";
        results_file.open(filename);
        results_file << "ReadTime,ControlTime,TotalTestTime\n";
        for(size_t i = 0; i < test_time.size(); i++){
            results_file << read_time[i] << "," << control_time[i] << "," << test_time[i] << "\n";
        }
        results_file.close();
        yInfo() << "[" + class_name_ + "::" + __func__ + "] Results saved to " << filename;

        return false;
    }


    return true;
}


void xHAND::homing(){
    std::vector<float>home(m_stateHome.finger_state.size());
    for (size_t i = 0; i < m_stateHome.finger_state.size(); i++){
        home[i] = m_stateHome.finger_state[i].position;
    }
    if(!sendCommand(home)){
        yError() << "[" + class_name_ + "::" + __func__ + "] Homing failed.";
    }
    else{
        for(size_t i = 0; i < m_stateHome.finger_state.size(); i++){
            home[i]*=(180/M_PI);
        }
        yInfo() << "[" + class_name_ + "::" + __func__ + "] Homing successful [deg]: "<< home;
    }
}


void xHAND::close_device(){
    m_ctrl.close_device();
}


void xHAND::printErrorStruct(const xhand_control::ErrorStruct& err){
    yError() << "[" + class_name_ + "::" + __func__ + "]"
    << " Read state failed" << err.error_code << "  " << err.error_message
    << "\n error_type: " << err.error_details.error_type
    << "\n error_name: " << err.error_details.error_name;
}
