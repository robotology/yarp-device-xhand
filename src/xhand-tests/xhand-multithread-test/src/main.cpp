// ...existing code...
#include <cstdlib>
#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <xhand-multithread-test/xhand.h>

#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <vector>
#include <cstring>


int main(int argc, char** argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "xhand-multithread-test::main(). Error: YARP network is not available.";
        return EXIT_FAILURE;
    }

    auto connection = "EtherCAT"; // "RS485" or "EtherCAT"
    auto kp = 225;
    auto ki = 0;
    auto kd = 12000;
    auto tor_max = 350;
    auto xhand = std::make_unique<xHAND>();
    if (!xhand->init(connection, kp, ki, kd, tor_max)) {
        yError() << "xhand init failed";
        return EXIT_FAILURE;
    }

    // prepare command structure (zeroed + sensible defaults)
    HandCommand_t command{};
    std::memset(&command, 0, sizeof(command));
    for (uint16_t finger = 0; finger < 12; ++finger) {
        command.finger_command[finger].id = finger;
        command.finger_command[finger].position = 0.0f;
        command.finger_command[finger].kp = static_cast<int16_t>(kp);
        command.finger_command[finger].ki = static_cast<int16_t>(ki);
        command.finger_command[finger].kd = static_cast<int16_t>(kd);
        command.finger_command[finger].mode = 3;
        command.finger_command[finger].tor_max = static_cast<uint16_t>(tor_max);
    }

    std::vector<int64_t> read_timestamps{}, ctrl_timestamps{}, readCycleTime{}, controlCycleTime{};
    const int num_tests{1000000};
    read_timestamps.reserve(num_tests);
    ctrl_timestamps.reserve(num_tests);
    readCycleTime.reserve(num_tests);
    controlCycleTime.reserve(num_tests);

    std::cout << "Starting test of " << num_tests << " calls..." << std::endl;
    std::pair<xhand_control::ErrorStruct, HandState_t>  read_ret;
    xhand_control::ErrorStruct ctrl_ret;


    // Run reader and sender concurrently in separate threads.
    // Each thread records its own durations independently.
    std::atomic<bool> failed{false};

    auto reader = [&](){
        for (int i = 0; i < num_tests; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            read_timestamps.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count()));
            auto local_read = xhand->readState(true);
            if (!local_read.first) {
                failed.store(true, std::memory_order_relaxed);
                break;
            }
            readCycleTime.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) - read_timestamps.back());
        }
    };

    auto sender = [&](){
        for (int i = 0; i < num_tests; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            ctrl_timestamps.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count()));
            auto local_ctrl = xhand->sendCommand(command);
            if (!local_ctrl) {
                failed.store(true, std::memory_order_relaxed);
                break;
            }
            controlCycleTime.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) - ctrl_timestamps.back());
        }
    };

    std::thread thr_reader(reader);
    std::thread thr_sender(sender);
    thr_reader.join();
    thr_sender.join();

    if (failed.load(std::memory_order_relaxed)) {
        std::cerr << "FAIL!" << std::endl;
        return EXIT_FAILURE;
    }
 
     std::cout << "Test completed." << std::endl;
 
     xhand->homing();
     double delay = 1.0;
     std::cout << "Waiting " << delay << " s for the hand to go home...";
     std::this_thread::sleep_for(std::chrono::duration<double>(delay));
     std::cout << "Closing hand device...";
     xhand->close_device();
     // Create a file with the results
     std::ofstream results_file;
     std::string filename = "xhand-multithread-test.csv";
     results_file.open(filename);
     results_file << "ReadTimestamp,ControlTimestamp,readCycleTime,controlCycleTime\n";

    // write rows up to the minimum available samples for each vector
    for (size_t i = 0; i < num_tests; ++i) {
        results_file << read_timestamps[i] << "," << ctrl_timestamps[i] << "," << readCycleTime[i] << "," << controlCycleTime[i] << "\n";
    }
     results_file.close();
     std::cout << "\nResults saved to " << filename << std::endl;
     return EXIT_SUCCESS;
 }
 