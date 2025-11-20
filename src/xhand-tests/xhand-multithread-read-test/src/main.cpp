// ...existing code...
#include <cstdlib>
#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <xhand-multithread-read-test/xhand.h>

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
        yError() << "xhand-multithread-read-test::main(). Error: YARP network is not available.";
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

    std::vector<double> read_durations, read_2_durations{};
    const int num_tests{1000000};
    read_durations.reserve(num_tests);
    read_2_durations.reserve(num_tests);

    std::cout << "Starting test of " << num_tests << " calls..." << std::endl;


    // Run reader and reader_2 concurrently in separate threads.
    // Each thread records its own durations independently.
    std::atomic<bool> failed{false};

    auto reader = [&](){
        auto t_prev = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_tests; ++i) {
            auto local_read = xhand->readState(true);
            if (!local_read.first) {
                failed.store(true, std::memory_order_relaxed);
                break;
            }
            auto t_now = std::chrono::high_resolution_clock::now();
            read_durations.push_back(
                static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t_now - t_prev).count())
            );
            t_prev = t_now;
        }
    };

    auto reader_2 = [&](){
        auto t_prev = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_tests; ++i) {
            auto local_read = xhand->readState(true);
            if (!local_read.first) {
                failed.store(true, std::memory_order_relaxed);
                break;
            }
            auto t_now = std::chrono::high_resolution_clock::now();
            read_2_durations.push_back(
                static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t_now - t_prev).count())
            );
            t_prev = t_now;
        }
    };

    std::thread thr_reader(reader);
    std::thread thr_reader_2(reader_2);
    thr_reader.join();
    thr_reader_2.join();

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
     std::string filename = "xhand-multithread-read-test.csv";
     results_file.open(filename);
     results_file << "Read1Time,Read2Time\n";
     std::cout << "\n\nRead1Time\tRead2Time:";

    // write rows up to the minimum available samples for each vector
    size_t rows = std::min(read_durations.size(), read_2_durations.size());
    for (size_t i = 0; i < rows; ++i) {
        results_file << read_durations[i] << "," << read_2_durations[i] << "\n";
        std::cout << "\nDuration " << i << ": " << read_durations[i] << " ns" << "\t" << read_2_durations[i] << " ns";
    }
     results_file.close();
     std::cout << "\nResults saved to " << filename << std::endl;
     return EXIT_SUCCESS;
 }
 