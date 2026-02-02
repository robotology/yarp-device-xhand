#include <cstdlib>
#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <xhand-read-test/xhand.h>

int main(int argc, char** argv)
{

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "xhand-read-test::main(). Error: YARP network is not available.";
        return EXIT_FAILURE;
    }

    auto connection = "EtherCAT"; // "RS485" or "EtherCAT"
    auto kp = 225;
    auto ki = 0;
    auto kd = 12000;
    auto tor_max = 350;
    auto xhand = std::make_unique<xHAND>();
    xhand->init(connection, kp, ki, kd, tor_max);

    std::vector<double> durations{};
    const int num_tests{10000};
    durations.reserve(num_tests);

    std::cout << "Starting test of " << num_tests << " sendCommand calls..." << std::endl;
    std::pair<xhand_control::ErrorStruct, HandState_t>  ret;
    std::chrono::high_resolution_clock::time_point t1, t2;

    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_tests; ++i){
        ret = xhand->readState(true);

        if(!ret.first){
            std::cerr << "Send command failed.";
            xhand->printErrorStruct(ret.first);
            return EXIT_FAILURE;
        }
        t2 = std::chrono::high_resolution_clock::now();
        durations.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());
        t1 = t2;
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
    std::string filename = "xhand-read-test.csv";
    results_file.open(filename);
    results_file << "ControlTime,TotalTestTime\n";
    std::cout << "\n\n\nDurations:";
    for(size_t i = 0; i < durations.size(); i++){
        results_file << durations[i] << "," << durations[i] << "\n";
        std::cout << "\nDuration " << i << ": " << durations[i] << " ns";
    }
    results_file.close();
    std::cout << "\nResults saved to " << filename << std::endl;
    return EXIT_SUCCESS;
}
