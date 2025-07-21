#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "aubo_sdk/rpc.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Implement blocking functionality: The program continues only after the robot reaches the target waypoint
int waitArrival(RobotInterfacePtr impl) {
  const int max_retry_count = 5;
  int cnt = 0;

  // API call: Get the current motion command ID
  int exec_id = impl->getMotionControl()->getExecId();

  // Wait for the robot to start moving
  while (exec_id == -1) {
    if (cnt++ > max_retry_count) {
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    exec_id = impl->getMotionControl()->getExecId();
  }

  // Wait for the robot action to complete
  while (impl->getMotionControl()->getExecId() != -1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}

// Read txt file, search for keyword and extract data
std::vector<double> findKeyword(std::vector<double> data,
                                std::string waypoint) {
  // Read trajectory file
  std::ifstream in("../trajs/path1.txt");
  // Target keyword
  std::string target_keyword;
  target_keyword = waypoint;
  // Store the read line of text
  std::string line;
  // Clear the input data vector
  data.clear();
  // Read file content line by line
  while (std::getline(in, line)) {
    // Check if the current line contains the target keyword
    if (line.find(target_keyword) != std::string::npos) {
      // Locate the content inside the brackets
      int start = line.find("[") + 1;
      int end = line.find("]");
      std::string first_group = line.substr(start, end - start);
      // Define delimiter
      std::string delimiter = ",";
      // Initialize string search position
      size_t pos = 0;
      // Used to store the separated data string
      std::string token;
      // Loop to separate the string and convert it to float, store in data vector
      while ((pos = first_group.find(delimiter)) != std::string::npos) {
        token = first_group.substr(0, pos);
        // Limit the vector size to prevent input data from exceeding 6 due to keyword location
        if (data.size() < 6) {
          data.push_back(std::stod(token));
        }
        // Remove the processed part
        first_group.erase(0, pos + delimiter.length());
      }
      // Process the remaining data
      if (data.size() < 6) {
        data.push_back(std::stod(first_group));
      }
    }
  }
  return data;
}
