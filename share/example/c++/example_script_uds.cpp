#include <thread>
#include <fstream>
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/script.h"
#ifdef WIN32
#include <Windows.h>
#endif
using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using namespace std;

/**
 * Function: Run script program
 * Steps:
 * Step 1: Connect to SCRIPT service and log in to the robot arm
 * Step 2: Enter the script file name
 * Step 3: Read the file and run the script program. If opening the file fails, exit the program.
 */
// This example program needs to send a runnable script to the arcs controller. The test_io.lua file in example/c++ is a provided script. Before running the example_script executable, copy test_io.lua to the executable's directory, then run the example_script executable and enter the lua script name (test_io.lua).
// Before running, please make sure there are no obstacles around the robot arm and it is safe. In case of emergency, press the emergency stop button.

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // Set Windows console output code page to UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto script = std::make_shared<ScriptClient>(1);
    // API call: Connect to SCRIPT service
    script->connect();
    // API call: Log in
    script->login("aubo", "123456");

    // Enter script file name
    char file_name[20];
    cin >> file_name;

    // Open file
    ifstream file;
    file.open(file_name);
    // If opening the file fails, exit the program
    if (!file) {
        cout << "open fail." << endl;
        exit(1);
    }

    // Read and print the contents of the script file
    std::string str_all;
    while (!file.eof()) {
        std::string str_line;
        getline(file, str_line);
        str_all += str_line;
        str_all += "\n";
    }
    str_all += "\r\n\r\n";
    cout << "Script content:" << endl << str_all << endl;
    file.close();

    // Send script content to controller
    script->sendString(str_all);

    // Add blocking to ensure the script is executed
    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

