#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char* argv[])
{
    // Open Arduino device file (Linux)
    std::fstream arduino;
    arduino.open("/dev/ttyACM1", std::ios::in | std::ios::out);

    if (!arduino.is_open()) {
        std::cerr << "Failed to open the device file." << std::endl;
        return 1;
    }

    // Write to it
    arduino << "Hello from C++!" << std::endl;

    // Ensure the write operation is complete
    arduino.flush();

    // Read from it
    std::string response;
    arduino >> response;

    std::cout << "Received: " << response << std::endl;

    arduino.close();

    return 0;
}
