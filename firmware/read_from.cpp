#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h> // For sleep function

int main(int argc, char* argv[])
{
    // Open Arduino device file (Linux)
    std::fstream arduino;
    arduino.open("/dev/ttyACM0", std::ios::in | std::ios::out);

    if (!arduino.is_open()) {
        std::cerr << "Failed to open the device file." << std::endl;
        return 1;
    }

    // Continuously read from the serial port
    while (true) {
        std::string response;
        if (arduino >> response) {
            std::cout << "Received: " << response << std::endl;
        } else {
            std::cerr << "No data received." << std::endl;
        }
        sleep(1); // Sleep for 1 second before trying to read again
    }

    arduino.close();

    return 0;
}