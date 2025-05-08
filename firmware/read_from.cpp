#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h> // For sleep function

int main() {
    // Open Arduino device file for reading (Linux)
    std::ifstream arduino("/dev/ttyACM0");

    if (!arduino.is_open()) {
        std::cerr << "Failed to open the device file for reading." << std::endl;
        return 1;
    }

    // Continuously read from the serial port
    while (true) {
        std::string response;
        if (std::getline(arduino, response)) {
            std::cout << "Received: " << response << std::endl;
        } else {
            std::cerr << "No data received." << std::endl;
        }
        sleep(1); // Sleep for 1 second before trying to read again
    }

    arduino.close();

    return 0;
}