#include <iostream>
#include <SerialStream.h>
#include <unistd.h> // for usleep

using namespace LibSerial;

int main() {
    // Open the serial port. Replace "/dev/ttyACM0" with your Arduino's serial port.
    SerialStream serial_port;
    serial_port.Open("/dev/ttyACM1");

    if (!serial_port.IsOpen()) {
        std::cerr << "Failed to open serial port." << std::endl;
        return 1;
    }

    // Configure the serial port
    serial_port.SetBaudRate(SerialStream::BAUD_9600);
    serial_port.SetCharacterSize(SerialStream::CHAR_SIZE_8);
    serial_port.SetParity(SerialStream::PARITY_NONE);
    serial_port.SetStopBits(1);

    // Two integers to send
    int int1 = 123;
    int int2 = 456;

    // Send the integers
    serial_port << int1 << "," << int2 << "\n";

    // Close the serial port
    serial_port.Close();

    return 0;
}
