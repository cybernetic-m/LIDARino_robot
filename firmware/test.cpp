#include <iostream>
#include <SerialStream.h>
#include <unistd.h> // for usleep

using namespace LibSerial;

int main() {
    // Open the serial port. Replace "/dev/ttyACM0" with your Arduino's serial port.
    SerialStream serial_port;
    serial_port.Open("/dev/ttyACM0");

    // Configure the serial port
    serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
    serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
    serial_port.SetNumOfStopBits(1);

    // Two integers to send
    int int1 = 123;
    int int2 = 456;

    // Send the integers
    serial_port << int1 << "," << int2 << "\n";

    // Close the serial port
    serial_port.Close();

    return 0;
}