#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{
	//open arduino device file (linux)
    std::ofstream arduino;
	arduino.open( "/dev/ttyACM1");

	//write to it
        arduino << "Hello from C++!";

    // Ensure the write operation is complete
    arduino.flush();

    // Read from it
    std::string response;
    arduino >> response;

    std::cout << "Received: " << response << std::endl;
	arduino.close();

	return 0;
}
